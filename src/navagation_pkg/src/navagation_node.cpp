//navagation_node.cpp
//aakash bajaj
#include <chrono>
#include <cstdlib>
#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Include SparkMax and service definitions from interfaces
#include "SparkMax.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"

// Include camera node from vision_pkg
#include "vision_pkg/rs_camera_node.hpp"

// Include localization node from localization_pkg
#include "localization_pkg/localization_node.hpp"

using namespace std::chrono_literals;

static constexpr double MAX_VELOCITY = 18.0;  // maximum allowed velocity

// Updated state machine enumeration.
enum class AutonomousState {
  SPIN_LOG_ZONES,      // Spin to log excavation (digging) and dumping (deposit) coordinates.
  SPIN_FOR_TAG1,       // Spin while searching for April Tag 1 (should be behind the robot).
  SPIN_90_TAG1,        // Perform a 90° right turn after Tag1 detection.
  SPIN_FOR_TAG2,       // Spin while searching for April Tag 2 (should appear on the right).
  SPIN_90_TAG2,        // Perform a 90° right turn after Tag2 detection (robot will then face forward).
  NAVIGATE_TO_DIGGING, // navigate toward the logged digging location
  DIGGING,             // DO DA the digging routine
  NAVIGATE_TO_DEPOSIT, // goto  the logged deposit location
  FINISHED             // halt all motion.
};

// helper: clamp a velocity value between -MAX_VELOCITY and MAX_VELOCITY. -> i think this makes sense idk im fucking fried its 2:42 am please god save me
double clamp_velocity(double v) {
  if (v > MAX_VELOCITY) return MAX_VELOCITY;
  if (v < -MAX_VELOCITY) return -MAX_VELOCITY;
  return v;
}

class AutonomousNode : public rclcpp::Node, public std::enable_shared_from_this<AutonomousNode> {
public:
  AutonomousNode()
  : Node("autonomous_node"),
    state_(AutonomousState::SPIN_LOG_ZONES),
    obstacle_detected_(false),
    obstacle_distance_(std::numeric_limits<float>::infinity()),
    digging_pose_logged_(false),
    deposit_pose_logged_(false),
    april_tag_detected_(false)
  {
    // here we create the drive motor objects
    left_motor_ = std::make_shared<SparkMax>("can0", 1);
    right_motor_ = std::make_shared<SparkMax>("can0", 2);
    // create the actuator for tilting the bucket (for deposit routine)
    tilt_actuator_ = std::make_shared<SparkMax>("can0", 5);

    // YOOOOO LEMME SUB TO DA LASERSCAN
    obstacle_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&AutonomousNode::obstacle_callback, this, std::placeholders::_1));

    // localization sub (localization pose topic)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose", 10,
      std::bind(&AutonomousNode::pose_callback, this, std::placeholders::_1));

    // added apriltag sub topic (publishes Bool when an April tag is detected)
    apriltag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/localization/apriltag", 10,
      std::bind(&AutonomousNode::apriltag_callback, this, std::placeholders::_1));

    // 100ms control timer for main function.
    timer_ = this->create_wall_timer(100ms, std::bind(&AutonomousNode::control_loop, this));

    // create service clients for excavation and depositing nodes from interfaces_pkg
    excavation_client_ = this->create_client<interfaces_pkg::srv::DepositingRequest>("excavation_service");
    depositing_client_ = this->create_client<interfaces_pkg::srv::DepositingRequest>("depositing_service");

    spin_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Autonomous node started in SPIN_LOG_ZONES state.");
  }

private:
  // obstacle callback -> this is where we get the laser scan data and check for obstacles
  void obstacle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_distance = std::numeric_limits<float>::infinity();
    // defining a front sector i used -30 -> 30 degrees, i hope it fucking works if it doesnt fuck you i hate fucking programming man
    float sector_min = -30.0f * M_PI / 180.0f;
    float sector_max = 30.0f * M_PI / 180.0f;
    for (size_t i = 0; i < msg->ranges.size(); i++) {
      float angle = msg->angle_min + i * msg->angle_increment;
      if (angle >= sector_min && angle <= sector_max) {
        if (msg->ranges[i] < min_distance) {
          min_distance = msg->ranges[i];
        }
      }
    }
    if (min_distance < 1.0f) {
      obstacle_detected_ = true;
      obstacle_distance_ = min_distance;
    } else {
      obstacle_detected_ = false;
    }
  }

  // update current pose from loclization via pose callback
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
  }

  // april tag callback.
  void apriltag_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      april_tag_detected_ = true;
      RCLCPP_INFO(this->get_logger(), "April Tag detected in current state.");
    }
  }

  // helper: compute Euclidean distance between two poses which is in 2d, i think this makes sense, i saw itin a cool math book i read one time
  double compute_distance(const geometry_msgs::msg::PoseStamped &pose1,
                          const geometry_msgs::msg::PoseStamped &pose2) {
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // helper: compute heading from current to target in rAdians 
  double compute_heading(const geometry_msgs::msg::PoseStamped &current,
                         const geometry_msgs::msg::PoseStamped &target) {
    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;
    return std::atan2(dy, dx);
  }

  // helper: extract yaw from posestamped quaternion orientation
  // this is a function that converts quaternion to yaw pretty sexy right i know im so cool
  double get_yaw(const geometry_msgs::msg::PoseStamped &pose) {
    double siny_cosp = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
                              pose.pose.orientation.x * pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y +
                                    pose.pose.orientation.z * pose.pose.orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // HERE WE GO BOYS ((( MAIN LOOP )))
  // this is the main loop that runs every 100ms and controls the motors
  void control_loop() {
    // if an obstacle is detected, override with avoiding it, call it avoidance manueever
    if (obstacle_detected_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m, executing avoidance maneuver.", obstacle_distance_);
      left_motor_->Heartbeat();
      right_motor_->Heartbeat();
      left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
      right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
      return;
    }

    try {
      switch (state_) {

          // spin in place using opposite velocities and log coordinates for excvation/depositng
          case AutonomousState::SPIN_LOG_ZONES: {
          left_motor_->Heartbeat();
          right_motor_->Heartbeat();
          // spinnny in place.
          left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
          right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));

          rclcpp::Time now = this->now();
          double elapsed = (now - spin_start_time_).seconds();

          //it will take 5 seconds, log the current pose as the digging location.
          if (elapsed > 5.0 && !digging_pose_logged_ && current_pose_.header.stamp.sec != 0) {
            digging_pose_ = current_pose_;
            digging_pose_logged_ = true;
            RCLCPP_INFO(this->get_logger(), "Logged excavation coordinates: [%.2f, %.2f]",
                        digging_pose_.pose.position.x, digging_pose_.pose.position.y);
          }
          // it will take 10 seconds, log the current pose as the deposit location.
          if (elapsed > 10.0 && !deposit_pose_logged_ && current_pose_.header.stamp.sec != 0) {
            deposit_pose_ = current_pose_;
            deposit_pose_logged_ = true;
            RCLCPP_INFO(this->get_logger(), "Logged deposit coordinates: [%.2f, %.2f]",
                        deposit_pose_.pose.position.x, deposit_pose_.pose.position.y);
          }
          // once both coordinates are logged, lets move to searching for april tags
          if (digging_pose_logged_ && deposit_pose_logged_) {
            RCLCPP_INFO(this->get_logger(), "Coordinates logged. Transitioning to SPIN_FOR_TAG1.");
            // reset the flag and timer
            spin_start_time_ = this->now();
            state_ = AutonomousState::SPIN_FOR_TAG1;
          }
          break;
        }
        // spin search for tag1
        case AutonomousState::SPIN_FOR_TAG1: {
          left_motor_->Heartbeat();
          right_motor_->Heartbeat();
          left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
          right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
          if (april_tag_detected_) {
            double current_yaw = get_yaw(current_pose_);
            spin_start_yaw_ = current_yaw;
            // 90 degree right turn -> substract pi/2
            target_yaw_ = current_yaw - (M_PI / 2);
            if (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
            if (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
            RCLCPP_INFO(this->get_logger(), "Tag1 detected. Initiating 90° right turn: current_yaw=%.2f, target_yaw=%.2f", current_yaw, target_yaw_);
            april_tag_detected_ = false;
            state_ = AutonomousState::SPIN_90_TAG1;
          }
          break;
        }
        // 90 turn for tag1
        case AutonomousState::SPIN_90_TAG1: {
          left_motor_->Heartbeat();
          right_motor_->Heartbeat();
          double current_yaw = get_yaw(current_pose_);
          double error = target_yaw_ - current_yaw;
          while (error > M_PI) error -= 2 * M_PI;
          while (error < -M_PI) error += 2 * M_PI;
          if (std::fabs(error) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Completed 90° right turn for Tag1.");
            // lets start looking for tag2 
            spin_start_time_ = this->now(); // reset timer
            state_ = AutonomousState::SPIN_FOR_TAG2;
          } else {
            left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
            right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
          }
          break;
        }
        // spin to search for tag2 (apirltag2)
        case AutonomousState::SPIN_FOR_TAG2: {
          left_motor_->Heartbeat();
          right_motor_->Heartbeat();
          left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
          right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
          if (april_tag_detected_) {
            double current_yaw = get_yaw(current_pose_);
            spin_start_yaw_ = current_yaw;
            // another 90 degree right turn: subtract pi/2
            target_yaw_ = current_yaw - (M_PI / 2);
            if (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
            if (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
            RCLCPP_INFO(this->get_logger(), "Tag2 detected. Initiating second 90° right turn: current_yaw=%.2f, target_yaw=%.2f", current_yaw, target_yaw_);
            april_tag_detected_ = false;
            state_ = AutonomousState::SPIN_90_TAG2;
          }
          break;
        }
        // 2nd 90 degree turn
        case AutonomousState::SPIN_90_TAG2: {
          left_motor_->Heartbeat();
          right_motor_->Heartbeat();
          double current_yaw = get_yaw(current_pose_);
          double error = target_yaw_ - current_yaw;
          while (error > M_PI) error -= 2 * M_PI;
          while (error < -M_PI) error += 2 * M_PI;
          if (std::fabs(error) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Completed second 90° right turn. Facing forward. Transitioning to NAVIGATE_TO_DIGGING.");
            spin_start_time_ = this->now(); // reset timer for navigation logging
            state_ = AutonomousState::NAVIGATE_TO_DIGGING;
          } else {
            left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
            right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
          }
          break;
        }
        case AutonomousState::NAVIGATE_TO_DIGGING: {
          // navagation logic toward the digging location.
          double distance = compute_distance(current_pose_, digging_pose_);
          RCLCPP_INFO(this->get_logger(), "Navigating to excavation spot. Distance: %.2f", distance);
          if (distance < 0.5) {
            left_motor_->SetVelocity(0.0);
            right_motor_->SetVelocity(0.0);
            state_ = AutonomousState::DIGGING;
          } else {
            double desired_heading = compute_heading(current_pose_, digging_pose_);
            double current_yaw = get_yaw(current_pose_);
            double heading_error = desired_heading - current_yaw;
            while (heading_error > M_PI) heading_error -= 2 * M_PI;
            while (heading_error < -M_PI) heading_error += 2 * M_PI;
            //  proportional controller logic.
            double k_forward = 0.3 * MAX_VELOCITY;
            double k_turn = 0.5 * MAX_VELOCITY;
            double forward_speed = k_forward;
            double turn_speed = k_turn * heading_error;
            double left_velocity = clamp_velocity(forward_speed - turn_speed);
            double right_velocity = clamp_velocity(forward_speed + turn_speed);
            left_motor_->SetVelocity(left_velocity);
            right_motor_->SetVelocity(right_velocity);
          }
          break;
        }
        case AutonomousState::DIGGING: {
          // HALT! and execute the digging routine.
          RCLCPP_INFO(this->get_logger(), "Executing excavation routine.");
          left_motor_->SetVelocity(0.0);
          right_motor_->SetVelocity(0.0);
          // call DA excavation service, AND PUT IT ON THE GRILL!!!!!!
          if (!excavation_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Excavation service not available.");
          } else {
            auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
            request->start_depositing = true;  // repurpose field for excavation
            auto future_result = excavation_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_from_this(), future_result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
              auto result = future_result.get();
              if (result->depositing_successful) {
                RCLCPP_INFO(this->get_logger(), "Excavation routine successful.");
              } else {
                RCLCPP_ERROR(this->get_logger(), "Excavation routine failed.");
              }
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to call excavation service.");
            }
          }
          RCLCPP_INFO(this->get_logger(), "Excavation complete. Transitioning to NAVIGATE_TO_DEPOSIT.");
          state_ = AutonomousState::NAVIGATE_TO_DEPOSIT;
          break;
        }
        case AutonomousState::NAVIGATE_TO_DEPOSIT: {
          // okkkk lets go to the  deposit location pretty please!!!
          double distance = compute_distance(current_pose_, deposit_pose_);
          RCLCPP_INFO(this->get_logger(), "Navigating to deposit spot. Distance: %.2f", distance);
          if (distance < 0.5) {
            left_motor_->SetVelocity(0.0);
            right_motor_->SetVelocity(0.0);
            RCLCPP_INFO(this->get_logger(), "Arrived at deposit location. Calling depositing service.");
            if (!depositing_client_->wait_for_service(5s)) {
              RCLCPP_ERROR(this->get_logger(), "Depositing service not available.");
            } else {
              auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
              request->start_depositing = true;
              auto future_result = depositing_client_->async_send_request(request);
              if (rclcpp::spin_until_future_complete(shared_from_this(), future_result) ==
                  rclcpp::FutureReturnCode::SUCCESS)
              {
                auto result = future_result.get();
                if (result->depositing_successful) {
                  RCLCPP_INFO(this->get_logger(), "Depositing routine successful.");
                } else {
                  RCLCPP_ERROR(this->get_logger(), "Depositing routine failed.");
                }
              } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call depositing service.");
              }
            }
            state_ = AutonomousState::FINISHED;
          } else {
            double desired_heading = compute_heading(current_pose_, deposit_pose_);
            double current_yaw = get_yaw(current_pose_);
            double heading_error = desired_heading - current_yaw;
            while (heading_error > M_PI) heading_error -= 2 * M_PI;
            while (heading_error < -M_PI) heading_error += 2 * M_PI;
            double k_forward = 0.3 * MAX_VELOCITY;
            double k_turn = 0.5 * MAX_VELOCITY;
            double forward_speed = k_forward;
            double turn_speed = k_turn * heading_error;
            double left_velocity = clamp_velocity(forward_speed - turn_speed);
            double right_velocity = clamp_velocity(forward_speed + turn_speed);
            left_motor_->SetVelocity(left_velocity);
            right_motor_->SetVelocity(right_velocity);
          }
          break;
        }
        case AutonomousState::FINISHED: {
          left_motor_->SetVelocity(0.0);
          right_motor_->SetVelocity(0.0);
          RCLCPP_INFO(this->get_logger(), "Autonomous routine complete.");
          break;
        }
      }
      
      // logging al velocities here
      float left_vel = left_motor_->GetVelocity();
      float right_vel = right_motor_->GetVelocity();
      RCLCPP_INFO(this->get_logger(), "Left Velocity: %.2f, Right Velocity: %.2f", left_vel, right_vel);
      
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Error in control loop: %s", ex.what());
    }
  }

  // member variables
  AutonomousState state_;
  bool obstacle_detected_;
  float obstacle_distance_;
  rclcpp::Time spin_start_time_;

  // pose variables
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped digging_pose_;
  geometry_msgs::msg::PoseStamped deposit_pose_;
  bool digging_pose_logged_;
  bool deposit_pose_logged_;

  // variables for handling aopril tag detection and turn.
  bool april_tag_detected_ = false;
  double spin_start_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr apriltag_sub_;

  // ROS2 objeczt
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // service clients for excavtin / dpeositng
  rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr excavation_client_;
  rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;

  // motor/actuator objects
  std::shared_ptr<SparkMax> left_motor_;
  std::shared_ptr<SparkMax> right_motor_;
  std::shared_ptr<SparkMax> tilt_actuator_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // start the autonomous node
  auto autonomous_node = std::make_shared<AutonomousNode>();
  // start the vision node  and the localization node 
  auto camera_node = std::make_shared<Camera>();
  auto localization_node = std::make_shared<LocalizationNode>();

  // runs all nodes at the same time using multigthread execeuturo
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(autonomous_node);
  executor.add_node(camera_node);
  executor.add_node(localization_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

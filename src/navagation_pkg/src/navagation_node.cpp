// This node uses a simple state machine to:
//   1. FORWARD_1: Drive forward for a defined distance (e.g., 2 meters), while avoiding obstacles using SLAM (LaserScan).
//   2. TURN_90: Once the first forward distance is covered, perform a 90° right turn (using yaw from localization).
//   3. FORWARD_2: Then drive forward for the same distance while avoiding obstacles.
//   4. FINISHED: Stop all motion and shut down.
//
// Obstacle avoidance: In a forward state, if an obstacle is detected within the forward cone (–30° to +30°)
// and within OBSTACLE_THRESHOLD, the robot overrides its forward command with a turning (avoidance) maneuver.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Include your SparkMax motor controller interface.
#include "SparkMax.hpp"

using namespace std::chrono_literals;

static constexpr double MAX_VELOCITY       = 1.0;      // Commanded forward velocity (m/s)
static constexpr double TURN_VELOCITY        = 0.5;      // Turning velocity (m/s)
static constexpr double FORWARD_DISTANCE     = 2.0;      // Distance to drive forward in each forward phase (meters)
static constexpr double OBSTACLE_THRESHOLD   = 1.0;      // LaserScan threshold (meters) for obstacles
static constexpr double ANGLE_TOLERANCE      = 0.1;      // Tolerance in radians to determine turn completion
static constexpr double GLOBAL_TIMEOUT       = 15.0;     // Global timeout in seconds

// Define a state machine.
enum class State {
  FORWARD_1,   // Drive forward until FORWARD_DISTANCE is reached.
  TURN_90,     // Turn 90° right.
  FORWARD_2,   // Drive forward again for FORWARD_DISTANCE.
  FINISHED     // Stop.
};

class BasicDistanceTurnNavigationNode : public rclcpp::Node {
public:
  BasicDistanceTurnNavigationNode()
  : Node("navagation_node"),
    state_(State::FORWARD_1),
    integrated_distance_(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Node Started.");

    // Initialize drivetrain motor objects.
    left_motor_  = std::make_shared<SparkMax>("can0", 1);
    right_motor_ = std::make_shared<SparkMax>("can0", 2);

    // Subscribe to LaserScan for obstacle detection.
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&BasicDistanceTurnNavigationNode::laser_callback, this, std::placeholders::_1)
    );

    // Subscribe to localization pose to get current yaw.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose", 10,
      std::bind(&BasicDistanceTurnNavigationNode::pose_callback, this, std::placeholders::_1)
    );

    // Set up the control loop timer (100 ms).
    last_time_ = this->now();
    timer_ = this->create_wall_timer(100ms, std::bind(&BasicDistanceTurnNavigationNode::control_loop, this));
  }

private:
  // LaserScan callback: processes the forward cone (-30° to +30°).
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_distance = std::numeric_limits<float>::infinity();
    float sector_min = -30.0f * M_PI / 180.0f;
    float sector_max = 30.0f * M_PI / 180.0f;
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); i++) {
      if (angle >= sector_min && angle <= sector_max) {
        if (msg->ranges[i] < min_distance) {
          min_distance = msg->ranges[i];
        }
      }
      angle += msg->angle_increment;
    }
    if (min_distance < OBSTACLE_THRESHOLD) {
      obstacle_detected_ = true;
      obstacle_distance_ = min_distance;
    } else {
      obstacle_detected_ = false;
    }
  }

  // Pose callback: updates current pose from localization.
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
  }

  // Helper: extract yaw (in radians) from a PoseStamped's quaternion.
  double get_yaw(const geometry_msgs::msg::PoseStamped &pose) {
    double siny_cosp = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
                              pose.pose.orientation.x * pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y +
                                    pose.pose.orientation.z * pose.pose.orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // Helper: integrate distance traveled based on commanded forward velocity.
  void integrate_distance(double commanded_velocity, double dt) {
    integrated_distance_ += std::fabs(commanded_velocity) * dt;
  }

  // Main control loop.
  void control_loop() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Global timeout check.
    double total_elapsed = (now - start_time_).seconds();
    if (total_elapsed > GLOBAL_TIMEOUT) {
      left_motor_->SetVelocity(0.0);
      right_motor_->SetVelocity(0.0);
      RCLCPP_INFO(this->get_logger(), "Global timeout reached. Stopping navigation.");
      rclcpp::shutdown();
      return;
    }

    // Override forward state if obstacle detected.
    if ((state_ == State::FORWARD_1 || state_ == State::FORWARD_2) && obstacle_detected_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m; executing avoidance maneuver.", obstacle_distance_);
      // Execute a simple avoidance: turn in place.
      left_motor_->SetVelocity(clamp_velocity(-0.2 * MAX_VELOCITY));
      right_motor_->SetVelocity(clamp_velocity(0.2 * MAX_VELOCITY));
      return;
    }

    // State machine.
    switch (state_) {
      case State::FORWARD_1: {
        // Drive forward for FORWARD_DISTANCE.
        left_motor_->Heartbeat();
        right_motor_->Heartbeat();
        left_motor_->SetVelocity(MAX_VELOCITY);
        right_motor_->SetVelocity(MAX_VELOCITY);
        integrate_distance(MAX_VELOCITY, dt);
        RCLCPP_INFO(this->get_logger(), "FORWARD_1: Integrated Distance = %.2f m", integrated_distance_);
        if (integrated_distance_ >= FORWARD_DISTANCE) {
          // Stop forward motion.
          left_motor_->SetVelocity(0.0);
          right_motor_->SetVelocity(0.0);
          integrated_distance_ = 0.0;
          // Record current yaw for turn reference.
          initial_yaw_ = get_yaw(current_pose_);
          RCLCPP_INFO(this->get_logger(), "Reached %.2f m forward. Transitioning to TURN_90.", FORWARD_DISTANCE);
          state_ = State::TURN_90;
        }
        break;
      }
      case State::TURN_90: {
        // Execute a 90° right turn.
        left_motor_->Heartbeat();
        right_motor_->Heartbeat();
        double current_yaw = get_yaw(current_pose_);
        // Compute target yaw based on initial_yaw_.
        target_yaw_ = initial_yaw_ - (M_PI / 2);
        if (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
        double error = target_yaw_ - current_yaw;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        if (std::fabs(error) < ANGLE_TOLERANCE) {
          left_motor_->SetVelocity(0.0);
          right_motor_->SetVelocity(0.0);
          RCLCPP_INFO(this->get_logger(), "Completed 90° turn. Transitioning to FORWARD_2.");
          integrated_distance_ = 0.0;
          state_ = State::FORWARD_2;
        } else {
          left_motor_->SetVelocity(TURN_VELOCITY);
          right_motor_->SetVelocity(-TURN_VELOCITY);
        }
        break;
      }
      case State::FORWARD_2: {
        // Drive forward for FORWARD_DISTANCE again.
        left_motor_->Heartbeat();
        right_motor_->Heartbeat();
        left_motor_->SetVelocity(MAX_VELOCITY);
        right_motor_->SetVelocity(MAX_VELOCITY);
        integrate_distance(MAX_VELOCITY, dt);
        RCLCPP_INFO(this->get_logger(), "FORWARD_2: Integrated Distance = %.2f m", integrated_distance_);
        if (integrated_distance_ >= FORWARD_DISTANCE) {
          left_motor_->SetVelocity(0.0);
          right_motor_->SetVelocity(0.0);
          RCLCPP_INFO(this->get_logger(), "Completed second forward drive of %.2f m. Navigation complete.", FORWARD_DISTANCE);
          state_ = State::FINISHED;
        }
        break;
      }
      case State::FINISHED: {
        left_motor_->SetVelocity(0.0);
        right_motor_->SetVelocity(0.0);
        break;
      }
    }
    
    // Log motor velocities.
    double left_vel = left_motor_->GetVelocity();
    double right_vel = right_motor_->GetVelocity();
    RCLCPP_INFO(this->get_logger(), "Left Velocity: %.2f, Right Velocity: %.2f", left_vel, right_vel);
  }

  // Helper: clamp velocity.
  double clamp_velocity(double v) {
    if (v > MAX_VELOCITY)
      return MAX_VELOCITY;
    if (v < -MAX_VELOCITY)
      return -MAX_VELOCITY;
    return v;
  }

  // Member variables.
  State state_;
  bool obstacle_detected_ = false;
  float obstacle_distance_ = std::numeric_limits<float>::infinity();
  rclcpp::Time start_time_, last_time_;
  geometry_msgs::msg::PoseStamped current_pose_;
  double integrated_distance_;
  double initial_yaw_;
  double target_yaw_;
  
  // ROS2 subscriptions and timer.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr apriltag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Motor controllers.
  std::shared_ptr<SparkMax> left_motor_;
  std::shared_ptr<SparkMax> right_motor_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Launch the basic navigation node.
  auto navigation_node = std::make_shared<BasicDistanceTurnNavigationNode>();

  // Optionally, you can launch your vision and localization nodes as well:
  auto camera_node = std::make_shared<Camera>();             // From vision_pkg.
  auto localization_node = std::make_shared<LocalizationNode>(); // From localization_pkg.

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(navigation_node);
  executor.add_node(camera_node);
  executor.add_node(localization_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM
const float VIBRATOR_OUTPUT = 1.0f; // Constant value for vibrator output

enum CAN_IDs
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT = 3,
  RIGHT_LIFT = 4,
  TILT = 5,
  VIBRATOR = 6
};

namespace Gp
{
  enum Buttons
  {
    _A = 0,             // Excavation Autonomy
    _B = 1,             // Stop Automation
    _X = 2,             // Excavation Reset
    _Y = 3,             // Deposit Autonomy
    _LEFT_BUMPER = 4,   // Alternate between control modes
    _RIGHT_BUMPER = 5,  // Vibration Toggle
    _LEFT_TRIGGER = 6,  // Safety Trigger
    _RIGHT_TRIGGER = 7, // Safety Trigger
    _WINDOW_KEY = 8,    // Button 8 /** I do not know what else to call this key */
    _D_PAD_UP = 12,     // Lift Actuator UP
    _D_PAD_DOWN = 13,   // Lift Actuator DOWN
    _D_PAD_LEFT = 14,   // Tilt Actuator Up   /** CHECK THESE */
    _D_PAD_RIGHT = 15,  // Tilt Actuator Down /** CHECK THESE */
    _X_BOX_KEY = 16
  };

  enum Axes
  {
    _LEFT_HORIZONTAL_STICK = 0,
    _LEFT_VERTICAL_STICK = 1,
    _RIGHT_HORIZONTAL_STICK = 2,
    _RIGHT_VERTICAL_STICK = 3,
  };
}

class ControllerNode : public rclcpp::Node
{
public:
  /** Function: ControllerNode Constructor
   * @brief ControllerNode class Constructor is passed a string reference of a CAN interface.
   *        It initiallizes the motors by flashing configuration settings to the SparkMaxes.
   *        Furtheremore, this creates subscriptions(2), publishers(3), and a timer for the following, respectively:
   *        joy_topic, health_subscriber, depositing client, excavation client, heartbeat pub, and a timer to publisher heatbeat.
   * @param can_interface The interface used by the operating system to communicate
   *                      on the Controller Area Network, listed under 'ip link list' (i.e., can0)
   * @returns None
   */
  ControllerNode(const std::string &can_interface)
      : Node("controller_node"),
        leftMotor(can_interface, LEFT_MOTOR),
        rightMotor(can_interface, RIGHT_MOTOR),
        leftLift(can_interface, LEFT_LIFT),
        rightLift(can_interface, RIGHT_LIFT),
        tilt(can_interface, TILT),
        vibrator(can_interface, VIBRATOR),
        vibrator_active_(false),
        prev_vibrator_button_(false),
        alternate_mode_active_(false),
        prev_alternate_button_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Begin Initializing Node");

    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");

    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);
    // Initializes the settings for the drivetrain motors

    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);
    // Initializes the settings for the lift actuators

    tilt.SetIdleMode(IdleMode::kBrake);
    tilt.SetMotorType(MotorType::kBrushed);
    tilt.SetSensorType(SensorType::kEncoder);
    // Initializes the settings for the tilt actuator

    vibrator.SetIdleMode(IdleMode::kBrake);
    vibrator.SetMotorType(MotorType::kBrushed);
    vibrator.SetSensorType(SensorType::kEncoder);
    // Initializes the settings fro the vibrator

    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);
    leftLift.SetInverted(true);
    rightLift.SetInverted(true);
    tilt.SetInverted(true);
    vibrator.SetInverted(true);
    // Initializes the inverting status

    leftMotor.SetP(0, 0.0002f);
    leftMotor.SetI(0, 0.0f);
    leftMotor.SetD(0, 0.0f);
    leftMotor.SetF(0, 0.00021f);
    // PID settings for left motor

    rightMotor.SetP(0, 0.0002f);
    rightMotor.SetI(0, 0.0f);
    rightMotor.SetD(0, 0.0f);
    rightMotor.SetF(0, 0.00021f);
    // PID settings for right motor

    leftLift.SetP(0, 1.51f);
    leftLift.SetI(0, 0.0f);
    leftLift.SetD(0, 0.0f);
    leftLift.SetF(0, 0.00021f);
    // PID settings for left lift

    rightLift.SetP(0, 1.51f);
    rightLift.SetI(0, 0.0f);
    rightLift.SetD(0, 0.0f);
    rightLift.SetF(0, 0.00021f);
    // PID settings for right lift

    // PID settings for tilt
    tilt.SetP(0, 1.51f);
    tilt.SetI(0, 0.0f);
    tilt.SetD(0, 0.0f);
    tilt.SetF(0, 0.00021f);
    // PID settings for tilt

    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();
    tilt.BurnFlash();
    vibrator.BurnFlash();
    RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");

    // ---ROS SUBSCRIPTIONS--- //
    RCLCPP_INFO(this->get_logger(), "Initializing Joy Subscription");
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Joy Subscription Initialized");

    health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
        "/health_topic", 10,
        std::bind(&ControllerNode::position_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initializing depositing, excavation, and travel client");
    depositing_client_ = (this->create_client<interfaces_pkg::srv::DepositingRequest>("depositing_service"));
    excavation_client_ = (this->create_client<interfaces_pkg::srv::ExcavationRequest>("excavation_service"));
    RCLCPP_INFO(this->get_logger(), "Excavation, depositing clients initialized");

    RCLCPP_INFO(this->get_logger(), "Initializing Heartbeat Publisher");
    heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);
    RCLCPP_INFO(this->get_logger(), "Heartbeat Publisher Initialized");

    RCLCPP_INFO(this->get_logger(), "Initializing Timer");
    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ControllerNode::publish_heartbeat, this));
    RCLCPP_INFO(this->get_logger(), "Timer Initialized");

    RCLCPP_INFO(this->get_logger(), "Node Initialization Complete");
  }

private:
  // Direct object members.
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;
  rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;
  rclcpp::TimerBase::SharedPtr timer;

  // Autonomy flag
  bool is_autonomy_active_ = false;

  // Vibrator toggle
  bool vibrator_active_;
  bool prev_vibrator_button_;

  // Alternate control mode toggle variables.
  bool alternate_mode_active_ = false;
  bool prev_alternate_button_ = false;

  float left_lift_position = 0.0f;
  float right_lift_position = 0.0f;

  // Helper for stepped output, in velocity control mode it is multiplied by VELOCITY_MAX
  /**
   * @brief Output must be bound within the range [-1.0,1.0].
   *        Returned value will be an integer multiple of 0.25.
   * @param value
   * @returns Step output of value.
   */
  float computeStepOutput(float value)
  {
    float absVal = std::fabs(value);
    if (absVal < 0.25f)
      return 0.0f;

    else if (absVal < 0.5f)
      return (value > 0 ? 0.25f : -0.25f);

    else if (absVal < 0.75f)
      return (value > 0 ? 0.5f : -0.5f);

    else if (absVal < 1.0f)
      return (value > 0 ? 0.75f : -0.75f);

    return (value > 0 ? 1.0f : -1.0f);
  }

  /**
   * @brief Sends request to depositing node and manages response
   * @param None
   * @returns None
   */
  void send_deposit_request()
  {
    if (!depositing_client_ || !depositing_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }
    auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
    request->start_depositing = true;
    depositing_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedFuture future)
                                           {
      auto response = future.get();
      if (response->depositing_successful) {
        RCLCPP_INFO(this->get_logger(), "Depositing successful");
      } else {
        RCLCPP_WARN(this->get_logger(), "Depositing failed");
      } });
  }

  /**
   * @brief Sends request to excavation node and manages node response
   * @param None
   * @returns None
   */
  void send_excavation_request()
  {
    if (!excavation_client_ || !excavation_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }
    auto request = std::make_shared<interfaces_pkg::srv::ExcavationRequest::Request>();
    request->start_excavation = true;
    excavation_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedFuture future)
                                           {
      auto response = future.get();
      if (response->excavation_successful) {
        RCLCPP_INFO(this->get_logger(), "Excavation successful");
      } else {
        RCLCPP_WARN(this->get_logger(), "Excavation failed");
      } });
  }

  /**
   * @brief Keeps track of position for actuator control
   * @param health_msg The received health message from the ROS framework
   * @returns None
   */
  void position_callback(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg)
  {
    left_lift_position = health_msg->left_lift_position;
    right_lift_position = health_msg->right_lift_position;
  }

  /**
   * @brief Manual control callback for the robot. This subscriber callback handles all
   *        control requests received from the joy interface.
   * @param joy_msg A subscription pointer to a joy interface topic.
   * @returns None
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->axes.size() < 2 || joy_msg->buttons.size() < 17)
    {
      RCLCPP_WARN(this->get_logger(), "Insufficient axes/buttons in Joy message");
      return;
    }

    // HEARTBEAT SIGNALS
    try
    { // Sends heartbeats
      leftMotor.Heartbeat();
      rightMotor.Heartbeat();
      leftLift.Heartbeat();
      rightLift.Heartbeat();
      tilt.Heartbeat();
      vibrator.Heartbeat();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending CAN command: %s", ex.what());
    }

    // CANCEL AUTONOMY (B Button)
    if (joy_msg->buttons[Gp::Buttons::_B] > 0)
    {
      std::system("pkill -9 -f depositing_node");
      std::system("pkill -9 -f excavation_node");
      std::system("pkill -9 -f odometry_node");
      // std::system("pkill -9 -f navigation_node");

      std::this_thread::sleep_for(std::chrono::seconds(2)); // Allows time for the nodes to restarted

      std::system("ros2 run controller_pkg excavation_node &");
      std::system("ros2 run controller_pkg depositing_node &");
    }

    // SAFETY LOCK (Right or left trigger)
    bool triggersPressed = (joy_msg->buttons[Gp::Buttons::_LEFT_TRIGGER] > 0 || joy_msg->buttons[Gp::Buttons::_RIGHT_TRIGGER] > 0);

    if (!triggersPressed)
    {
      leftMotor.SetDutyCycle(0.0f);
      rightMotor.SetDutyCycle(0.0f);
      leftLift.SetDutyCycle(0.0f);
      rightLift.SetDutyCycle(0.0f);
      tilt.SetDutyCycle(0.0f);
      return;
    }

    //----------EXCAVATION SYSTEM----------//
    // VIBRATOR TOGGLE (Right bumper)
    bool current_vibrator_button = (joy_msg->buttons[Gp::Buttons::_RIGHT_BUMPER] > 0);
    if (current_vibrator_button && !prev_vibrator_button_)
    {
      vibrator_active_ = !vibrator_active_;
      RCLCPP_INFO(this->get_logger(), "Vibrator toggled %s", vibrator_active_ ? "ON" : "OFF");
    }
    prev_vibrator_button_ = current_vibrator_button;
    float vibrator_duty = vibrator_active_ ? VIBRATOR_OUTPUT : 0.0f;

    vibrator.SetDutyCycle(vibrator_duty);

    // EXCAVATION RESET BUTTON (X button)
    if (joy_msg->buttons[Gp::Buttons::_X] > 0)
    {
      leftLift.SetPosition(0.0f);
      rightLift.SetPosition(0.0f);
      tilt.SetDutyCycle(1.0f);
    }
    else
    {
      // TILT ACTUATOR (D pad left and right)
      float tilt_duty = 0.0f;
      if (joy_msg->buttons[Gp::Buttons::_D_PAD_RIGHT] > 0 && joy_msg->buttons[Gp::Buttons::_D_PAD_DOWN] == 0)
      {
        tilt_duty = 1.0f;
      }
      else if (joy_msg->buttons[Gp::Buttons::_D_PAD_LEFT] > 0 && joy_msg->buttons[Gp::Buttons::_X_BOX_KEY] == 0)
      {
        tilt_duty = -1.0f;
      }
      tilt.SetDutyCycle(tilt_duty);

      const float HIGH_PASS_FILTER = 0.38; // 1/64 inch
      const float KP_LIFT = 9.0f; // Proportional gain
      //...

      // LIFT ACTUATOR (D pad up and down)
      // lift_error = right_lift_position - left_lift_position; error, right minus left

      // ---- LIFT POSITION SYNC ---- //

      // ---- LIFT POSITION SYNC ---- //

      // compute lift error (right - left)
      float lift_error = right_lift_position - left_lift_position;

      // high-pass deadband
      if (fabs(lift_error) < HIGH_PASS_FILTER)
          lift_error = 0.0f;

      // get user lift direction
      float lift_setpoint = 0.0f;
      if (joy_msg->buttons[Gp::Buttons::_D_PAD_UP] > 0)
          lift_setpoint = 1.0f;
      else if (joy_msg->buttons[Gp::Buttons::_D_PAD_DOWN] > 0)
          lift_setpoint = -1.0f;

      // base duties (before correction)
      float left_duty  = lift_setpoint;
      float right_duty = lift_setpoint;

      // proportional correction (always slows the higher side)
      float correction = KP_LIFT * fabs(lift_error);

      // right is higher → slow right
      if (lift_error > 0)
      {
          right_duty = lift_setpoint - correction;
      }
      // left is higher → slow left
      else if (lift_error < 0)
      {
          left_duty = lift_setpoint - correction;
      }

      left_duty  = std::clamp(left_duty,  -1.0f, 1.0f);
      right_duty = std::clamp(right_duty, -1.0f, 1.0f);

      leftLift.SetDutyCycle(left_duty);
      rightLift.SetDutyCycle(right_duty);

    }

    //----------EXCAVATION SYSTEM----------//

    //----------DRIVETRAIN----------//
    // Toggling Alternate Control Mode using the left bumper (button index 4).
    bool current_alternate_button = (joy_msg->buttons[Gp::Buttons::_LEFT_BUMPER] > 0);
    if (current_alternate_button && !prev_alternate_button_)
    {
      alternate_mode_active_ = !alternate_mode_active_;
      RCLCPP_INFO(this->get_logger(), "Alternate control mode %s", alternate_mode_active_ ? "activated" : "deactivated");
    }
    prev_alternate_button_ = current_alternate_button;

    float left_drive = 0.0;
    float right_drive = 0.0;
    float left_drive_raw = 0.0;
    float right_drive_raw = 0.0;

    float left_drive_slow = 0.0;
    float right_drive_slow = 0.0;
    if (alternate_mode_active_)
    { // Left and right joystick, controlled with duty cycle
      float leftJS = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
      float rightJS = -joy_msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];
      left_drive_raw = std::max(-1.0f, std::min(1.0f, leftJS));
      right_drive_raw = std::max(-1.0f, std::min(1.0f, rightJS));

      left_drive = computeStepOutput(left_drive_raw);
      right_drive = computeStepOutput(right_drive_raw);

      leftMotor.SetDutyCycle(left_drive);
      rightMotor.SetDutyCycle(right_drive);
    }

    else
    { // Left joystick, controlled with velocity
      float forward = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
      float turn = fabs(joy_msg->axes[Gp::Axes::_LEFT_HORIZONTAL_STICK]) > 0.25 ? joy_msg->axes[Gp::Axes::_LEFT_HORIZONTAL_STICK] : 0.0f;
      left_drive_raw = forward + turn;
      right_drive_raw = forward - turn;
      left_drive_raw = std::max(-1.0f, std::min(1.0f, left_drive_raw));
      right_drive_raw = std::max(-1.0f, std::min(1.0f, right_drive_raw));

      left_drive = computeStepOutput(left_drive_raw) * VELOCITY_MAX;
      right_drive = computeStepOutput(right_drive_raw) * VELOCITY_MAX;

      float slow_forward = -joy_msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];
      float slow_turn = fabs(joy_msg->axes[Gp::Axes::_RIGHT_HORIZONTAL_STICK]) > 0.25 ? joy_msg->axes[Gp::Axes::_RIGHT_HORIZONTAL_STICK] : 0.0;

      left_drive_slow = slow_forward + slow_turn;
      right_drive_slow = slow_forward - slow_turn;
      left_drive_slow = std::max(-1.0f, std::min(1.0f, left_drive_slow));
      right_drive_slow = std::max(-1.0f, std::min(1.0f, right_drive_slow));

      if (fabs(joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK]) > 0 || fabs(joy_msg->axes[Gp::Axes::_LEFT_HORIZONTAL_STICK]) > 0)
      {
        leftMotor.SetVelocity(left_drive);
        rightMotor.SetVelocity(right_drive);
      }
      else
      {
        leftMotor.SetVelocity(1500 * left_drive_slow);
        rightMotor.SetVelocity(1500 * right_drive_slow);
      }
    }
    //----------DRIVETRAIN----------//

    //----------AUTONOMOUS FUNCTIONS----------//
    // DEPOSIT AUTONOMY (Y button)
    bool current_deposit_button = (joy_msg->buttons[Gp::Buttons::_Y] > 0);
    static bool prev_deposit_button = false;
    if (current_deposit_button && !prev_deposit_button)
    {
      send_deposit_request();
    }
    prev_deposit_button = current_deposit_button;

    // EXCAVATION AUTONOMY (A button)
    bool current_excavate_button = (joy_msg->buttons[Gp::Buttons::_A] > 0);
    static bool prev_excavate_button = false;
    if (current_excavate_button && !prev_excavate_button)
    {
      send_excavation_request();
    }
    prev_excavate_button = current_excavate_button;

    bool current_cycle_button = (joy_msg->buttons[Gp::Buttons::_WINDOW_KEY] > 0);
    static bool prev_cycle_button = false;
    if (current_cycle_button && !prev_cycle_button)
    {
      RCLCPP_INFO(this->get_logger(), "Full cycle launched");
      std::system("ros2 run controller_pkg odometry_node &");
    }
    prev_cycle_button = current_cycle_button;

    // TRAVEL AUTONOMY (Back)
    /*bool current_travel_button = (joy_msg->buttons[9] > 0);
    static bool prev_travel_button = false;
    if (current_travel_button && !prev_travel_button){
      std::system("ros2 run navigation_pkg navigation_node &");
    }
    prev_travel_button = current_travel_button;*/

    //----------AUTONOMOUS FUNCTIONS----------//
  }

  /**
   * @brief Sends periodic heartbeats to Sparkmax motorcontrollers
   * @param None
   * @returns None
   */
  void publish_heartbeat()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Heartbeat";
    heartbeatPub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Heartbeat published");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("controller_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

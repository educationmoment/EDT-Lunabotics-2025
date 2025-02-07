#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <memory>
#include <iostream>

// Test Importing SparkCan -- 
// Note: SparkMax.hpp and SparkBase.hpp should be located in src/<package-name>/include/<package-name>/
#include "SparkMax.hpp"

/* Class: ControllerNode
 *  Methods:
 *      ControllerNode()
 *      topic_callback()
 *      joy_callback()
 *******************************************/

 /* Standard CAN IDs    ID      Motor-Type
  *     Left-Motor:     1       Brushless
  *     Right-Motor:    2       Brushless
  *     Left-Tilt:      3       Brushed?
  *     Right-Tilt:     4       Brushed?
  *     Tilt:           5       Burshed?
  *     Vibrator:       6       Brushed?
  *******************************************/
class ControllerNode : public rclcpp::Node {
public: 
    
    /* ControllerNode::ControllerNode() - Constructor
     *   
     *  Description:
     *
     *
     */
    ControllerNode() : Node("controller_node") {
        // Begin Initiallizing Node
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Hello RCLCPP" );


        // DONE: Create Subscription to /example_interface Topic
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Initiallizing Subscription");
        subscription = this->create_subscription<std_msgs::msg::String>(
            "example_interface",
            10,
            std::bind(&ControllerNode::topic_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO( this->get_logger(), "Subscription Initiallized");

        // TODO: Initiallize Motor Controllers
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Initiallizing Motor Controllers" );
        // motor = std::make_shared<SparkMax>("can0", 47);
        init_motors();
        RCLCPP_INFO( this->get_logger(), "Motor Controllers Initiallized" );


        // DONE: Create Subscription to /joy Topic
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Initiallizing Joy Subscription");
        joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            3,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO( this->get_logger(), "Joy Subscription Initiallized");

        // DONE: Create Publisher to /heartbeat Topic
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Initiallizing /heartbeat Publisher" );
        heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10); 
        RCLCPP_INFO( this->get_logger(), "Initiallized /hearbeat Publisher" );

        // DONE: Create Timer for Heartbeat
        /////////////////////////////////////////////////////////////////////////////////
        RCLCPP_INFO( this->get_logger(), "Initiallizing Timer");
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ControllerNode::publish_heartbeat, this)
        );
        RCLCPP_INFO( this->get_logger(), "Timer Initiallized");


        return;
    }

private:
    // Subscription Callback
    ///////////////////////////
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        return;
    }

    /* Joystick Callback
     *
     * Description:
     *      
     *
     **********************/
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Get Axis 1 and 3
        //      >> Axis 1: Top-Left Joystick
        //      >> Axis 3: Bottom-Right Joystick
        //      >> Button 4: Left Bumpter
        //      >> Button 5: Right Bumper
        ///////////////////////////////////////////////////
        float 
            axis1 = msg->axes[1],
            axis3 = msg->axes[3];
        bool
            bumper_left  = msg->buttons[4],
            bumper_right = msg->buttons[5];
        

        // Scale Duty-Cycle (control.cpp example uses 0.05, use 0.05 and -0.05 as maximum/minimum limits)
        RCLCPP_INFO( this->get_logger(), "Axis 3: %0.3f", axis3);\
        float
            pwm_output_1 = axis1,
            pwm_output_3 = axis3;


        // If Left-Bumper Pressed
        if ( bumper_left ) {
            pwm_output_1 *= 0.12;
        } else {
            pwm_output_1 *= 0.06;
        }

        // If Right-Bumper Pressed
        if ( bumper_right ) {
            pwm_output_3 *= 0.65;
        } else {
            pwm_output_3 *= 0.33;
        }

        // RCLCPP_INFO( this->get_logger(), "[\033[104m\t%0.3f\t|\t%0.3f\t\033[0m]", pwm_output_1, pwm_output_3 );

        // Set Motor Output
        this->motor->Heartbeat();
        this->motor->SetDutyCycle(pwm_output_1);

        // Set Actuator Output
        this->actuator->Heartbeat();
        this->actuator->SetDutyCycle(pwm_output_3);

        RCLCPP_INFO( this->get_logger(), "Sending: %0.3f to motor", pwm_output_1);
        RCLCPP_INFO( this->get_logger(), "Sending: %0.3f to actuator", pwm_output_3);
        
        return;
    }

    // Publisher Callback
    ///////////////////////////
    void publish_heartbeat() {
        auto message = std_msgs::msg::String();
        message.data = "[ controller_node ] Heartbeat";

        heartbeatPub->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
    
    }


    // Initiallize Motors
    ///////////////////////////
    void init_motors() {
        // Configure Motor Interface
        motor = std::make_shared<SparkMax>("can0", 1);
        this->motor->SetIdleMode(IdleMode::kBrake);
        this->motor->SetMotorType(MotorType::kBrushless);
        this->motor->BurnFlash();

        // Configure Actuator Interface
        actuator = std::make_shared<SparkMax>("can0", 3);
        this->actuator->SetIdleMode(IdleMode::kBrake);
        this->actuator->SetMotorType(MotorType::kBrushless);
        this->actuator->BurnFlash();
        return;
    }

    // Subscriber Variables
    ///////////////////////////
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

    // Publisher Variable
    ///////////////////////////
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;

    // Timer Variable
    ///////////////////////////
    rclcpp::TimerBase::SharedPtr timer;

    // Motor Object
    // SparkMax motor;
    std::shared_ptr<SparkMax> motor;
    std::shared_ptr<SparkMax> actuator;
    // std::shared_ptr<SparkMax> motor = new SparkMax("can0", 47);
   
};



/* Function: main()
 *  
 *  Description:
 *
 **/
int main( int argc, char *argv[] ){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
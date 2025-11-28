/**
 * Unitree Go2 Control Node
 * 
 * Bridges ROS2 velocity commands to Unitree Go2 high-level API.
 * This node uses the SportClient to send commands via the robot's
 * built-in sport_mode service.
 * 
 * EDTH Hackathon - Laelaps AI
 * 
 * Subscriptions:
 *   /cmd_vel (geometry_msgs/Twist)     - Velocity commands
 *   /emergency_stop (std_msgs/Bool)    - Emergency stop trigger
 *   /cmd_posture (std_msgs/String)     - Posture commands (up/down/balance/recovery)
 * 
 * Publishes to:
 *   /api/sport/request                 - Unitree API commands
 */

#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/sport_client.hpp"
#include "unitree_api/msg/request.hpp"

// Topic names
#define TOPIC_CMD_VEL "/cmd_vel"
#define TOPIC_ESTOP "/emergency_stop"
#define TOPIC_POSTURE "/cmd_posture"

class Go2ControlNode : public rclcpp::Node {
 public:
  Go2ControlNode()
      : Node("go2_control_node"),
        sport_client_(this),
        estop_active_(false) {
    
    // Velocity command subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        TOPIC_CMD_VEL, rclcpp::QoS(10),
        std::bind(&Go2ControlNode::CmdVelCallback, this, std::placeholders::_1));
    
    // Emergency stop subscriber
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        TOPIC_ESTOP, rclcpp::QoS(10),
        std::bind(&Go2ControlNode::EstopCallback, this, std::placeholders::_1));
    
    // Posture command subscriber
    posture_sub_ = this->create_subscription<std_msgs::msg::String>(
        TOPIC_POSTURE, rclcpp::QoS(10),
        std::bind(&Go2ControlNode::PostureCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(),
                "Go2 Control Node started");
    RCLCPP_INFO(this->get_logger(),
                "  Subscribed to: %s, %s, %s",
                TOPIC_CMD_VEL, TOPIC_ESTOP, TOPIC_POSTURE);
    RCLCPP_INFO(this->get_logger(),
                "  Publishing to: /api/sport/request");
  }

 private:
  /**
   * Handle velocity commands from /cmd_vel
   */
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Ignore commands if emergency stop is active
    if (estop_active_) {
      return;
    }
    
    // Convert ROS Twist to Unitree Move command
    // linear.x = forward velocity (m/s)
    // linear.y = lateral velocity (m/s), positive = left
    // angular.z = yaw rate (rad/s), positive = counter-clockwise
    sport_client_.Move(
        request_,
        static_cast<float>(msg->linear.x),
        static_cast<float>(msg->linear.y),
        static_cast<float>(msg->angular.z));
  }

  /**
   * Handle emergency stop commands
   */
  void EstopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !estop_active_) {
      // Emergency stop activated
      estop_active_ = true;
      RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED - Motors damped!");
      sport_client_.Damp(request_);
    } else if (!msg->data && estop_active_) {
      // Emergency stop released
      estop_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Emergency stop released");
    }
  }

  /**
   * Handle posture commands
   * Commands: "up", "down", "balance", "recovery", "sit", "hello"
   */
  void PostureCallback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& command = msg->data;
    
    if (command == "up" || command == "stand") {
      estop_active_ = false;  // Clear e-stop on stand up
      sport_client_.StandUp(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Stand Up");
      
    } else if (command == "down" || command == "lie") {
      sport_client_.StandDown(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Stand Down");
      
    } else if (command == "balance") {
      sport_client_.BalanceStand(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Balance Stand");
      
    } else if (command == "recovery") {
      estop_active_ = false;  // Clear e-stop on recovery
      sport_client_.RecoveryStand(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Recovery Stand");
      
    } else if (command == "sit") {
      sport_client_.Sit(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Sit");
      
    } else if (command == "hello") {
      sport_client_.Hello(request_);
      RCLCPP_INFO(this->get_logger(), "Action: Hello!");
      
    } else if (command == "stretch") {
      sport_client_.Stretch(request_);
      RCLCPP_INFO(this->get_logger(), "Action: Stretch");
      
    } else if (command == "stop") {
      sport_client_.StopMove(request_);
      RCLCPP_INFO(this->get_logger(), "Posture: Stop Move");
      
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "Unknown posture command: '%s'. Valid: up, down, balance, recovery, sit, hello, stretch, stop",
                  command.c_str());
    }
  }

  // SportClient instance for sending commands
  SportClient sport_client_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  
  // Reusable request message
  unitree_api::msg::Request request_;
  
  // Emergency stop flag
  bool estop_active_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


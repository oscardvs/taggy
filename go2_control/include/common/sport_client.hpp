/**
 * Unitree Go2 Sport Client
 * Provides high-level motion commands via ROS2 DDS interface
 * 
 * Based on Unitree Robotics SDK
 * Adapted for EDTH Hackathon by Laelaps AI
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

// Sport API command IDs
namespace SportAPI {
  constexpr int32_t DAMP = 1001;
  constexpr int32_t BALANCE_STAND = 1002;
  constexpr int32_t STOP_MOVE = 1003;
  constexpr int32_t STAND_UP = 1004;
  constexpr int32_t STAND_DOWN = 1005;
  constexpr int32_t RECOVERY_STAND = 1006;
  constexpr int32_t EULER = 1007;
  constexpr int32_t MOVE = 1008;
  constexpr int32_t SIT = 1009;
  constexpr int32_t RISE_SIT = 1010;
  constexpr int32_t SPEED_LEVEL = 1015;
  constexpr int32_t HELLO = 1016;
  constexpr int32_t STRETCH = 1017;
  constexpr int32_t SWITCH_JOYSTICK = 1027;
}

/**
 * SportClient - High-level motion control for Unitree Go2
 * 
 * Publishes commands to /api/sport/request topic which is handled
 * by the robot's onboard sport_mode service.
 */
class SportClient {
 public:
  explicit SportClient(rclcpp::Node* node);
  
  // ============================================================
  // BASIC MOTION COMMANDS
  // ============================================================
  
  /**
   * Emergency damp - Immediately disable all motors
   * Use in emergency situations only!
   */
  void Damp(unitree_api::msg::Request& req);
  
  /**
   * Balance stand - Robot stands and actively balances
   */
  void BalanceStand(unitree_api::msg::Request& req);
  
  /**
   * Stop all movement - Robot holds position
   */
  void StopMove(unitree_api::msg::Request& req);
  
  /**
   * Stand up from sitting/lying position
   */
  void StandUp(unitree_api::msg::Request& req);
  
  /**
   * Sit/lie down from standing position
   */
  void StandDown(unitree_api::msg::Request& req);
  
  /**
   * Recovery stand - Attempt to stand up from fallen position
   */
  void RecoveryStand(unitree_api::msg::Request& req);
  
  // ============================================================
  // VELOCITY CONTROL
  // ============================================================
  
  /**
   * Move command - Set robot velocity
   * @param vx Forward velocity (m/s), positive = forward
   * @param vy Lateral velocity (m/s), positive = left
   * @param vyaw Yaw rate (rad/s), positive = counter-clockwise
   */
  void Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw);
  
  /**
   * Set body orientation (Euler angles)
   * @param roll Roll angle (rad)
   * @param pitch Pitch angle (rad)
   * @param yaw Yaw angle (rad)
   */
  void Euler(unitree_api::msg::Request& req, float roll, float pitch, float yaw);
  
  /**
   * Set speed level (affects max velocity)
   * @param level Speed level (0-2)
   */
  void SpeedLevel(unitree_api::msg::Request& req, int level);
  
  // ============================================================
  // POSTURE COMMANDS
  // ============================================================
  
  void Sit(unitree_api::msg::Request& req);
  void RiseSit(unitree_api::msg::Request& req);
  
  // ============================================================
  // SPECIAL ACTIONS
  // ============================================================
  
  void Hello(unitree_api::msg::Request& req);
  void Stretch(unitree_api::msg::Request& req);
  
  /**
   * Switch joystick control mode
   * @param flag true = enable remote control, false = disable
   */
  void SwitchJoystick(unitree_api::msg::Request& req, bool flag);

 private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_publisher_;
  rclcpp::Node* node_;
};


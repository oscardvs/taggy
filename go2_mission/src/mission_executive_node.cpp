/**
 * Mission Executive Node
 * 
 * Finite State Machine that orchestrates the Go2 autonomous search-and-find
 * behavior. Manages state transitions between IDLE, LISTENING, SEARCHING,
 * TRACKING, ARRIVED, and FAILED states.
 * 
 * EDTH Hackathon - Oscar Devos
 * 
 * Note: Visual servoing control is delegated to visual_servo_node which
 * subscribes to /mission/state and handles IBVS + LiDAR safety.
 * 
 * Subscriptions:
 *   /wirelesscontroller (unitree_go/WirelessController) - Button trigger
 *   /mission/target_object (std_msgs/String)           - Voice command result
 *   /detection/bbox (vision_msgs/Detection2DArray)     - Object detections
 *   /detection/target_pose (geometry_msgs/PoseStamped) - Target 3D pose (for arrival)
 *   /odom (nav_msgs/Odometry)                          - Robot odometry
 * 
 * Publications:
 *   /cmd_posture (std_msgs/String)     - Posture commands to robot
 *   /mission/state (std_msgs/String)   - Current state for debugging
 * 
 * Action Clients:
 *   /navigate_to_pose (nav2_msgs/NavigateToPose) - Cancel Nav2 goals
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <unitree_go/msg/wireless_controller.hpp>

#include "mission_executive/states.hpp"

using namespace std::chrono_literals;
using namespace mission_executive;

class MissionExecutiveNode : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionExecutiveNode()
      : Node("mission_executive_node"),
        current_state_(State::IDLE),
        state_entry_time_(this->now()),
        target_object_(""),
        last_detection_time_(this->now()),
        prev_keys_(0) {
    
    // Declare parameters
    declare_parameters();
    
    // Get parameters
    listening_timeout_ = this->get_parameter("listening_timeout").as_double();
    target_lost_timeout_ = this->get_parameter("target_lost_timeout").as_double();
    arrived_delay_ = this->get_parameter("arrived_delay").as_double();
    failed_delay_ = this->get_parameter("failed_delay").as_double();
    arrival_distance_ = this->get_parameter("arrival_distance").as_double();
    search_timeout_ = this->get_parameter("search_timeout").as_double();
    
    // Create subscribers
    controller_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::controller_callback, this, std::placeholders::_1));
    
    target_object_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/mission/target_object", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::target_object_callback, this, std::placeholders::_1));
    
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detection/bbox", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::detection_callback, this, std::placeholders::_1));
    
    // Target pose from perception (for accurate depth-based arrival detection)
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/detection/target_pose", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::target_pose_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::odom_callback, this, std::placeholders::_1));
    
    // Subscribe to explore_lite status (exploration complete)
    explore_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/explore/exploring", rclcpp::QoS(10),
        std::bind(&MissionExecutiveNode::explore_status_callback, this, std::placeholders::_1));
    
    // Create publishers
    posture_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_posture", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/mission/state", 10);
    
    // Enable/disable exploration
    explore_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/explore/resume", 10);
    
    // Create Nav2 action client
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    
    // Create state machine timer (10 Hz)
    fsm_timer_ = this->create_wall_timer(
        100ms, std::bind(&MissionExecutiveNode::fsm_update, this));
    
    // Initialize with IDLE state
    enter_state(State::IDLE);
    
    RCLCPP_INFO(this->get_logger(), "Mission Executive Node started");
    RCLCPP_INFO(this->get_logger(), "  Listening timeout: %.1f s", listening_timeout_);
    RCLCPP_INFO(this->get_logger(), "  Target lost timeout: %.1f s", target_lost_timeout_);
    RCLCPP_INFO(this->get_logger(), "  Search timeout: %.1f s (0=disabled)", search_timeout_);
    RCLCPP_INFO(this->get_logger(), "  Arrival distance: %.2f m", arrival_distance_);
    RCLCPP_INFO(this->get_logger(), "  Trigger: L2 + A (Safety + Confirm)");
  }

 private:
  // ============================================================
  // PARAMETER DECLARATION
  // ============================================================
  void declare_parameters() {
    this->declare_parameter("listening_timeout", timeouts::LISTENING_TIMEOUT);
    this->declare_parameter("target_lost_timeout", timeouts::TARGET_LOST_TIMEOUT);
    this->declare_parameter("arrived_delay", timeouts::ARRIVED_DELAY);
    this->declare_parameter("failed_delay", timeouts::FAILED_DELAY);
    this->declare_parameter("arrival_distance", thresholds::ARRIVAL_DISTANCE);
    // Search timeout - how long to search before giving up (0 = no timeout)
    this->declare_parameter("search_timeout", 300.0);  // 5 minutes default
  }

  // ============================================================
  // SUBSCRIBER CALLBACKS
  // ============================================================
  
  /**
   * Handle wireless controller input
   * Detects L1+R1 trigger to start LISTENING state
   */
  void controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg) {
    uint16_t keys = msg->keys;
    
    // Detect rising edge of trigger (L1 + R1)
    bool trigger_pressed = (keys & TRIGGER_MASK) == TRIGGER_MASK;
    bool prev_trigger = (prev_keys_ & TRIGGER_MASK) == TRIGGER_MASK;
    
    if (trigger_pressed && !prev_trigger) {
      // Rising edge detected
      if (current_state_ == State::IDLE) {
        RCLCPP_INFO(this->get_logger(), "Trigger detected! Transitioning to LISTENING");
        transition_to(State::LISTENING);
      }
    }
    
    prev_keys_ = keys;
  }
  
  /**
   * Handle voice command result
   * Receives target object name from voice_command_node
   */
  void target_object_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (current_state_ == State::LISTENING) {
      if (!msg->data.empty()) {
        target_object_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Target object received: '%s'", target_object_.c_str());
        transition_to(State::SEARCHING);
      }
    }
  }
  
  /**
   * Handle object detection results
   * Checks if target object is in detection list for state transitions
   */
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (current_state_ != State::SEARCHING && current_state_ != State::TRACKING) {
      return;
    }
    
    // Search for target object in detections
    for (const auto& detection : msg->detections) {
      for (const auto& result : detection.results) {
        // Check if this detection matches our target
        // ObjectHypothesisWithPose.id is the class name (not .hypothesis.class_id)
        if (result.id == target_object_) {
          last_detection_time_ = this->now();
          
          if (current_state_ == State::SEARCHING) {
            RCLCPP_INFO(this->get_logger(), "Target '%s' detected! Transitioning to TRACKING",
                        target_object_.c_str());
            transition_to(State::TRACKING);
          }
          return;
        }
      }
    }
  }
  
  /**
   * Handle target pose from perception node
   * Uses actual depth for accurate arrival detection
   */
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (current_state_ != State::TRACKING) {
      return;
    }
    
    // Compute distance from robot to target
    // Target pose is in map frame, robot position from odom
    double target_x = msg->pose.position.x;
    double target_y = msg->pose.position.y;
    double robot_x = current_odom_.pose.pose.position.x;
    double robot_y = current_odom_.pose.pose.position.y;
    
    double dx = target_x - robot_x;
    double dy = target_y - robot_y;
    estimated_target_distance_ = std::sqrt(dx * dx + dy * dy);
    
    RCLCPP_DEBUG(this->get_logger(), "Target distance: %.2f m (target: %.2f,%.2f robot: %.2f,%.2f)", 
                 estimated_target_distance_, target_x, target_y, robot_x, robot_y);
  }
  
  /**
   * Handle odometry updates
   * Used for distance calculations and position tracking
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
  }
  
  /**
   * Handle explore_lite status
   * Detects when exploration is complete (no frontiers)
   * 
   * NOTE: Standard explore_lite does NOT publish this topic.
   * This callback is for compatibility with custom explore_lite forks
   * that implement a /explore/exploring publisher.
   * Without this, the search_timeout parameter handles exhausted exploration.
   */
  void explore_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    exploration_active_ = msg->data;
    
    if (current_state_ == State::SEARCHING && !exploration_active_) {
      RCLCPP_WARN(this->get_logger(), "Exploration complete - no frontiers left!");
      transition_to(State::FAILED);
    }
  }

  // ============================================================
  // STATE MACHINE LOGIC
  // ============================================================
  
  /**
   * Main FSM update loop
   * Called at 10 Hz to check timeouts and state conditions
   */
  void fsm_update() {
    auto now = this->now();
    double time_in_state = (now - state_entry_time_).seconds();
    
    switch (current_state_) {
      case State::IDLE:
        // Nothing to do - waiting for trigger
        break;
        
      case State::LISTENING:
        // Check for timeout
        if (time_in_state > listening_timeout_) {
          RCLCPP_INFO(this->get_logger(), "Listening timeout - returning to IDLE");
          transition_to(State::IDLE);
        }
        break;
        
      case State::SEARCHING:
        // Searching handled by detection callback
        // Check for search timeout (gives up if target not found)
        if (search_timeout_ > 0.0 && time_in_state > search_timeout_) {
          RCLCPP_WARN(this->get_logger(), "Search timeout (%.0f s) - target not found!", search_timeout_);
          transition_to(State::FAILED);
        }
        break;
        
      case State::TRACKING:
        // Check if target lost
        {
          double time_since_detection = (now - last_detection_time_).seconds();
          if (time_since_detection > target_lost_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Target lost! Resuming search");
            transition_to(State::SEARCHING);
          }
          
          // Check if arrived (close enough to target)
          // Note: visual_servo_node handles the actual control
          if (estimated_target_distance_ < arrival_distance_) {
            RCLCPP_INFO(this->get_logger(), "Target reached! Distance: %.2f m",
                        estimated_target_distance_);
            transition_to(State::ARRIVED);
          }
        }
        break;
        
      case State::ARRIVED:
        // Wait before returning to IDLE
        if (time_in_state > arrived_delay_) {
          RCLCPP_INFO(this->get_logger(), "Mission complete - returning to IDLE");
          transition_to(State::IDLE);
        }
        break;
        
      case State::FAILED:
        // Wait before returning to IDLE
        if (time_in_state > failed_delay_) {
          RCLCPP_INFO(this->get_logger(), "Returning to IDLE after failure");
          transition_to(State::IDLE);
        }
        break;
    }
    
    // Publish current state
    publish_state();
  }
  
  /**
   * Transition to a new state
   * Handles exit actions from current state and entry actions for new state
   */
  void transition_to(State new_state) {
    if (new_state == current_state_) {
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                state_to_string(current_state_).c_str(),
                state_to_string(new_state).c_str());
    
    // Exit actions for current state
    exit_state(current_state_);
    
    // Update state
    current_state_ = new_state;
    state_entry_time_ = this->now();
    
    // Entry actions for new state
    enter_state(new_state);
  }
  
  /**
   * Execute entry actions for a state
   */
  void enter_state(State state) {
    switch (state) {
      case State::IDLE:
        // Stand and stop
        publish_posture(postures::STAND);
        disable_exploration();
        target_object_.clear();
        break;
        
      case State::LISTENING:
        // Balance stance for stability
        publish_posture(postures::BALANCE);
        break;
        
      case State::SEARCHING:
        // Enable exploration
        enable_exploration();
        break;
        
      case State::TRACKING:
        // Cancel Nav2 goals and disable exploration
        // visual_servo_node will handle velocity commands
        cancel_nav2_goal();
        disable_exploration();
        break;
        
      case State::ARRIVED:
        // Sit down
        publish_posture(postures::SIT);
        break;
        
      case State::FAILED:
        // Sit down
        publish_posture(postures::SIT);
        break;
    }
  }
  
  /**
   * Execute exit actions for a state
   */
  void exit_state(State state) {
    switch (state) {
      case State::IDLE:
        // Nothing special
        break;
        
      case State::LISTENING:
        // Nothing special
        break;
        
      case State::SEARCHING:
        // Disable exploration
        disable_exploration();
        break;
        
      case State::TRACKING:
        // visual_servo_node stops automatically when state changes
        break;
        
      case State::ARRIVED:
        // Stand up
        publish_posture(postures::STAND);
        break;
        
      case State::FAILED:
        // Stand up
        publish_posture(postures::STAND);
        break;
    }
  }

  // ============================================================
  // HELPER FUNCTIONS
  // ============================================================
  
  /**
   * Publish posture command to robot
   */
  void publish_posture(const char* posture) {
    auto msg = std_msgs::msg::String();
    msg.data = posture;
    posture_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Posture command: %s", posture);
  }
  
  /**
   * Publish current state for debugging
   * This is also used by visual_servo_node to know when to be active
   */
  void publish_state() {
    auto msg = std_msgs::msg::String();
    msg.data = state_to_string(current_state_);
    state_pub_->publish(msg);
  }
  
  /**
   * Enable exploration (resume explore_lite)
   * 
   * NOTE: Standard explore_lite does NOT subscribe to /explore/resume.
   * This is for compatibility with custom explore_lite forks.
   * The main control is via Nav2 goal cancellation and twist_mux priority.
   */
  void enable_exploration() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    explore_enable_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Exploration enabled (via /explore/resume)");
  }
  
  /**
   * Disable exploration (pause explore_lite)
   * 
   * NOTE: Standard explore_lite does NOT subscribe to /explore/resume.
   * Actual control is via cancel_nav2_goal() and twist_mux priority.
   * visual_servo_node at priority 2 overrides Nav2's priority 1.
   */
  void disable_exploration() {
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    explore_enable_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Exploration disabled (via /explore/resume)");
  }
  
  /**
   * Cancel any active Nav2 navigation goal
   */
  void cancel_nav2_goal() {
    if (!nav2_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 action server not available");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Cancelling Nav2 goals");
    nav2_client_->async_cancel_all_goals();
  }

  // ============================================================
  // MEMBER VARIABLES
  // ============================================================
  
  // Current FSM state
  State current_state_;
  rclcpp::Time state_entry_time_;
  
  // Target object from voice command
  std::string target_object_;
  
  // Detection tracking
  rclcpp::Time last_detection_time_;
  double estimated_target_distance_{10.0};  // Updated by target_pose_callback
  
  // Controller state for edge detection
  uint16_t prev_keys_;
  
  // Exploration status
  bool exploration_active_{false};
  
  // Current odometry
  nav_msgs::msg::Odometry current_odom_;
  
  // Parameters
  double listening_timeout_;
  double target_lost_timeout_;
  double arrived_delay_;
  double failed_delay_;
  double arrival_distance_;
  double search_timeout_;
  
  // Subscribers
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr controller_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_object_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_status_sub_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_enable_pub_;
  
  // Action clients
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr fsm_timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


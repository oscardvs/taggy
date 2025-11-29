/**
 * Visual Servo Node - Image-Based Visual Servoing (IBVS) Controller
 * 
 * Implements reactive visual servoing for target tracking with LiDAR safety.
 * Operates directly in 2D image space, minimizing error between detected
 * target position and image center.
 * 
 * Control Laws:
 *   Lateral (Yaw):    ω_z = K_p_yaw * (u_principal - u_target) / image_width
 *   Longitudinal:     v_x = clamp(K_p_dist * (d_current - d_goal), 0, v_max)
 * 
 * Safety:
 *   d_obs = min(scan[front_sector ± cone_angle])
 *   if d_obs < d_safe: v_x = 0  (reflex inhibition)
 * 
 * EDTH Hackathon - Unitree Go2 Visual Servoing
 * 
 * Subscriptions:
 *   /mission/state (std_msgs/String)              - Activate in TRACKING state
 *   /mission/target_object (std_msgs/String)      - Target class name
 *   /detection/bbox (vision_msgs/Detection2DArray) - 2D bounding boxes
 *   /detection/target_pose (geometry_msgs/PoseStamped) - 3D target position
 *   /scan (sensor_msgs/LaserScan)                 - LiDAR for safety
 * 
 * Publications:
 *   /servo_cmd_vel (geometry_msgs/Twist)          - Velocity command
 */

#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

using namespace std::chrono_literals;

class VisualServoNode : public rclcpp::Node {
 public:
  VisualServoNode()
      : Node("visual_servo_node"),
        is_tracking_(false),
        target_object_(""),
        last_detection_time_(this->now()),
        last_pose_time_(this->now()),
        current_distance_(std::numeric_limits<double>::infinity()),
        obstacle_distance_(std::numeric_limits<double>::infinity()) {
    
    // ============================================================
    // PARAMETERS
    // ============================================================
    declare_parameters();
    
    // IBVS gains
    kp_yaw_ = this->get_parameter("servo_kp_yaw").as_double();
    kp_distance_ = this->get_parameter("servo_kp_distance").as_double();
    max_linear_vel_ = this->get_parameter("servo_max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("servo_max_angular_vel").as_double();
    
    // Safety parameters
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    safety_cone_angle_ = this->get_parameter("safety_cone_angle").as_double();
    goal_distance_ = this->get_parameter("goal_distance").as_double();
    
    // Image parameters
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    
    // Detection timeout
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();
    
    // Compute image principal point (center)
    u_principal_ = image_width_ / 2.0;
    v_principal_ = image_height_ / 2.0;
    
    // ============================================================
    // SUBSCRIBERS
    // ============================================================
    
    // Mission state - determines when servo is active
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/mission/state", rclcpp::QoS(10),
        std::bind(&VisualServoNode::state_callback, this, std::placeholders::_1));
    
    // Target object name
    target_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/mission/target_object", rclcpp::QoS(10),
        std::bind(&VisualServoNode::target_callback, this, std::placeholders::_1));
    
    // 2D detections from perception node
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detection/bbox", rclcpp::QoS(10),
        std::bind(&VisualServoNode::detection_callback, this, std::placeholders::_1));
    
    // 3D target pose from perception node (for depth)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/detection/target_pose", rclcpp::QoS(10),
        std::bind(&VisualServoNode::pose_callback, this, std::placeholders::_1));
    
    // LiDAR scan for safety
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10),
        std::bind(&VisualServoNode::scan_callback, this, std::placeholders::_1));
    
    // Robot odometry (for computing robot-to-target distance)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&VisualServoNode::odom_callback, this, std::placeholders::_1));
    
    // ============================================================
    // PUBLISHERS
    // ============================================================
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/servo_cmd_vel", 10);
    
    // ============================================================
    // CONTROL LOOP TIMER
    // ============================================================
    // Run at 20 Hz for smooth control
    control_timer_ = this->create_wall_timer(
        50ms, std::bind(&VisualServoNode::control_loop, this));
    
    // ============================================================
    // LOGGING
    // ============================================================
    RCLCPP_INFO(this->get_logger(), "Visual Servo Node initialized");
    RCLCPP_INFO(this->get_logger(), "  IBVS Gains: K_yaw=%.2f, K_dist=%.2f", 
                kp_yaw_, kp_distance_);
    RCLCPP_INFO(this->get_logger(), "  Velocity limits: v_max=%.2f m/s, ω_max=%.2f rad/s",
                max_linear_vel_, max_angular_vel_);
    RCLCPP_INFO(this->get_logger(), "  Safety: d_safe=%.2f m, cone=±%.1f°",
                safety_distance_, safety_cone_angle_);
    RCLCPP_INFO(this->get_logger(), "  Goal distance: %.2f m", goal_distance_);
  }

 private:
  // ============================================================
  // PARAMETER DECLARATION
  // ============================================================
  void declare_parameters() {
    // IBVS gains
    this->declare_parameter("servo_kp_yaw", 1.0);
    this->declare_parameter("servo_kp_distance", 0.5);
    this->declare_parameter("servo_max_linear_vel", 0.3);
    this->declare_parameter("servo_max_angular_vel", 0.5);
    
    // Safety
    this->declare_parameter("safety_distance", 0.3);
    this->declare_parameter("safety_cone_angle", 30.0);
    this->declare_parameter("goal_distance", 0.5);
    
    // Image
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    
    // Detection timeout
    this->declare_parameter("detection_timeout", 0.5);
  }

  // ============================================================
  // CALLBACKS
  // ============================================================
  
  /**
   * Mission state callback - activate servo only in TRACKING state
   */
  void state_callback(const std_msgs::msg::String::SharedPtr msg) {
    bool was_tracking = is_tracking_;
    is_tracking_ = (msg->data == "TRACKING");
    
    if (is_tracking_ && !was_tracking) {
      RCLCPP_INFO(this->get_logger(), "TRACKING state entered - visual servo ACTIVE");
      // Reset tracking state
      current_distance_ = std::numeric_limits<double>::infinity();
    } else if (!is_tracking_ && was_tracking) {
      RCLCPP_INFO(this->get_logger(), "TRACKING state exited - visual servo INACTIVE");
      // Publish zero velocity
      publish_zero_velocity();
    }
  }
  
  /**
   * Target object callback - store target class name
   */
  void target_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (target_object_ != msg->data) {
      target_object_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Target object set: '%s'", target_object_.c_str());
    }
  }
  
  /**
   * Detection callback - extract target bounding box center
   */
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (!is_tracking_ || target_object_.empty()) {
      return;
    }
    
    // Search for target object in detections
    for (const auto& detection : msg->detections) {
      for (const auto& result : detection.results) {
        // ObjectHypothesisWithPose.id is the class name (not .hypothesis.class_id)
        if (result.id == target_object_) {
          // BoundingBox2D.center is Pose2D with x/y directly (not .position.x)
          target_u_ = detection.bbox.center.x;
          target_v_ = detection.bbox.center.y;
          last_detection_time_ = this->now();
          
          RCLCPP_DEBUG(this->get_logger(), 
              "Target detected at (%.1f, %.1f)", target_u_, target_v_);
          return;
        }
      }
    }
  }
  
  /**
   * Pose callback - extract depth distance to target
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!is_tracking_) {
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
    current_distance_ = std::sqrt(dx * dx + dy * dy);
    last_pose_time_ = this->now();
    
    RCLCPP_DEBUG(this->get_logger(), "Target distance: %.2f m", current_distance_);
  }
  
  /**
   * Odometry callback - store robot position for distance calculation
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
  }
  
  /**
   * LiDAR scan callback - compute minimum distance in frontal cone
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Convert cone angle to radians
    double cone_rad = safety_cone_angle_ * M_PI / 180.0;
    
    // Find indices corresponding to frontal cone
    // LaserScan angle_min is typically at the right (negative y in base_link)
    // angle_max is at the left
    // 0 radians is straight ahead (positive x in base_link)
    
    double min_distance = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < msg->ranges.size(); i++) {
      // Compute angle for this reading
      double angle = msg->angle_min + i * msg->angle_increment;
      
      // Check if within frontal cone
      if (std::abs(angle) <= cone_rad) {
        double range = msg->ranges[i];
        
        // Validate range
        if (range >= msg->range_min && range <= msg->range_max && 
            std::isfinite(range)) {
          min_distance = std::min(min_distance, static_cast<double>(range));
        }
      }
    }
    
    obstacle_distance_ = min_distance;
    
    // Log if obstacle is close
    if (obstacle_distance_ < safety_distance_ * 1.5) {
      RCLCPP_DEBUG(this->get_logger(), 
          "Obstacle distance: %.2f m (safety: %.2f m)", 
          obstacle_distance_, safety_distance_);
    }
  }

  // ============================================================
  // CONTROL LOOP
  // ============================================================
  
  /**
   * Main control loop - compute and publish IBVS velocity command
   */
  void control_loop() {
    // Only active in TRACKING state
    if (!is_tracking_) {
      return;
    }
    
    auto now = this->now();
    
    // Check if detection is recent
    double detection_age = (now - last_detection_time_).seconds();
    if (detection_age > detection_timeout_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Detection stale (%.2f s old) - stopping", detection_age);
      publish_zero_velocity();
      return;
    }
    
    // ========================================
    // COMPUTE IBVS CONTROL
    // ========================================
    
    geometry_msgs::msg::Twist cmd;
    
    // --- Lateral Control (Yaw) ---
    // Error: positive when target is to the left of image center
    double e_u = u_principal_ - target_u_;
    
    // Normalized proportional control
    // Positive error -> positive ω_z (turn left) to center the target
    double omega_z = kp_yaw_ * (e_u / u_principal_);
    
    // Clamp angular velocity
    omega_z = std::clamp(omega_z, -max_angular_vel_, max_angular_vel_);
    cmd.angular.z = omega_z;
    
    // --- Longitudinal Control (Forward) ---
    double v_x = 0.0;
    
    // Use depth-based distance if available, otherwise stay stopped
    if (std::isfinite(current_distance_)) {
      // Error: positive when robot is farther than goal
      double e_d = current_distance_ - goal_distance_;
      
      // Only move forward if we're farther than goal
      if (e_d > 0.0) {
        v_x = kp_distance_ * e_d;
        // Clamp to maximum velocity
        v_x = std::min(v_x, max_linear_vel_);
      }
    }
    
    // ========================================
    // SAFETY WRAPPER (LiDAR Reflex)
    // ========================================
    
    bool safety_override = false;
    if (obstacle_distance_ < safety_distance_) {
      // Override forward velocity to zero
      v_x = 0.0;
      safety_override = true;
      
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "SAFETY: Obstacle at %.2f m - forward velocity inhibited!",
          obstacle_distance_);
    }
    
    cmd.linear.x = v_x;
    
    // ========================================
    // PUBLISH COMMAND
    // ========================================
    
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Servo cmd: v_x=%.2f, ω_z=%.2f | e_u=%.1f, d=%.2f%s",
        cmd.linear.x, cmd.angular.z, e_u, current_distance_,
        safety_override ? " [SAFETY]" : "");
  }
  
  /**
   * Publish zero velocity command
   */
  void publish_zero_velocity() {
    geometry_msgs::msg::Twist cmd;
    // All fields default to 0.0
    cmd_vel_pub_->publish(cmd);
  }

  // ============================================================
  // MEMBER VARIABLES
  // ============================================================
  
  // State
  bool is_tracking_;
  std::string target_object_;
  
  // Detection data
  double target_u_{0.0};          // Target u coordinate (horizontal)
  double target_v_{0.0};          // Target v coordinate (vertical)
  rclcpp::Time last_detection_time_;
  
  // Depth data
  rclcpp::Time last_pose_time_;
  double current_distance_;
  
  // Safety data
  double obstacle_distance_;
  
  // Image parameters
  int image_width_;
  int image_height_;
  double u_principal_;            // Image center u
  double v_principal_;            // Image center v
  
  // IBVS gains
  double kp_yaw_;
  double kp_distance_;
  double max_linear_vel_;
  double max_angular_vel_;
  
  // Safety parameters
  double safety_distance_;
  double safety_cone_angle_;
  double goal_distance_;
  double detection_timeout_;
  
  // Robot odometry
  nav_msgs::msg::Odometry current_odom_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


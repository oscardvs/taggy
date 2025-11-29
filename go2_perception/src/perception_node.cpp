/**
 * Perception Node - YOLOv8 Object Detection with Depth Fusion
 * 
 * Performs real-time object detection using YOLOv8 optimized with TensorRT.
 * Fuses RGB detections with aligned depth to compute 3D object positions,
 * then transforms to the map frame for navigation.
 * 
 * EDTH Hackathon - Unitree Go2 Perception
 * 
 * Subscriptions:
 *   /camera/color/image_raw (sensor_msgs/Image)              - RGB camera
 *   /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image) - Aligned depth
 *   /camera/color/camera_info (sensor_msgs/CameraInfo)       - Camera intrinsics
 *   /mission/target_object (std_msgs/String)                 - Target to track
 * 
 * Publications:
 *   /detection/bbox (vision_msgs/Detection2DArray)           - 2D detections
 *   /detection/target_pose (geometry_msgs/PoseStamped)       - 3D pose in map frame
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>

#include "go2_perception/tensorrt_yolo.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PerceptionNode : public rclcpp::Node {
 public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  PerceptionNode()
      : Node("perception_node"),
        camera_info_received_(false) {
    
    // ============================================================
    // PARAMETERS
    // ============================================================
    declare_parameters();
    
    // Get parameters
    onnx_path_ = this->get_parameter("model_path").as_string();
    engine_path_ = this->get_parameter("engine_path").as_string();
    conf_threshold_ = this->get_parameter("confidence_threshold").as_double();
    nms_threshold_ = this->get_parameter("nms_threshold").as_double();
    input_width_ = this->get_parameter("input_width").as_int();
    input_height_ = this->get_parameter("input_height").as_int();
    use_fp16_ = this->get_parameter("use_fp16").as_bool();
    depth_median_kernel_ = this->get_parameter("depth_median_kernel").as_int();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    
    // Ensure kernel size is odd
    if (depth_median_kernel_ % 2 == 0) {
      depth_median_kernel_++;
    }
    
    // ============================================================
    // TENSORRT INITIALIZATION
    // ============================================================
    yolo_ = std::make_unique<go2_perception::TensorRTYolo>(
      onnx_path_,
      engine_path_,
      input_width_,
      input_height_,
      static_cast<float>(conf_threshold_),
      static_cast<float>(nms_threshold_),
      use_fp16_
    );
    
    if (!yolo_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize TensorRT engine!");
      RCLCPP_ERROR(this->get_logger(), "  ONNX path: %s", onnx_path_.c_str());
      RCLCPP_ERROR(this->get_logger(), "  Engine path: %s", engine_path_.c_str());
      // Node will continue but detections will be empty
    }
    
    // ============================================================
    // TF2 BUFFER
    // ============================================================
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // ============================================================
    // SUBSCRIBERS
    // ============================================================
    
    // Target object from mission executive (via voice command)
    target_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/mission/target_object", 10,
      std::bind(&PerceptionNode::target_callback, this, _1));
    
    // Camera intrinsics (only need once)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      std::bind(&PerceptionNode::camera_info_callback, this, _1));
    
    // Synchronized RGB + Depth using message_filters
    rgb_sub_.subscribe(this, "/camera/color/image_raw");
    depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&PerceptionNode::image_callback, this, _1, _2));
    
    // ============================================================
    // PUBLISHERS
    // ============================================================
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/detection/bbox", 10);
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detection/target_pose", 10);
    
    // ============================================================
    // LOGGING
    // ============================================================
    RCLCPP_INFO(this->get_logger(), "Perception Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Model: %s", onnx_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Engine: %s", engine_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Input size: %dx%d", input_width_, input_height_);
    RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", conf_threshold_);
    RCLCPP_INFO(this->get_logger(), "  NMS threshold: %.2f", nms_threshold_);
    RCLCPP_INFO(this->get_logger(), "  FP16: %s", use_fp16_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
  }

 private:
  // ============================================================
  // PARAMETER DECLARATION
  // ============================================================
  void declare_parameters() {
    this->declare_parameter("model_path", "/opt/yolo/yolov8n.onnx");
    this->declare_parameter("engine_path", "/opt/yolo/yolov8n.engine");
    this->declare_parameter("confidence_threshold", 0.6);
    this->declare_parameter("nms_threshold", 0.45);
    this->declare_parameter("input_width", 640);
    this->declare_parameter("input_height", 640);
    this->declare_parameter("use_fp16", true);
    this->declare_parameter("depth_median_kernel", 5);
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    this->declare_parameter("target_frame", "map");
  }

  // ============================================================
  // CALLBACKS
  // ============================================================
  
  /**
   * Store target object name from voice command
   */
  void target_callback(const std_msgs::msg::String::SharedPtr msg) {
    target_object_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Target object set: '%s'", target_object_.c_str());
  }

  /**
   * Store camera intrinsics (only processed once)
   */
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
      fx_ = msg->k[0];  // K[0,0]
      fy_ = msg->k[4];  // K[1,1]
      cx_ = msg->k[2];  // K[0,2]
      cy_ = msg->k[5];  // K[1,2]
      
      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera intrinsics received:");
      RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                  fx_, fy_, cx_, cy_);
    }
  }

  /**
   * Main perception pipeline - synchronized RGB + Depth callback
   */
  void image_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    
    if (!yolo_->isInitialized()) {
      return;
    }

    // Convert ROS images to OpenCV
    cv_bridge::CvImageConstPtr rgb_cv;
    cv_bridge::CvImageConstPtr depth_cv;
    
    try {
      rgb_cv = cv_bridge::toCvShare(rgb_msg, "bgr8");
      depth_cv = cv_bridge::toCvShare(depth_msg);  // Keep original encoding (16UC1)
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat& rgb_image = rgb_cv->image;
    const cv::Mat& depth_image = depth_cv->image;

    // Run YOLOv8 inference
    auto detections = yolo_->detect(rgb_image);

    // Build Detection2DArray message
    vision_msgs::msg::Detection2DArray det_array_msg;
    det_array_msg.header = rgb_msg->header;

    // Track best matching detection for target pose
    bool found_target = false;
    go2_perception::Detection best_target_det;
    float best_target_confidence = 0.0f;

    for (const auto& det : detections) {
      // Create Detection2D message
      vision_msgs::msg::Detection2D det_msg;
      det_msg.header = rgb_msg->header;
      
      // Bounding box
      // BoundingBox2D.center is Pose2D with x, y, theta (not position.x)
      det_msg.bbox.center.x = det.cx;
      det_msg.bbox.center.y = det.cy;
      det_msg.bbox.size_x = det.bbox.width;
      det_msg.bbox.size_y = det.bbox.height;
      
      // ObjectHypothesisWithPose has direct id/score fields (not hypothesis.*)
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.id = det.class_name;
      hyp.score = det.confidence;
      det_msg.results.push_back(hyp);
      
      det_array_msg.detections.push_back(det_msg);

      // Check if this matches our target object
      if (!target_object_.empty() && 
          det.class_name == target_object_ &&
          det.confidence > best_target_confidence) {
        found_target = true;
        best_target_det = det;
        best_target_confidence = det.confidence;
      }
    }

    // Publish all detections
    detection_pub_->publish(det_array_msg);

    // If we found the target, compute and publish 3D pose
    if (found_target && camera_info_received_) {
      publish_target_pose(best_target_det, depth_image, rgb_msg->header);
    }
  }

  /**
   * Compute 3D position from detection and depth, transform to map frame
   */
  void publish_target_pose(
      const go2_perception::Detection& det,
      const cv::Mat& depth_image,
      const std_msgs::msg::Header& header) {
    
    // Get depth at detection centroid with median filtering
    int u = static_cast<int>(det.cx);
    int v = static_cast<int>(det.cy);
    
    float depth_m = get_median_depth(depth_image, u, v);
    
    if (depth_m <= 0.0f || depth_m > 10.0f) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Invalid depth at target centroid: %.2f m", depth_m);
      return;
    }

    // Deproject 2D pixel to 3D point in camera frame
    // X = (u - cx) * Z / fx
    // Y = (v - cy) * Z / fy  
    // Z = depth
    double x_cam = (u - cx_) * depth_m / fx_;
    double y_cam = (v - cy_) * depth_m / fy_;
    double z_cam = depth_m;

    // Create point in camera frame
    geometry_msgs::msg::PoseStamped pose_cam;
    pose_cam.header = header;
    pose_cam.header.frame_id = camera_frame_;
    pose_cam.pose.position.x = z_cam;   // In optical frame, Z is forward
    pose_cam.pose.position.y = -x_cam;  // X is right, we want left
    pose_cam.pose.position.z = -y_cam;  // Y is down, we want up
    pose_cam.pose.orientation.w = 1.0;

    // Transform to map frame
    geometry_msgs::msg::PoseStamped pose_map;
    try {
      pose_map = tf_buffer_->transform(pose_cam, target_frame_, 100ms);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "TF transform failed: %s", e.what());
      return;
    }

    // Publish pose
    pose_pub_->publish(pose_map);

    RCLCPP_DEBUG(this->get_logger(), 
      "Target '%s' at (%.2f, %.2f, %.2f) in %s frame",
      det.class_name.c_str(),
      pose_map.pose.position.x,
      pose_map.pose.position.y,
      pose_map.pose.position.z,
      target_frame_.c_str());
  }

  /**
   * Get median depth value in a kernel around the pixel
   * More robust than single pixel sampling
   */
  float get_median_depth(const cv::Mat& depth_image, int u, int v) {
    int half_k = depth_median_kernel_ / 2;
    
    // Clamp to image bounds
    int u_min = std::max(0, u - half_k);
    int u_max = std::min(depth_image.cols - 1, u + half_k);
    int v_min = std::max(0, v - half_k);
    int v_max = std::min(depth_image.rows - 1, v + half_k);

    std::vector<float> depths;
    depths.reserve((u_max - u_min + 1) * (v_max - v_min + 1));

    for (int vv = v_min; vv <= v_max; vv++) {
      for (int uu = u_min; uu <= u_max; uu++) {
        uint16_t depth_mm = depth_image.at<uint16_t>(vv, uu);
        if (depth_mm > 0) {
          depths.push_back(depth_mm / 1000.0f);  // Convert mm to meters
        }
      }
    }

    if (depths.empty()) {
      return -1.0f;
    }

    // Get median
    size_t mid = depths.size() / 2;
    std::nth_element(depths.begin(), depths.begin() + mid, depths.end());
    return depths[mid];
  }

  // ============================================================
  // MEMBER VARIABLES
  // ============================================================
  
  // Parameters
  std::string onnx_path_;
  std::string engine_path_;
  double conf_threshold_;
  double nms_threshold_;
  int input_width_;
  int input_height_;
  bool use_fp16_;
  int depth_median_kernel_;
  std::string camera_frame_;
  std::string target_frame_;

  // Camera intrinsics
  bool camera_info_received_;
  double fx_, fy_, cx_, cy_;

  // Target object name
  std::string target_object_;

  // TensorRT engine
  std::unique_ptr<go2_perception::TensorRTYolo> yolo_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publishers
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


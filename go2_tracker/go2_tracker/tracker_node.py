#!/usr/bin/env python3
"""
Threat Tracker ROS2 Node

Main tracker node that automatically selects the appropriate tracker based on config:
- Standard YOLO11 tracker for single-threat environments
- Re-ID tracker for multi-threat environments (with reference image)

Configuration is loaded from tracker_config.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header

from cv_bridge import CvBridge
import numpy as np
from typing import Optional
from pathlib import Path

from go2_tracker.base_tracker import CameraIntrinsics
from go2_tracker.standard_tracker import StandardTracker
from go2_tracker.reid_tracker import ReIDTracker
from go2_tracker.coco_classes import get_class_id, COCO_CLASSES


class ThreatTrackerNode(Node):
    """
    Main threat tracker ROS2 node.
    
    Automatically selects tracker based on configuration:
    - env_with_multiple_threats=false -> StandardTracker
    - env_with_multiple_threats=true  -> ReIDTracker (requires reference image)
    """
    
    def __init__(self):
        super().__init__('tracker')
        
        # Declare parameters
        self.declare_parameter('threat_id', 'person')
        self.declare_parameter('env_with_multiple_threats', False)
        self.declare_parameter('reference_image_path', '')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('reid_method', 'combined')
        self.declare_parameter('match_threshold', 0.5)
        self.declare_parameter('tracker_type', 'bytetrack.yaml')
        self.declare_parameter('device', '')
        self.declare_parameter('depth_filter_size', 5)
        self.declare_parameter('publish_debug_image', True)

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        self.declare_parameter('target_object_topic', '/mission/target_object')
        
        # Get parameters
        self.threat_id = self.get_parameter('threat_id').value
        self.multi_threat_env = self.get_parameter('env_with_multiple_threats').value
        self.reference_path = self.get_parameter('reference_image_path').value
        self.model = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.reid_method = self.get_parameter('reid_method').value
        self.match_threshold = self.get_parameter('match_threshold').value
        self.tracker_type = self.get_parameter('tracker_type').value
        self.device = self.get_parameter('device').value
        self.depth_filter_size = self.get_parameter('depth_filter_size').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        target_object_topic = self.get_parameter('target_object_topic').value
        
        # Validate threat_id
        class_id = get_class_id(self.threat_id)
        if class_id < 0:
            self.get_logger().error(f"Unknown threat_id: '{self.threat_id}'")
            self.get_logger().error(f"Available classes: {list(COCO_CLASSES.keys())}")
            raise ValueError(f"Unknown threat_id: {self.threat_id}")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera state
        self.camera_intrinsics: Optional[CameraIntrinsics] = None
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None

        # Tracker state
        self.tracker = None
        self.tracker_initialized = False
        
        # Wait for target from topic or use config
        self.target_from_topic = False
        self.waiting_for_target = True
        
        # Initialize appropriate tracker
        self._init_tracker()

        # Subscribe to target object topic first
        self.target_sub = self.create_subscription(
            String, target_object_topic, self._target_callback, 10)
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers        
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self._rgb_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, sensor_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, sensor_qos)
        
        # Publishers
        #self.pose_pub = self.create_publisher(PoseStamped, '/tracked_object/pose', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/tracked_object/status', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/tracked_object/debug_image', 10)

        # Timer to wait for target topic, then fallback to config
        self.startup_timer = self.create_timer(2.0, self._startup_check)
        
        # Processing timer (30 Hz) - starts after tracker is initialized
        self.process_timer = None
        
        # # Processing timer (30 Hz)
        # self.timer = self.create_timer(1.0 / 30.0, self._process_frame)
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("THREAT TRACKER INITIALIZED")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  threat_id: {self.threat_id} (class {class_id})")
        self.get_logger().info(f"  env_with_multiple_threats: {self.multi_threat_env}")
        self.get_logger().info(f"  tracker_type: {'Re-ID' if self.multi_threat_env else 'Standard YOLO11'}")
        self.get_logger().info(f"  model: {self.model}")
        self.get_logger().info(f"  rgb_topic: {rgb_topic}")
        self.get_logger().info(f"  depth_topic: {depth_topic}")
        self.get_logger().info(f"  camera_info_topic: {camera_info_topic}")
        if self.multi_threat_env:
            self.get_logger().info(f"  reference_image: {self.reference_path or 'NOT SET'}")
            self.get_logger().info(f"  match_threshold: {self.match_threshold}")
        self.get_logger().info("=" * 60)
    
    def _target_callback(self, msg: String):
        """Handle dynamic target updates from /mission/target_object."""
        new_target = msg.data.strip().lower()
        
        if not new_target:
            return
        
        # Validate the target class
        class_id = get_class_id(new_target)
        if class_id < 0:
            self.get_logger().warn(f"Unknown target class: '{new_target}'. Available: {list(COCO_CLASSES.keys())}")
            return
        
        # Check if target changed
        if new_target != self.threat_id:
            self.get_logger().info(f"Target changed: {self.threat_id} -> {new_target}")
            self.threat_id = new_target
            self.target_from_topic = True
            self._init_tracker()
        
        # If waiting for first target
        if self.waiting_for_target:
            self.waiting_for_target = False
            self.target_from_topic = True
            self._init_tracker()
            self._start_processing()
    
    def _startup_check(self):
        """Check if we received target from topic, otherwise use config."""
        # Cancel this timer
        self.startup_timer.cancel()
        
        if self.waiting_for_target:
            self.get_logger().info(f"No target received from topic, using config: {self.threat_id}")
            self.waiting_for_target = False
            self._init_tracker()
            self._start_processing()
    
    def _start_processing(self):
        """Start the main processing timer."""
        if self.process_timer is None:
            self.process_timer = self.create_timer(1.0 / 30.0, self._process_frame)
    
    def _init_tracker(self):
        """Initialize the appropriate tracker based on config."""
        if self.multi_threat_env:
            # Multi-threat environment: use Re-ID tracker
            self.get_logger().info("Initializing Re-ID tracker for multi-threat environment")
            
            if not self.reference_path:
                self.get_logger().warn(
                    "No reference_image_path set! Run 'ros2 run go2_tracker capture_reference' first."
                )
            
            self.tracker = ReIDTracker(
                threat_id=self.threat_id,
                reference_image_path=self.reference_path,
                model=self.model,
                confidence_threshold=self.conf_threshold,
                reid_method=self.reid_method,
                match_threshold=self.match_threshold,
                device=self.device,
                depth_filter_size=self.depth_filter_size,
            )
        else:
            # Single-threat environment: use standard tracker
            self.get_logger().info("Initializing standard YOLO11 tracker")
            
            self.tracker = StandardTracker(
                threat_id=self.threat_id,
                model=self.model,
                confidence_threshold=self.conf_threshold,
                tracker_type=self.tracker_type,
                device=self.device,
                depth_filter_size=self.depth_filter_size,
            )
    
    def _rgb_callback(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
    
    def _depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
    
    def _camera_info_callback(self, msg: CameraInfo):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = CameraIntrinsics(
                fx=msg.k[0],
                fy=msg.k[4],
                cx=msg.k[2],
                cy=msg.k[5],
                width=msg.width,
                height=msg.height
            )
            self.get_logger().info(f"Camera intrinsics received: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}")
    
    def _process_frame(self):
        """Main processing loop."""

        if self.latest_rgb is None:
            return
        
        # Process frame with tracker
        target = self.tracker.process_frame(
            self.latest_rgb,
            self.latest_depth,
            self.camera_intrinsics
        )

        # Publish pose if we have a target
        if target and target.position_3d is not None:
            self._publish_pose(target.position_3d)
        
        # Publish status
        status = self.tracker.get_status()
        self.status_pub.publish(String(data=status))
        
        # Publish debug image
        if self.publish_debug:
            debug_img = self.tracker.get_debug_image(self.latest_rgb)
            try:
                msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                self.debug_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Debug image error: {e}")
    
    def _publish_pose(self, position: np.ndarray):
        """Publish pose message."""
        msg = PoseStamped()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="camera_color_optical_frame"
        )
        msg.pose.position = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2])
        )
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ThreatTrackerNode()
        rclpy.spin(node)
    except ValueError as e:
        print(f"Configuration error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
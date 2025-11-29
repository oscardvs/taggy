#!/usr/bin/env python3
"""
Threat Tracker ROS2 Node

Main tracker node that automatically selects the appropriate tracker based on config:
- Standard YOLO11 tracker for single-threat environments
- Classical Re-ID tracker (SIFT + histogram) for multi-threat environments
- Deep Re-ID tracker (neural network) for robust multi-threat tracking

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

try:
    from go2_tracker.deep_reid_tracker import DeepReIDTracker
    DEEP_REID_AVAILABLE = True
except ImportError as e:
    DEEP_REID_AVAILABLE = False
    print(f"Deep Re-ID not available: {e}")


class ThreatTrackerNode(Node):
    """
    Main threat tracker ROS2 node.
    
    Automatically selects tracker based on configuration:
    - env_with_multiple_threats=false -> StandardTracker
    - env_with_multiple_threats=true  -> ReIDTracker or DeepReIDTracker (requires reference image)
      - reid_backend='classical' -> ReIDTracker (SIFT + histogram)
      - reid_backend='deep'      -> DeepReIDTracker (neural network)
    """
    
    def __init__(self):
        super().__init__('tracker')
        
        # Declare parameters
        self.declare_parameter('threat_id', 'person')
        self.declare_parameter('env_with_multiple_threats', False)
        self.declare_parameter('reference_image_path', '')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Re-ID backend selection
        self.declare_parameter('reid_backend', 'deep')  # 'classical' or 'deep'
        
        # Classical Re-ID parameters (SIFT + histogram)
        self.declare_parameter('reid_method', 'combined')
        
        # Deep Re-ID parameters
        self.declare_parameter('reid_model', 'osnet_x1_0')
        self.declare_parameter('reid_weights', 'osnet_x1_0_imagenet')
        self.declare_parameter('track_memory_frames', 30)
        self.declare_parameter('reid_verification_interval', 10)
        
        # Common parameters
        self.declare_parameter('match_threshold', 0.6)
        self.declare_parameter('tracker_type', 'bytetrack.yaml')
        self.declare_parameter('device', '')
        self.declare_parameter('depth_filter_size', 5)
        self.declare_parameter('publish_debug_image', True)

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        
        # Get parameters
        self.threat_id = self.get_parameter('threat_id').value
        self.multi_threat_env = self.get_parameter('env_with_multiple_threats').value
        self.reference_path = self.get_parameter('reference_image_path').value
        self.model = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        
        # Re-ID configuration
        self.reid_backend = self.get_parameter('reid_backend').value
        self.reid_method = self.get_parameter('reid_method').value
        self.reid_model = self.get_parameter('reid_model').value
        self.reid_weights = self.get_parameter('reid_weights').value
        self.track_memory_frames = self.get_parameter('track_memory_frames').value
        self.reid_verify_interval = self.get_parameter('reid_verification_interval').value
        
        self.match_threshold = self.get_parameter('match_threshold').value
        self.tracker_type = self.get_parameter('tracker_type').value
        self.device = self.get_parameter('device').value
        self.depth_filter_size = self.get_parameter('depth_filter_size').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
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
        
        # Initialize appropriate tracker
        self._init_tracker()
        
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
        self.pose_pub = self.create_publisher(PoseStamped, '/tracked_object/pose', 10)
        self.status_pub = self.create_publisher(String, '/tracked_object/status', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/tracked_object/debug_image', 10)
        
        # Processing timer (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self._process_frame)
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("THREAT TRACKER INITIALIZED")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  threat_id: {self.threat_id} (class {class_id})")
        self.get_logger().info(f"  env_with_multiple_threats: {self.multi_threat_env}")
        
        if self.multi_threat_env:
            tracker_name = f"Re-ID ({self.reid_backend})"
        else:
            tracker_name = "Standard YOLO11"
        self.get_logger().info(f"  tracker_type: {tracker_name}")
        
        self.get_logger().info(f"  model: {self.model}")
        self.get_logger().info(f"  rgb_topic: {rgb_topic}")
        self.get_logger().info(f"  depth_topic: {depth_topic}")
        self.get_logger().info(f"  camera_info_topic: {camera_info_topic}")
        
        if self.multi_threat_env:
            self.get_logger().info(f"  reference_image: {self.reference_path or 'NOT SET'}")
            self.get_logger().info(f"  match_threshold: {self.match_threshold}")
            if self.reid_backend == 'deep':
                self.get_logger().info(f"  reid_model: {self.reid_model}")
                self.get_logger().info(f"  track_memory: {self.track_memory_frames} frames")
        
        self.get_logger().info("=" * 60)
    
    def _init_tracker(self):
        """Initialize the appropriate tracker based on config."""
        if self.multi_threat_env:
            # Multi-threat environment: use Re-ID tracker
            if not self.reference_path:
                self.get_logger().warn(
                    "No reference_image_path set! Run 'ros2 run go2_tracker capture_reference' first."
                )
            
            # Select Re-ID backend
            if self.reid_backend == 'deep':
                # Deep learning Re-ID
                if not DEEP_REID_AVAILABLE:
                    self.get_logger().error(
                        "Deep Re-ID requested but not available! "
                        "Install with: pip install -r requirements.txt"
                    )
                    self.get_logger().warn("Falling back to classical Re-ID tracker")
                    self.reid_backend = 'classical'
                else:
                    self.get_logger().info(
                        f"Initializing Deep Re-ID tracker with {self.reid_model}"
                    )
                    self.tracker = DeepReIDTracker(
                        threat_id=self.threat_id,
                        reference_image_path=self.reference_path,
                        model=self.model,
                        confidence_threshold=self.conf_threshold,
                        reid_model=self.reid_model,
                        reid_weights=self.reid_weights,
                        match_threshold=self.match_threshold,
                        device=self.device,
                        depth_filter_size=self.depth_filter_size,
                        track_memory_frames=self.track_memory_frames,
                        reid_verification_interval=self.reid_verify_interval,
                    )
            
            if self.reid_backend == 'classical':
                # Classical Re-ID (SIFT + histogram)
                self.get_logger().info(
                    "Initializing classical Re-ID tracker (SIFT + histogram)"
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
#!/usr/bin/env python3
"""
Reference Image Capture Tool

Interactive tool to capture a reference image of the target object.
Shows the user what the camera sees and lets them capture when ready.

Usage:
    ros2 run go2_tracker capture_reference

The script will:
1. Show live camera feed
2. Wait for user to position the target object
3. Press SPACE or ENTER to capture
4. Save the reference image for the Re-ID tracker
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
import os


class ReferenceCaptureNode(Node):
    """
    Interactive reference image capture node.
    """
    
    def __init__(self):
        super().__init__('reference_capture')
        
        # Parameters
        self.declare_parameter('output_dir', str(Path.home() / 'tracker_references'))
        self.declare_parameter('output_name', '')
        
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.output_name = self.get_parameter('output_name').value
        
        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # CV bridge
        self.bridge = CvBridge()
        
        # State
        self.latest_image = None
        self.captured = False
        self.capture_path = None
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self._image_callback, sensor_qos)
        
        # Display timer
        self.timer = self.create_timer(1.0 / 30.0, self._display_loop)
        
        # Print instructions
        self._print_instructions()
    
    def _print_instructions(self):
        """Print usage instructions."""
        print("\n" + "=" * 60)
        print("REFERENCE IMAGE CAPTURE")
        print("=" * 60)
        print("\nInstructions:")
        print("  1. Hold the target object in front of the camera")
        print("  2. Make sure the object fills most of the frame")
        print("  3. Ensure good lighting and clear visibility")
        print("  4. Press SPACE or ENTER to capture")
        print("  5. Press 'q' or ESC to quit without saving")
        print("\nTips for best results:")
        print("  - Capture at similar distance to tracking scenario")
        print("  - Include distinctive features/colors")
        print("  - Avoid motion blur")
        print("=" * 60 + "\n")
    
    def _image_callback(self, msg: Image):
        """Store latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
    
    def _display_loop(self):
        """Display loop with capture logic."""
        if self.latest_image is None:
            return
        
        # Create display image
        display = self.latest_image.copy()
        h, w = display.shape[:2]
        
        # Draw center guide
        cx, cy = w // 2, h // 2
        guide_size = min(w, h) // 3
        
        # Draw target rectangle
        cv2.rectangle(
            display,
            (cx - guide_size, cy - guide_size),
            (cx + guide_size, cy + guide_size),
            (0, 255, 0), 2
        )
        
        # Draw crosshair
        cv2.line(display, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(display, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
        
        # Add instructions
        cv2.putText(display, "Position target object in frame", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display, "Press SPACE/ENTER to capture, 'q'/ESC to quit", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show image
        cv2.imshow("Reference Capture", display)
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        
        if key in [ord(' '), 13]:  # SPACE or ENTER
            self._capture_image()
        elif key in [ord('q'), 27]:  # 'q' or ESC
            self._quit()
    
    def _capture_image(self):
        """Capture and save reference image."""
        if self.latest_image is None:
            return
        
        # Generate filename
        if self.output_name:
            filename = f"{self.output_name}.jpg"
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"reference_{timestamp}.jpg"
        
        self.capture_path = self.output_dir / filename
        
        # Save image
        cv2.imwrite(str(self.capture_path), self.latest_image)
        
        self.captured = True
        
        print("\n" + "=" * 60)
        print("✓ REFERENCE IMAGE CAPTURED!")
        print("=" * 60)
        print(f"\nSaved to: {self.capture_path}")
        print("\nTo use this reference, update your tracker_config.yaml:")
        print(f'  reference_image_path: "{self.capture_path}"')
        print("\nOr launch with:")
        print(f"  ros2 launch go2_tracker tracker.launch.py reference_image:={self.capture_path}")
        print("=" * 60 + "\n")
        
        # Show captured image briefly
        cv2.putText(self.latest_image, "CAPTURED!", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)
        cv2.imshow("Reference Capture", self.latest_image)
        cv2.waitKey(1500)
        
        self._quit()
    
    def _quit(self):
        """Clean up and quit."""
        cv2.destroyAllWindows()
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    
    print("\nWaiting for camera feed...")
    print("Make sure RealSense is running:")
    print("  ros2 launch realsense2_camera rs_launch.py\n")
    
    node = ReferenceCaptureNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
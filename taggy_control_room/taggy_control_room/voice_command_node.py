#!/usr/bin/env python3
"""
Voice Command ROS2 Node for TAGGY Control Room

Subscribes to /voice/transcript and publishes target class to /mission/target_object
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from typing import Optional, Tuple


THREAT_MAPPING = {
    'soldier': 'person', 'enemy': 'person', 'person': 'person', 'intruder': 'person',
    'human': 'person', 'man': 'person', 'woman': 'person', 'hostile': 'person',
    'dog': 'dog', 'canine': 'dog', 'k9': 'dog', 'hound': 'dog', 'cat': 'cat',
    'vehicle': 'car', 'car': 'car', 'truck': 'truck', 'jeep': 'car',
    'drone': 'airplane', 'uav': 'airplane', 'aircraft': 'airplane', 'helicopter': 'airplane',
    'motorcycle': 'motorcycle', 'bike': 'bicycle', 'boat': 'boat',
}

ACTION_KEYWORDS = ['intercept', 'engage', 'track', 'follow', 'pursue', 'stop', 'target', 'locate', 'find']


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command')
        
        self.declare_parameter('transcript_topic', '/voice/transcript')
        transcript_topic = self.get_parameter('transcript_topic').value
        
        # Publishers
        self.target_pub = self.create_publisher(String, '/mission/target_object', 10)
        self.status_pub = self.create_publisher(String, '/voice/status', 10)
        self.command_pub = self.create_publisher(String, '/mission/command', 10)
        
        # Subscriber
        self.transcript_sub = self.create_subscription(
            String, transcript_topic, self._transcript_callback, 10
        )
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("VOICE COMMAND NODE READY")
        self.get_logger().info(f"Listening on: {transcript_topic}")
        self.get_logger().info(f"Publishing to: /mission/target_object")
        self.get_logger().info("=" * 50)
    
    def _transcript_callback(self, msg: String):
        transcript = msg.data.strip()
        if transcript:
            self.get_logger().info(f"Received: {transcript}")
            self._process_command(transcript)
    
    def _process_command(self, command: str):
        command_lower = command.lower()
        threat_class, keyword = self._extract_threat(command_lower)
        
        if threat_class:
            self.get_logger().info(f"Threat detected: {keyword} -> {threat_class}")
            has_action = any(action in command_lower for action in ACTION_KEYWORDS)
            
            if has_action:
                self._publish_target(threat_class)
                self._publish_command(f"INTERCEPT:{threat_class}")
            
            self._publish_status(f"Target: {threat_class}")
        else:
            self.get_logger().warn(f"No threat found in: {command}")
            self._publish_status("Could not identify threat")
    
    def _extract_threat(self, command: str) -> Tuple[Optional[str], Optional[str]]:
        for keyword, coco_class in THREAT_MAPPING.items():
            if re.search(r'\b' + re.escape(keyword) + r'\b', command):
                return coco_class, keyword
        return None, None
    
    def _publish_target(self, target_class: str):
        msg = String()
        msg.data = target_class
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published target: {target_class}")
    
    def _publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
    
    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
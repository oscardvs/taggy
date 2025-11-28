#!/usr/bin/env python3
"""
Unitree Go2 State Publisher Node
Publishes robot state information including battery, mode, etc.

EDTH Hackathon Starter Pack by Laelaps AI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import sys
import os

# Add parent directory to path for local imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from go2_interfaces.go2_sdk import Go2HighLevelInterface


class Go2StatePublisher(Node):
    """
    ROS2 node that publishes Go2 state information
    
    Publications:
        /go2/battery (std_msgs/Float32): Battery level (0-100%)
        /go2/mode (std_msgs/Int32): Robot mode (0=idle, 1=standing, 2=walking)
        /go2/mode_str (std_msgs/String): Human-readable mode string
    
    Parameters:
        robot_ip: IP address of Go2 (default: 192.168.123.161)
        publish_rate: Publishing rate in Hz (default: 1.0)
    """
    
    MODE_NAMES = {
        0: 'IDLE',
        1: 'STANDING',
        2: 'WALKING',
    }
    
    def __init__(self):
        super().__init__('go2_state_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('publish_rate', 1.0)
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize Go2 interface
        self.go2 = Go2HighLevelInterface(robot_ip=self.robot_ip)
        
        # Publishers
        self.battery_pub = self.create_publisher(Float32, 'go2/battery', 10)
        self.mode_pub = self.create_publisher(Int32, 'go2/mode', 10)
        self.mode_str_pub = self.create_publisher(String, 'go2/mode_str', 10)
        
        # Connect to robot
        self.connected = self.go2.connect()
        if self.connected:
            self.get_logger().info(f'Connected to Go2 at {self.robot_ip}')
        else:
            self.get_logger().warn(f'Failed to connect to Go2 at {self.robot_ip}')
            self.get_logger().warn('Running in simulation mode')
        
        # Publishing timer
        period = 1.0 / self.publish_rate
        self.pub_timer = self.create_timer(period, self.publish_state)
        
        self.get_logger().info('Go2 State Publisher started')
    
    def publish_state(self):
        """Publish robot state"""
        if self.connected:
            state = self.go2.get_state()
            battery_level = state.battery_level
            mode = state.mode
        else:
            # Simulated values
            battery_level = 100.0
            mode = 1  # Standing
        
        # Publish battery
        battery_msg = Float32()
        battery_msg.data = battery_level
        self.battery_pub.publish(battery_msg)
        
        # Publish mode
        mode_msg = Int32()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        
        # Publish mode string
        mode_str_msg = String()
        mode_str_msg.data = self.MODE_NAMES.get(mode, 'UNKNOWN')
        self.mode_str_pub.publish(mode_str_msg)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.connected:
            self.go2.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2StatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


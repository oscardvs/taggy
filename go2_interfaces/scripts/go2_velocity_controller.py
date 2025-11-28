#!/usr/bin/env python3
"""
Unitree Go2 Velocity Controller Node
Subscribes to /cmd_vel and sends commands to the robot

EDTH Hackathon Starter Pack by Laelaps AI
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import os

# Add parent directory to path for local imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from go2_interfaces.go2_sdk import Go2HighLevelInterface


class Go2VelocityController(Node):
    """
    ROS2 node that bridges /cmd_vel commands to Unitree Go2
    
    Subscriptions:
        /cmd_vel (geometry_msgs/Twist): Velocity commands
            - linear.x: Forward velocity (m/s)
            - linear.y: Lateral velocity (m/s), positive = left
            - angular.z: Yaw rate (rad/s), positive = CCW
        
        /emergency_stop (std_msgs/Bool): Emergency stop trigger
    
    Parameters:
        robot_ip: IP address of Go2 (default: 192.168.123.161)
        max_linear_velocity: Max linear speed (m/s)
        max_angular_velocity: Max angular speed (rad/s)
        control_frequency: Command rate (Hz)
        timeout: Command timeout (s) - stops robot if no cmd received
    """
    
    def __init__(self):
        super().__init__('go2_velocity_controller')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('timeout', 0.5)
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.timeout = self.get_parameter('timeout').value
        
        # Initialize Go2 interface
        self.go2 = Go2HighLevelInterface(robot_ip=self.robot_ip)
        
        # State variables
        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.emergency_stop = False
        
        # QoS for cmd_vel (best effort for real-time control)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos
        )
        
        self.estop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.estop_callback,
            10
        )
        
        # Control timer
        period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(period, self.control_loop)
        
        # Connect to robot
        self.connected = self.go2.connect()
        if self.connected:
            self.get_logger().info(f'Connected to Go2 at {self.robot_ip}')
            self.go2.stand()  # Start in standing mode
        else:
            self.get_logger().warn(f'Failed to connect to Go2 at {self.robot_ip}')
            self.get_logger().warn('Running in simulation mode (commands will be logged)')
        
        self.get_logger().info('Go2 Velocity Controller started')
        self.get_logger().info(f'  Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Control freq: {self.control_freq} Hz')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        self.current_cmd = msg
        self.last_cmd_time = self.get_clock().now()
    
    def estop_callback(self, msg: Bool):
        """Handle emergency stop"""
        if msg.data and not self.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.emergency_stop = True
            if self.connected:
                self.go2.stop()
        elif not msg.data and self.emergency_stop:
            self.get_logger().info('Emergency stop released')
            self.emergency_stop = False
    
    def control_loop(self):
        """Main control loop - sends commands to robot"""
        # Check for emergency stop
        if self.emergency_stop:
            if self.connected:
                self.go2.stop()
            return
        
        # Check for command timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout:
            # No recent commands - stop robot
            if self.connected:
                self.go2.stop()
            return
        
        # Clamp velocities to limits
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, self.current_cmd.linear.x))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, self.current_cmd.linear.y))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, self.current_cmd.angular.z))
        
        # Send command to robot
        if self.connected:
            self.go2.set_velocity(vx, vy, wz)
        else:
            # Simulation mode - log commands
            if vx != 0 or vy != 0 or wz != 0:
                self.get_logger().debug(f'[SIM] cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down - stopping robot')
        if self.connected:
            self.go2.stop()
            self.go2.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2VelocityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


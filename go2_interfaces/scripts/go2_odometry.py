#!/usr/bin/env python3
"""
Unitree Go2 Odometry Publisher Node
Publishes robot odometry from Go2's internal state estimation

EDTH Hackathon Starter Pack by Laelaps AI
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import sys
import os
import math

# Add parent directory to path for local imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from go2_interfaces.go2_sdk import Go2HighLevelInterface, RobotState


class Go2Odometry(Node):
    """
    ROS2 node that publishes Go2 odometry and TF
    
    Publications:
        /odom (nav_msgs/Odometry): Robot odometry
            - pose: Robot position and orientation in odom frame
            - twist: Robot velocity in base_link frame
        
        /imu (sensor_msgs/Imu): IMU data from robot
        
        TF: odom -> base_link transform
    
    Parameters:
        robot_ip: IP address of Go2 (default: 192.168.123.161)
        odom_frame: Odometry frame ID (default: odom)
        base_frame: Robot base frame ID (default: base_link)
        publish_tf: Whether to publish TF (default: true)
        publish_rate: Publishing rate in Hz (default: 50.0)
    """
    
    def __init__(self):
        super().__init__('go2_odometry')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize Go2 interface
        self.go2 = Go2HighLevelInterface(robot_ip=self.robot_ip)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Simulated odometry for testing without robot
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_yaw = 0.0
        self.last_sim_time = self.get_clock().now()
        
        # Connect to robot
        self.connected = self.go2.connect()
        if self.connected:
            self.get_logger().info(f'Connected to Go2 at {self.robot_ip}')
        else:
            self.get_logger().warn(f'Failed to connect to Go2 at {self.robot_ip}')
            self.get_logger().warn('Running in simulation mode (publishing simulated odom)')
        
        # Publishing timer
        period = 1.0 / self.publish_rate
        self.pub_timer = self.create_timer(period, self.publish_odometry)
        
        self.get_logger().info('Go2 Odometry node started')
    
    def publish_odometry(self):
        """Publish odometry and TF"""
        now = self.get_clock().now()
        
        if self.connected:
            state = self.go2.get_state()
            self._publish_from_state(state, now)
        else:
            self._publish_simulated(now)
    
    def _publish_from_state(self, state: RobotState, stamp):
        """Publish odometry from robot state"""
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = state.z
        
        # Orientation (quaternion)
        odom.pose.pose.orientation.w = state.qw
        odom.pose.pose.orientation.x = state.qx
        odom.pose.pose.orientation.y = state.qy
        odom.pose.pose.orientation.z = state.qz
        
        # Velocity (in base_link frame)
        odom.twist.twist.linear.x = state.vx
        odom.twist.twist.linear.y = state.vy
        odom.twist.twist.linear.z = state.vz
        odom.twist.twist.angular.x = state.wx
        odom.twist.twist.angular.y = state.wy
        odom.twist.twist.angular.z = state.wz
        
        # Covariance (approximate values)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[14] = 0.01  # z
        odom.pose.covariance[21] = 0.01  # roll
        odom.pose.covariance[28] = 0.01  # pitch
        odom.pose.covariance[35] = 0.01  # yaw
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            self._publish_tf(state, stamp)
        
        # Publish IMU
        self._publish_imu(state, stamp)
    
    def _publish_simulated(self, stamp):
        """Publish simulated odometry (for testing without robot)"""
        # Calculate time delta
        dt = (stamp - self.last_sim_time).nanoseconds / 1e9
        self.last_sim_time = stamp
        
        # Create odometry message with zero velocity (standing still)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position (stationary at origin)
        odom.pose.pose.position.x = self.sim_x
        odom.pose.pose.position.y = self.sim_y
        odom.pose.pose.position.z = 0.35  # Approximate Go2 standing height
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.w = math.cos(self.sim_yaw / 2)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.sim_yaw / 2)
        
        # Zero velocity
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.sim_x
            t.transform.translation.y = self.sim_y
            t.transform.translation.z = 0.35
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
    
    def _publish_tf(self, state: RobotState, stamp):
        """Publish odom -> base_link TF"""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = state.z
        
        t.transform.rotation.w = state.qw
        t.transform.rotation.x = state.qx
        t.transform.rotation.y = state.qy
        t.transform.rotation.z = state.qz
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_imu(self, state: RobotState, stamp):
        """Publish IMU data"""
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.base_frame
        
        # Orientation
        imu.orientation.w = state.qw
        imu.orientation.x = state.qx
        imu.orientation.y = state.qy
        imu.orientation.z = state.qz
        
        # Angular velocity
        imu.angular_velocity.x = state.imu_gyro_x
        imu.angular_velocity.y = state.imu_gyro_y
        imu.angular_velocity.z = state.imu_gyro_z
        
        # Linear acceleration
        imu.linear_acceleration.x = state.imu_acc_x
        imu.linear_acceleration.y = state.imu_acc_y
        imu.linear_acceleration.z = state.imu_acc_z
        
        self.imu_pub.publish(imu)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.connected:
            self.go2.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2Odometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


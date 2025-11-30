#!/usr/bin/env python3
"""
Unitree Go2 Odometry Publisher Node
Publishes robot odometry from Go2's internal state estimation

EDTH Hackathon Starter Pack by Laelaps AI

Modified: Added +90° yaw correction to align base_link with odom frame
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
        yaw_correction_deg: Yaw correction in degrees (default: 90.0)
    """
    
    # Precomputed correction quaternion for +90° around Z
    # q = cos(θ/2) + sin(θ/2)*k where θ = π/2
    YAW_CORRECTION_W = 0.7071067811865476  # cos(45°)
    YAW_CORRECTION_Z = 0.7071067811865476  # sin(45°)
    
    def __init__(self):
        super().__init__('go2_odometry')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('yaw_correction_deg', 180.0)
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        yaw_correction_deg = self.get_parameter('yaw_correction_deg').value
        
        # Compute correction quaternion from parameter
        yaw_rad = math.radians(yaw_correction_deg)
        self.correction_qw = math.cos(yaw_rad / 2)
        self.correction_qz = math.sin(yaw_rad / 2)
        
        self.get_logger().info(f'Yaw correction: {yaw_correction_deg}° ({yaw_rad:.4f} rad)')
        
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
    
    def _correct_orientation(self, qw, qx, qy, qz):
        """
        Apply yaw correction using quaternion multiplication.
        
        Corrects the -90° offset between Go2's internal frame and ROS convention.
        Uses: q_corrected = q_robot * q_correction
        
        Args:
            qw, qx, qy, qz: Original quaternion from robot (wxyz order)
            
        Returns:
            Tuple of (qw, qx, qy, qz) corrected quaternion
        """
        # Correction quaternion (rotation around Z axis only)
        cw = self.correction_qw
        cx = 0.0
        cy = 0.0
        cz = self.correction_qz
        
        # Quaternion multiplication: q_robot * q_correction
        # This applies the correction in the robot's local frame
        new_w = cw * qw - cx * qx - cy * qy - cz * qz
        new_x = cw * qx + cx * qw + cy * qz - cz * qy
        new_y = cw * qy - cx * qz + cy * qw + cz * qx
        new_z = cz * qz + cx * qy - cy * qx + cw * qw
        
        return new_w, new_x, new_y, new_z
    
    def _correct_velocity(self, vx, vy, yaw_correction_rad):
        """
        Rotate velocity vector by yaw correction angle.
        
        Args:
            vx, vy: Original velocities
            yaw_correction_rad: Correction angle in radians
            
        Returns:
            Tuple of (vx_corrected, vy_corrected)
        """
        cos_yaw = math.cos(yaw_correction_rad)
        sin_yaw = math.sin(yaw_correction_rad)
        
        new_vx = vx * cos_yaw - vy * sin_yaw
        new_vy = vx * sin_yaw + vy * cos_yaw
        
        return new_vx, new_vy
    
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
        
        # Position (no correction needed for position)
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = state.z
        
        # Orientation (quaternion) - apply yaw correction
        qw, qx, qy, qz = self._correct_orientation(
            state.qw, state.qx, state.qy, state.qz
        )
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        
        # Velocity (in base_link frame) - apply rotation correction
        yaw_rad = math.radians(self.get_parameter('yaw_correction_deg').value)
        vx_corrected, vy_corrected = self._correct_velocity(state.vx, state.vy, yaw_rad)
        
        odom.twist.twist.linear.x = vx_corrected
        odom.twist.twist.linear.y = vy_corrected
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
            self._publish_tf(qw, qx, qy, qz, state, stamp)
        
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
        
        # Orientation (quaternion from yaw) - apply correction to simulated yaw too
        corrected_yaw = self.sim_yaw + math.radians(self.get_parameter('yaw_correction_deg').value)
        odom.pose.pose.orientation.w = math.cos(corrected_yaw / 2)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(corrected_yaw / 2)
        
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
    
    def _publish_tf(self, qw, qx, qy, qz, state: RobotState, stamp):
        """Publish odom -> base_link TF with corrected orientation"""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = state.z
        
        # Use already-corrected quaternion
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_imu(self, state: RobotState, stamp):
        """Publish IMU data with corrected orientation"""
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.base_frame
        
        # Orientation - apply correction
        qw, qx, qy, qz = self._correct_orientation(
            state.qw, state.qx, state.qy, state.qz
        )
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        
        # Angular velocity (no correction needed - already in body frame)
        imu.angular_velocity.x = state.imu_gyro_x
        imu.angular_velocity.y = state.imu_gyro_y
        imu.angular_velocity.z = state.imu_gyro_z
        
        # Linear acceleration (no correction needed - already in body frame)
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

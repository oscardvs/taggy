#!/usr/bin/env python3
"""
Odometry to TF Broadcaster for Unitree Go2
Subscribes to /lf/sportmodestate for position/velocity and publishes odom->base_link TF

Uses RPY values directly from IMU to construct quaternion, avoiding quaternion order ambiguity.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

# Import Unitree messages
from unitree_go.msg import SportModeState


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    Uses ZYX convention (yaw-pitch-roll) which is standard for ROS.
    
    Args:
        roll: Roll angle in radians (rotation around X)
        pitch: Pitch angle in radians (rotation around Y)
        yaw: Yaw angle in radians (rotation around Z)
    
    Returns:
        Tuple of (qw, qx, qy, qz)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz


class OdomFromSportModeNode(Node):
    """
    Creates odometry from Go2's SportModeState which has position and velocity.
    Uses RPY directly from IMU to avoid quaternion order ambiguity.
    """
    
    def __init__(self):
        super().__init__('odom_to_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Use BEST_EFFORT QoS to match Unitree robot topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to sportmodestate (has position + velocity)
        self.sport_sub = self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self.sportmode_callback,
            qos_profile
        )

        # Also try /sportmodestate as backup
        self.sport_sub2 = self.create_subscription(
            SportModeState,
            '/sportmodestate',
            self.sportmode_callback,
            qos_profile
        )

        # Publisher for odom topic
        self.odom_pub = self.create_publisher(Odometry, '/odom_from_imu', 10)

        self.msg_count = 0
        self.get_logger().info('=' * 50)
        self.get_logger().info('Odometry from SportModeState started')
        self.get_logger().info('  Subscribing to: /lf/sportmodestate, /sportmodestate')
        self.get_logger().info('  Publishing TF: odom -> base_link')
        self.get_logger().info('  Publishing: /odom_from_imu')
        self.get_logger().info('  Using RPY directly for quaternion (no rotation hack)')
        self.get_logger().info('=' * 50)

    def sportmode_callback(self, msg: SportModeState):
        """Process sportmodestate and broadcast TF + odometry"""
        self.msg_count += 1
        now = self.get_clock().now()
        
        # Extract position (x, y, z) - raw from robot
        pos_x = float(msg.position[0])
        pos_y = float(msg.position[1])
        pos_z = float(msg.position[2])
        
        # Extract velocity (vx, vy, vz) - raw from robot
        vel_x = float(msg.velocity[0])
        vel_y = float(msg.velocity[1])
        vel_z = float(msg.velocity[2])
        
        # Yaw speed
        yaw_speed = float(msg.yaw_speed)
        
        # Get RPY directly from IMU (avoids quaternion order ambiguity)
        imu = msg.imu_state
        roll = float(imu.rpy[0])
        pitch = float(imu.rpy[1])
        yaw_raw = float(imu.rpy[2])
        
        # Apply +90 degree rotation around Z to align base_link with robot forward
        yaw = yaw_raw + (math.pi / 2.0)  # +90 degrees
        
        # Convert RPY to quaternion using standard ROS convention
        qw, qx, qy, qz = quaternion_from_euler(roll, pitch, yaw)
        
        # Log first message
        if self.msg_count == 1:
            self.get_logger().info(f'First sportmodestate received!')
            self.get_logger().info(f'  Position: [{pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}] m')
            self.get_logger().info(f'  Velocity: [{vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f}] m/s')
            self.get_logger().info(f'  Raw yaw: {math.degrees(yaw_raw):.1f} deg, Adjusted: {math.degrees(yaw):.1f} deg (+90)')
            self.get_logger().info(f'  Quaternion: [{qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}]')
        
        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Position from robot's state estimate
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0  # Keep z at 0 for 2D SLAM
        
        # Orientation from RPY-derived quaternion
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        
        # Velocity
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.linear.z = vel_z
        odom.twist.twist.angular.z = yaw_speed
        
        self.odom_pub.publish(odom)
        
        # Periodic logging
        if self.msg_count % 500 == 0:
            self.get_logger().info(f'Processed {self.msg_count} msgs, pos=({pos_x:.2f}, {pos_y:.2f}), yaw={math.degrees(yaw_raw):.1f}° (raw)')


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromSportModeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Target Follower Node

Follows a tracked target using Nav2 path planning. Subscribes to a target pose
topic and sends NavigateToPose goals to Nav2, maintaining a configurable follow
distance.

Works with either:
- go2_tracker: /tracked_object/pose (camera frame) 
- go2_perception: /detection/target_pose (map frame)

Use topic remapping to connect to the desired perception source.

Subscriptions:
    /target_pose (geometry_msgs/PoseStamped) - Target position (remappable)
    /odom (nav_msgs/Odometry) - Robot odometry for distance calculations

Publications:
    /follower/goal (geometry_msgs/PoseStamped) - Current goal for visualization

Action Clients:
    /navigate_to_pose (nav2_msgs/NavigateToPose) - Nav2 navigation

Parameters:
    follow_distance (float): Distance to maintain from target [m]
    replan_threshold (float): Min target movement to trigger replan [m]
    replan_rate (float): Max replan frequency [Hz]
    goal_timeout (float): Nav2 goal timeout [s]
    target_frame (str): Planning frame (usually "map")
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


def euler_from_quaternion(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class TargetFollowerNode(Node):
    """
    Follows tracked targets using Nav2 path planning.
    
    Transforms target poses to map frame and sends NavigateToPose goals
    with a configurable follow distance offset.
    """

    def __init__(self):
        super().__init__('target_follower')

        # Declare parameters
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('replan_threshold', 0.5)
        self.declare_parameter('replan_rate', 1.0)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('enable_following', True)

        # Get parameters
        self.follow_distance = self.get_parameter('follow_distance').value
        self.replan_threshold = self.get_parameter('replan_threshold').value
        self.replan_rate = self.get_parameter('replan_rate').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.target_frame = self.get_parameter('target_frame').value
        self.enable_following = self.get_parameter('enable_following').value

        # State
        self.last_target_pose: Optional[PoseStamped] = None
        self.last_goal_pose: Optional[PoseStamped] = None
        self.current_goal_handle: Optional[ClientGoalHandle] = None
        self.robot_pose: Optional[PoseStamped] = None
        self.last_replan_time = self.get_clock().now()

        # TF2 for frame transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Callback group for async action calls
        self.callback_group = ReentrantCallbackGroup()

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self._target_callback,
            10,
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            sensor_qos
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/follower/goal',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/follower/status',
            10
        )

        # Nav2 action client
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group
        )

        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('TARGET FOLLOWER NODE INITIALIZED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  follow_distance: {self.follow_distance} m')
        self.get_logger().info(f'  replan_threshold: {self.replan_threshold} m')
        self.get_logger().info(f'  replan_rate: {self.replan_rate} Hz')
        self.get_logger().info(f'  target_frame: {self.target_frame}')
        self.get_logger().info(f'  enable_following: {self.enable_following}')
        self.get_logger().info('=' * 60)

        # Publish initial status
        self._publish_status('WAITING')

    def _odom_callback(self, msg: Odometry):
        """Store robot pose from odometry."""
        self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose

    def _target_callback(self, msg: PoseStamped):
        """Handle incoming target pose."""
        if not self.enable_following:
            return

        # Transform to target frame if needed
        target_in_map = self._transform_to_frame(msg, self.target_frame)
        if target_in_map is None:
            return

        self.last_target_pose = target_in_map

        # Check if we should replan
        if self._should_replan(target_in_map):
            self._send_follow_goal(target_in_map)

    def _transform_to_frame(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """Transform pose to target frame using TF2."""
        if pose.header.frame_id == target_frame:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return do_transform_pose_stamped(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF transform failed ({pose.header.frame_id} -> {target_frame}): {e}',
                throttle_duration_sec=1.0
            )
            return None

    def _should_replan(self, target_pose: PoseStamped) -> bool:
        """Determine if we should send a new goal."""
        now = self.get_clock().now()

        # Rate limiting
        time_since_replan = (now - self.last_replan_time).nanoseconds / 1e9
        min_replan_interval = 1.0 / self.replan_rate
        if time_since_replan < min_replan_interval:
            return False

        # No previous goal - always plan
        if self.last_goal_pose is None:
            return True

        # Check if target moved significantly
        dx = target_pose.pose.position.x - self.last_target_pose.pose.position.x if self.last_target_pose else 0
        dy = target_pose.pose.position.y - self.last_target_pose.pose.position.y if self.last_target_pose else 0
        target_movement = math.sqrt(dx * dx + dy * dy)

        return target_movement > self.replan_threshold

    def _compute_follow_goal(self, target_pose: PoseStamped) -> PoseStamped:
        """
        Compute goal pose with follow distance offset.
        
        The goal is placed between the robot and target, at follow_distance
        from the target. The goal orientation faces the target.
        """
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()

        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # Use robot pose if available, otherwise approach from behind
        if self.robot_pose is not None:
            # Transform robot pose to map frame
            robot_in_map = self._transform_to_frame(self.robot_pose, self.target_frame)
            if robot_in_map is not None:
                robot_x = robot_in_map.pose.position.x
                robot_y = robot_in_map.pose.position.y
            else:
                robot_x = target_x - self.follow_distance
                robot_y = target_y
        else:
            robot_x = target_x - self.follow_distance
            robot_y = target_y

        # Direction from robot to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.01:
            # Too close, use default direction
            dx, dy = 1.0, 0.0
            distance = 1.0

        # Normalize direction
        dx /= distance
        dy /= distance

        # Goal position: follow_distance away from target, toward robot
        goal.pose.position.x = target_x - dx * self.follow_distance
        goal.pose.position.y = target_y - dy * self.follow_distance
        goal.pose.position.z = 0.0

        # Goal orientation: face the target
        yaw = math.atan2(dy, dx)
        goal.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)

        return goal

    def _send_follow_goal(self, target_pose: PoseStamped):
        """Send NavigateToPose goal to Nav2."""
        # Wait for action server
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available', throttle_duration_sec=5.0)
            self._publish_status('NAV2_UNAVAILABLE')
            return

        # Cancel previous goal if active
        if self.current_goal_handle is not None:
            self.get_logger().debug('Cancelling previous goal')
            self.current_goal_handle.cancel_goal_async()

        # Compute follow goal
        goal_pose = self._compute_follow_goal(target_pose)
        self.last_goal_pose = goal_pose
        self.last_replan_time = self.get_clock().now()

        # Publish goal for visualization
        self.goal_pub.publish(goal_pose)

        # Create Nav2 goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.get_logger().info(
            f'Sending goal: ({goal_pose.pose.position.x:.2f}, '
            f'{goal_pose.pose.position.y:.2f})'
        )

        # Send goal async
        self._publish_status('NAVIGATING')
        send_goal_future = self.nav2_client.send_goal_async(
            nav_goal,
            feedback_callback=self._goal_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._publish_status('GOAL_REJECTED')
            self.current_goal_handle = None
            return

        self.get_logger().debug('Goal accepted')
        self.current_goal_handle = goal_handle

        # Get result async
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {distance:.2f} m', throttle_duration_sec=1.0)

    def _goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached!')
            self._publish_status('GOAL_REACHED')
        elif status == 5:  # CANCELED
            self.get_logger().info('Goal canceled (replanning)')
            self._publish_status('REPLANNING')
        elif status == 6:  # ABORTED
            self.get_logger().warn('Goal aborted by Nav2')
            self._publish_status('GOAL_ABORTED')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
            self._publish_status(f'STATUS_{status}')

        self.current_goal_handle = None

    def _publish_status(self, status: str):
        """Publish follower status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TargetFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cancel any active goal
        if node.current_goal_handle is not None:
            node.current_goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


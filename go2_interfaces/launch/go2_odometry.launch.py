#!/usr/bin/env python3
"""
Unitree Go2 Odometry Launch File
Launches the odometry node that publishes robot odometry from Go2's internal state estimation

Usage:
    ros2 launch go2_interfaces go2_odometry.launch.py
    ros2 launch go2_interfaces go2_odometry.launch.py robot_ip:=192.168.123.161

Publications:
    /odom (nav_msgs/Odometry): Robot odometry
    /imu (sensor_msgs/Imu): IMU data from robot
    TF: odom -> base_link transform

EDTH Hackathon Starter Pack by Laelaps AI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Go2 robot'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF (odom -> base_link)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )

    # ============================================================
    # GO2 ODOMETRY NODE
    # ============================================================
    # Publishes odometry and TF from Go2's internal state estimation.
    # Also publishes IMU data from the robot.
    # ============================================================
    go2_odometry_node = Node(
        package='go2_interfaces',
        executable='go2_odometry',
        name='go2_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        odom_frame_arg,
        base_frame_arg,
        publish_tf_arg,
        publish_rate_arg,
        go2_odometry_node,
    ])


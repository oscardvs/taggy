#!/usr/bin/env python3
"""
Step 3: Start quadruped controller (optional)
Run this AFTER the robot is standing in Gazebo

Usage:
    ros2 launch go2_gazebo 03_quadruped.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    quadruped_controller = Node(
        package='quadropted_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'verbose': False,
            'robot_id': 1,
        }],
        remappings=[
            ('joint_group_controller/commands', '/joint_group_position_controller/commands'),
        ],
    )

    cmd_vel_bridge = Node(
        package='quadropted_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odometry_node = Node(
        package='quadropted_controller',
        executable='QuadrupedOdometryNode.py',
        name='quadruped_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'verbose': False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'enable_odom_tf': True,
        }],
    )

    return LaunchDescription([
        LogInfo(msg='=== STEP 3: Starting quadruped controller ==='),
        quadruped_controller,
        cmd_vel_bridge,
        odometry_node,
    ])


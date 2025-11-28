#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 Control Interface Launch File
Launches the velocity control node that bridges ROS2 commands to Unitree API

Usage:
    ros2 launch go2_bringup go2_control.launch.py

This node subscribes to:
    /cmd_vel (geometry_msgs/Twist)     - Velocity commands
    /emergency_stop (std_msgs/Bool)    - Emergency stop
    /cmd_posture (std_msgs/String)     - Posture commands

And publishes to the Unitree API:
    /api/sport/request                 - Robot commands

by Laelaps AI
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # GO2 CONTROL NODE
    # ============================================================
    # Uses the Unitree Sport API to control the robot.
    # Commands are sent via DDS to /api/sport/request which is
    # handled by the robot's onboard sport_mode service.
    # ============================================================
    go2_control_node = Node(
        package='go2_control',
        executable='go2_control_node',
        name='go2_control',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        go2_control_node,
    ])

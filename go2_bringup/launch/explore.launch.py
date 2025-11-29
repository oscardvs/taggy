#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 Exploration Launch File
Launches explore_lite for autonomous frontier-based exploration

Usage:
    # Start autonomous exploration (requires SLAM and Nav2 already running)
    ros2 launch go2_bringup explore.launch.py

Prerequisites:
    - SLAM must be running (ros2 launch go2_bringup slam.launch.py)
    - Nav2 must be running (ros2 launch go2_bringup navigation.launch.py)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    go2_bringup_dir = get_package_share_directory('go2_bringup')

    # Config file path
    explore_params_file = os.path.join(go2_bringup_dir, 'config', 'explore_params.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ============================================================
    # EXPLORE_LITE - Frontier-Based Exploration
    # ============================================================
    # Autonomous exploration using frontier detection
    # Sends NavigateToPose goals to Nav2
    # Can be preempted by mission executive when target detected
    #
    # Publishes:
    #   - /explore/frontiers (visualization_msgs/MarkerArray)
    # Subscribes:
    #   - /map (nav_msgs/OccupancyGrid)
    # Actions:
    #   - Sends goals to /navigate_to_pose
    # ============================================================
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,

        # Exploration
        explore_node,
    ])


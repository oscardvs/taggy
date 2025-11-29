#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 Navigation Launch File
Launches Nav2 navigation stack only (requires SLAM or map server running)

Usage:
    # Start navigation (assumes SLAM or localization already running)
    ros2 launch go2_bringup navigation.launch.py
    
    # Then use RViz to send navigation goals
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    go2_bringup_dir = get_package_share_directory('go2_bringup')

    # Config file path
    nav2_params_file = os.path.join(go2_bringup_dir, 'config', 'nav2_params.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    # ============================================================
    # NAV2 NAVIGATION STACK
    # ============================================================
    # Includes:
    #   - Controller Server (DWB local planner)
    #   - Planner Server (NavFn global planner)
    #   - Behavior Server (recovery behaviors)
    #   - BT Navigator (behavior tree execution)
    #   - Costmap nodes (local + global)
    # ============================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': nav2_params_file,
        }.items()
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        autostart_arg,

        # Navigation
        nav2_bringup_launch,
    ])


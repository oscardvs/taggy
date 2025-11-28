#!/usr/bin/env python3
"""
Go2 Robot Full Bringup - Modular Hardware/Simulation Architecture

This launch file provides a unified interface for running the Go2 robot
in either hardware (real robot) mode or simulation (Gazebo) mode.

Usage:
    # Hardware mode (default) - Real Go2 robot
    ros2 launch go2_bringup full_bringup.launch.py

    # Simulation mode - Gazebo
    ros2 launch go2_bringup full_bringup.launch.py use_sim:=true

    # Simulation with specific world
    ros2 launch go2_bringup full_bringup.launch.py use_sim:=true world:=outdoor

    # Hardware mode with specific robot IP
    ros2 launch go2_bringup full_bringup.launch.py robot_ip:=192.168.123.161

Topics (both modes):
    /cmd_vel              - Velocity commands (geometry_msgs/Twist)
    /emergency_stop       - Emergency stop (std_msgs/Bool)
    /cmd_posture          - Posture commands (std_msgs/String)
    /scan                 - LiDAR scan (sensor_msgs/LaserScan)
    /camera/color/image_raw - RGB camera (sensor_msgs/Image)

by Laelaps AI
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_bringup = FindPackageShare('go2_bringup')
    pkg_gazebo = FindPackageShare('go2_gazebo')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (Gazebo) instead of hardware'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Unitree Go2 robot (hardware mode only)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world file (simulation mode only): empty, outdoor'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense camera (hardware) or simulated camera (sim)'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable Hesai LiDAR (hardware) or simulated LiDAR (sim)'
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='go2',
        description='Robot namespace'
    )

    # ============================================================
    # LOG MODE
    # ============================================================
    log_hardware_mode = LogInfo(
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        msg=['[GO2 BRINGUP] Starting in HARDWARE mode - Robot IP: ', LaunchConfiguration('robot_ip')]
    )

    log_simulation_mode = LogInfo(
        condition=IfCondition(LaunchConfiguration('use_sim')),
        msg=['[GO2 BRINGUP] Starting in SIMULATION mode - World: ', LaunchConfiguration('world')]
    )

    # ============================================================
    # HARDWARE MODE - Real Robot
    # ============================================================
    hardware_bringup = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        actions=[
            # Sensors (RealSense camera + Hesai LiDAR)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_bringup, 'launch', 'sensors.launch.py'])
                ]),
                launch_arguments={
                    'enable_camera': LaunchConfiguration('enable_camera'),
                    'enable_lidar': LaunchConfiguration('enable_lidar'),
                    'enable_rviz': LaunchConfiguration('enable_rviz'),
                }.items()
            ),

            # Control interface (Unitree API bridge)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_bringup, 'launch', 'go2_control.launch.py'])
                ]),
                launch_arguments={
                    'robot_ip': LaunchConfiguration('robot_ip'),
                }.items()
            ),
        ]
    )

    # ============================================================
    # SIMULATION MODE - Gazebo
    # ============================================================
    simulation_bringup = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_sim')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
                ]),
                launch_arguments={
                    'world': LaunchConfiguration('world'),
                    'use_camera': LaunchConfiguration('enable_camera'),
                    'use_lidar': LaunchConfiguration('enable_lidar'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'use_rviz': LaunchConfiguration('enable_rviz'),
                }.items()
            ),
        ]
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================
    return LaunchDescription([
        # Arguments
        use_sim_arg,
        robot_ip_arg,
        world_arg,
        enable_camera_arg,
        enable_lidar_arg,
        enable_rviz_arg,
        robot_name_arg,

        # Logging
        log_hardware_mode,
        log_simulation_mode,

        # Mode-specific bringup
        hardware_bringup,
        simulation_bringup,
    ])

#!/usr/bin/env python3
"""
EDTH Hackathon - Full Mission Bringup Launch File
Launches the complete autonomous search-and-find stack:
  - SLAM Toolbox
  - Nav2 Navigation
  - twist_mux
  - explore_lite
  - Perception (YOLOv8 + Depth Fusion)
  - Mission Executive FSM

Usage:
    # Full autonomous stack
    ros2 launch go2_bringup mission_bringup.launch.py
    
    # Without exploration (manual goals only)
    ros2 launch go2_bringup mission_bringup.launch.py enable_exploration:=false
    
    # With custom perception settings
    ros2 launch go2_bringup mission_bringup.launch.py \
        confidence_threshold:=0.5 model_path:=/opt/yolo/yolov8s.onnx
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_exploration_arg = DeclareLaunchArgument(
        'enable_exploration',
        default_value='true',
        description='Enable autonomous frontier exploration'
    )

    listening_timeout_arg = DeclareLaunchArgument(
        'listening_timeout',
        default_value='10.0',
        description='Timeout for voice command (seconds)'
    )

    arrival_distance_arg = DeclareLaunchArgument(
        'arrival_distance',
        default_value='0.5',
        description='Distance threshold for target reached (meters)'
    )

    # Perception arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/opt/yolo/yolov8n.onnx',
        description='Path to YOLOv8 ONNX model'
    )

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value='/opt/yolo/yolov8n.engine',
        description='Path to TensorRT engine (auto-built if missing)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.6',
        description='Detection confidence threshold [0.0-1.0]'
    )

    # ============================================================
    # FULL NAVIGATION STACK
    # ============================================================
    # Includes: SLAM, Nav2, twist_mux, explore_lite
    full_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('go2_bringup'),
                'launch',
                'full_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_exploration': LaunchConfiguration('enable_exploration'),
        }.items()
    )

    # ============================================================
    # PERCEPTION (YOLOv8 + Depth Fusion)
    # ============================================================
    # Object detection with TensorRT and 3D localization
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('go2_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_path': LaunchConfiguration('model_path'),
            'engine_path': LaunchConfiguration('engine_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }.items()
    )

    # ============================================================
    # MISSION EXECUTIVE
    # ============================================================
    # FSM that orchestrates the search-and-find behavior
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('go2_mission'),
                'launch',
                'mission.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'listening_timeout': LaunchConfiguration('listening_timeout'),
            'arrival_distance': LaunchConfiguration('arrival_distance'),
        }.items()
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_exploration_arg,
        listening_timeout_arg,
        arrival_distance_arg,
        model_path_arg,
        engine_path_arg,
        confidence_threshold_arg,

        # Navigation stack
        full_navigation_launch,

        # Perception (YOLOv8 + Depth)
        perception_launch,

        # Mission executive
        mission_launch,
    ])


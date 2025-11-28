#!/usr/bin/env python3
"""
Step 1: Start Gazebo (PAUSED) and spawn robot
Run this first, then run 02_controllers.launch.py

Usage:
    ros2 launch go2_gazebo 01_gazebo_spawn.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_go2_gazebo = get_package_share_directory('go2_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Process URDF
    xacro_file = os.path.join(pkg_go2_gazebo, 'xacro', 'go2_gazebo.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            'robot_name': 'go2',
            'use_camera': 'true',
            'use_lidar': 'true',
        }
    ).toxml()

    # Start Gazebo PAUSED
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_go2_gazebo, 'worlds', 'empty.world'),
            'verbose': 'true',
            'pause': 'true',  # PAUSED - no physics yet
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True}
        ],
    )

    # Spawn robot (while paused - won't fall)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_go2',
        arguments=[
            '-entity', 'go2',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.45',  # Standing height
            '-Y', '0.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='=== STEP 1: Starting Gazebo (PAUSED) and spawning robot ==='),
        LogInfo(msg='Robot will be frozen until you run 02_controllers.launch.py'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])


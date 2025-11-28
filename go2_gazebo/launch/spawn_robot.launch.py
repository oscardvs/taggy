#!/usr/bin/env python3
"""
Spawn Go2 Robot in an Already Running Gazebo Instance

Usage:
    ros2 launch go2_gazebo spawn_robot.launch.py
    ros2 launch go2_gazebo spawn_robot.launch.py robot_name:=go2_1 x:=1.0 y:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_go2_gazebo = get_package_share_directory('go2_gazebo')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='go2',
        description='Robot name and namespace'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable simulated RealSense camera'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable simulated LiDAR'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # ============================================================
    # PROCESS URDF/XACRO
    # ============================================================
    xacro_file = os.path.join(pkg_go2_gazebo, 'xacro', 'go2_gazebo.xacro')
    
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' use_camera:=', LaunchConfiguration('use_camera'),
        ' use_lidar:=', LaunchConfiguration('use_lidar'),
    ])

    robot_description = {'robot_description': robot_description_content}

    # ============================================================
    # NODES
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', [LaunchConfiguration('robot_name'), '/robot_description'],
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('robot_name'),
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('robot_name'),
        arguments=['joint_group_position_controller'],
        output='screen',
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    delayed_joint_group_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_group_controller],
        )
    )

    return LaunchDescription([
        robot_name_arg,
        use_camera_arg,
        use_lidar_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,

        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster,
        delayed_joint_group_controller,
    ])


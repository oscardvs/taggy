#!/usr/bin/env python3
"""
Go2 Gazebo Simulation Bringup (Standalone)

Quick launch file to start the Go2 robot in Gazebo simulation.
This is a convenience wrapper around go2_gazebo's launch file.

Usage:
    ros2 launch go2_bringup simulation.launch.py
    ros2 launch go2_bringup simulation.launch.py world:=outdoor
    ros2 launch go2_bringup simulation.launch.py use_camera:=false

by Laelaps AI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('go2_gazebo')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world: empty, outdoor'
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='go2',
        description='Robot namespace'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # ============================================================
    # INCLUDE GAZEBO LAUNCH
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_camera': LaunchConfiguration('use_camera'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'robot_name': LaunchConfiguration('robot_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    return LaunchDescription([
        world_arg,
        use_camera_arg,
        use_lidar_arg,
        use_rviz_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo_launch,
    ])


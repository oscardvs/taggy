#!/usr/bin/env python3
"""
EDTH Hackathon - Complete Unitree Go2 Bringup
Launches all sensors and robot control interfaces
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('go2_bringup')

    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Unitree Go2 robot'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode'
    )

    # Include sensors launch
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'sensors.launch.py'])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
        }.items()
    )

    # Include control launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'go2_control.launch.py'])
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        use_sim_arg,
        sensors_launch,
        control_launch,
    ])


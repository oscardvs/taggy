#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 SLAM Launch File
Launches SLAM Toolbox for mapping

Usage:
    # Start SLAM mapping
    ros2 launch go2_bringup slam.launch.py
    
    # Save map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/map
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
    slam_params_file = os.path.join(go2_bringup_dir, 'config', 'slam_toolbox_params.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ============================================================
    # ODOMETRY TO TF BROADCASTER
    # ============================================================
    # Subscribes to /lowstate IMU and publishes odom->base_link TF
    # ============================================================
    odom_to_tf_node = Node(
        package='go2_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
    )

    # ============================================================
    # SLAM TOOLBOX - Online Async (Lifelong Mapping)
    # ============================================================
    # Provides:
    #   - /map topic (nav_msgs/OccupancyGrid)
    #   - map -> odom TF transform
    # Subscribes:
    #   - /scan (sensor_msgs/LaserScan)
    #   - /odom TF
    # ============================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,

        # TF broadcaster (reads from /lowstate IMU)
        odom_to_tf_node,

        # SLAM
        slam_toolbox_node,
    ])


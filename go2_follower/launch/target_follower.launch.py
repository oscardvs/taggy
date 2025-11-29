#!/usr/bin/env python3
"""
Target Follower Launch File

Launches the target_follower_node with appropriate topic remappings based on
the perception source.

Usage:
    # With go2_tracker (default)
    ros2 launch go2_follower target_follower.launch.py

    # With go2_perception
    ros2 launch go2_follower target_follower.launch.py perception_source:=perception

    # Override follow distance
    ros2 launch go2_follower target_follower.launch.py follow_distance:=1.5

    # Full example with tracker
    ros2 launch go2_follower target_follower.launch.py \
        perception_source:=tracker \
        follow_distance:=1.0 \
        replan_threshold:=0.3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Generate launch description based on perception_source argument."""
    
    # Get launch configuration values
    perception_source = LaunchConfiguration('perception_source').perform(context)
    follow_distance = LaunchConfiguration('follow_distance').perform(context)
    replan_threshold = LaunchConfiguration('replan_threshold').perform(context)
    replan_rate = LaunchConfiguration('replan_rate').perform(context)
    
    # Get package directory
    pkg_share = get_package_share_directory('go2_follower')
    config_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')
    
    # Determine topic remapping based on perception source
    if perception_source == 'tracker':
        # go2_tracker publishes to /tracked_object/pose
        remappings = [
            ('/target_pose', '/tracked_object/pose'),
        ]
    elif perception_source == 'perception':
        # go2_perception publishes to /detection/target_pose
        remappings = [
            ('/target_pose', '/detection/target_pose'),
        ]
    else:
        # Custom - no remapping, user provides their own topic
        remappings = []
    
    # Build parameter overrides (only non-empty values)
    param_overrides = {}
    if follow_distance:
        param_overrides['follow_distance'] = float(follow_distance)
    if replan_threshold:
        param_overrides['replan_threshold'] = float(replan_threshold)
    if replan_rate:
        param_overrides['replan_rate'] = float(replan_rate)
    
    # Create node
    target_follower_node = Node(
        package='go2_follower',
        executable='target_follower_node.py',
        name='target_follower',
        output='screen',
        parameters=[config_file, param_overrides],
        remappings=remappings,
    )
    
    return [target_follower_node]


def generate_launch_description():
    # Get package directory for default config
    pkg_share = get_package_share_directory('go2_follower')
    default_config = os.path.join(pkg_share, 'config', 'follower_params.yaml')
    
    return LaunchDescription([
        # ============================================================
        # LAUNCH ARGUMENTS
        # ============================================================
        DeclareLaunchArgument(
            'perception_source',
            default_value='tracker',
            description='Perception source: "tracker" (go2_tracker), "perception" (go2_perception), or "custom"'
        ),
        
        DeclareLaunchArgument(
            'follow_distance',
            default_value='',
            description='Distance to maintain from target [m] (overrides config)'
        ),
        
        DeclareLaunchArgument(
            'replan_threshold',
            default_value='',
            description='Min target movement to trigger replan [m] (overrides config)'
        ),
        
        DeclareLaunchArgument(
            'replan_rate',
            default_value='',
            description='Max replan frequency [Hz] (overrides config)'
        ),
        
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to follower config file'
        ),
        
        # ============================================================
        # LAUNCH SETUP (generates node with remappings)
        # ============================================================
        OpaqueFunction(function=launch_setup),
    ])


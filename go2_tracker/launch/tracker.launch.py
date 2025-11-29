"""
Main Tracker Launch File

Launches the threat tracker with configuration from tracker_config.yaml.

Usage:
    # Use default config
    ros2 launch go2_tracker tracker.launch.py
    
    # Override threat_id
    ros2 launch go2_tracker tracker.launch.py threat_id:=person
    
    # Enable multi-threat mode with reference image
    ros2 launch go2_tracker tracker.launch.py \
        env_with_multiple_threats:=true \
        reference_image:=/path/to/reference.jpg
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package share directory
    pkg_share = get_package_share_directory('go2_tracker')
    default_config = os.path.join(pkg_share, 'config', 'tracker_config.yaml')
    
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to tracker config file'
    )
    
    threat_id_arg = DeclareLaunchArgument(
        'threat_id',
        default_value='person',
        description='Class name to track. Examples: dog, person, cat'
    )
    
    multi_threat_arg = DeclareLaunchArgument(
        'env_with_multiple_threats',
        default_value='false',
        description='Set to true if multiple objects of same class may be present'
    )
    
    reference_arg = DeclareLaunchArgument(
        'reference_image',
        default_value='',
        description='Path to reference image (for multi-threat mode)'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolo11n.pt',
        description='YOLO11 model: yolo11n.pt, yolo11s.pt, yolo11m.pt'
    )
    
    # Tracker node - config FIRST, then override with launch args
    tracker_node = Node(
        package='go2_tracker',
        executable='tracker_node.py',
        name='tracker',
        output='screen',
        parameters=[
            # LaunchConfiguration('config'),  # Load config file FIRST
            {
                # Override with launch arguments SECOND
                'threat_id': LaunchConfiguration('threat_id'),
                'env_with_multiple_threats': LaunchConfiguration('env_with_multiple_threats'),
                'reference_image_path': LaunchConfiguration('reference_image'),
                'model': LaunchConfiguration('model'),
            }
        ],
    )
    
    return LaunchDescription([
        config_arg,
        threat_id_arg,
        multi_threat_arg,
        reference_arg,
        model_arg,
        tracker_node,
    ])
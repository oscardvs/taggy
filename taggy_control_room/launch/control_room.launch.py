"""
TAGGY Control Room Launch File

Launches:
- ROS Bridge WebSocket server (port 9090)
- Voice command node (optional)
- Mission monitor node (watches for targets, launches tracker)
- Web server for UI (port 8080)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_share = get_package_share_directory('taggy_control_room')
    
    # Arguments
    launch_package_arg = DeclareLaunchArgument(
        'launch_package', default_value='go2_tracker',
        description='Package containing the autonomy launch file'
    )
    
    launch_file_arg = DeclareLaunchArgument(
        'launch_file', default_value='tracker.launch.py',
        description='Launch file for autonomy pipeline'
    )
    
    # ROS Bridge WebSocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )
    
    # Mission monitor node - watches for targets and launches tracker
    mission_monitor_node = Node(
        package='taggy_control_room',
        executable='mission_monitor_node',
        name='mission_monitor',
        output='screen',
        parameters=[{
            'launch_package': LaunchConfiguration('launch_package'),
            'launch_file': LaunchConfiguration('launch_file'),
        }]
    )
    
    # Voice command node (for future ROS-native mic input)
    voice_node = Node(
        package='taggy_control_room',
        executable='voice_command_node',
        name='voice_command',
        output='screen',
    )
    
    # Web server for control room UI
    control_room_dir = os.path.join(pkg_share, 'control_room')
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', control_room_dir],
        output='screen'
    )
    
    return LaunchDescription([
        launch_package_arg,
        launch_file_arg,
        rosbridge_node,
        mission_monitor_node,
        voice_node,
        web_server,
    ])
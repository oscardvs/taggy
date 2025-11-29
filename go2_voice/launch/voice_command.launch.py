#!/usr/bin/env python3
"""
Voice Command Node Launch File (Standalone)

Launches only the voice_command_node for testing on a laptop without the
full mission stack. Useful for development and debugging.

Usage:
    ros2 launch go2_voice voice_command.launch.py

    # With custom model path
    ros2 launch go2_voice voice_command.launch.py \
        model_path:=/home/user/vosk-models/vosk-model-small-en-us-0.15

    # With USB microphone (PyAudio device)
    ros2 launch go2_voice voice_command.launch.py use_pyaudio:=true

Testing:
    # Simulate LISTENING state
    ros2 topic pub /mission/state std_msgs/String "data: 'LISTENING'" -1

    # Monitor recognized keywords
    ros2 topic echo /mission/target_object
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    go2_voice_dir = get_package_share_directory('go2_voice')

    # Config file path
    vosk_params_file = os.path.join(go2_voice_dir, 'config', 'vosk_params.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='~/vosk-models/vosk-model-small-en-us-0.15',
        description='Path to Vosk model directory (supports ~ and $HOME expansion)'
    )

    audio_topic_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='/audiohub/data',
        description='ROS topic for audio input'
    )

    use_pyaudio_arg = DeclareLaunchArgument(
        'use_pyaudio',
        default_value='false',
        description='Use PyAudio for local microphone instead of ROS topic'
    )

    # ============================================================
    # VOICE COMMAND NODE
    # ============================================================
    # Note: Parameters are merged in order - dict parameters override YAML
    voice_command_node = Node(
        package='go2_voice',
        executable='voice_command_node.py',
        name='voice_command_node',
        output='screen',
        parameters=[
            # Load YAML first (provides defaults)
            vosk_params_file,
            # Override with launch arguments (these take precedence)
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': LaunchConfiguration('model_path'),
                'audio_topic': LaunchConfiguration('audio_topic'),
                'use_pyaudio': LaunchConfiguration('use_pyaudio'),
            }
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        model_path_arg,
        audio_topic_arg,
        use_pyaudio_arg,

        # Voice Command Node
        voice_command_node,
    ])

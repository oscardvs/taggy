#!/usr/bin/env python3
"""
EDTH Hackathon - Mission Executive Launch File
Launches the mission_executive_node FSM for autonomous search-and-find behavior

Usage:
    # Launch mission executive only (requires navigation stack already running)
    ros2 launch go2_mission mission.launch.py
    
    # With custom parameters
    ros2 launch go2_mission mission.launch.py listening_timeout:=15.0 arrival_distance:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    go2_mission_dir = get_package_share_directory('go2_mission')

    # Config file paths
    mission_params_file = os.path.join(go2_mission_dir, 'config', 'mission_params.yaml')
    
    # Voice command config is in go2_voice package
    go2_voice_dir = get_package_share_directory('go2_voice')
    vosk_params_file = os.path.join(go2_voice_dir, 'config', 'vosk_params.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    listening_timeout_arg = DeclareLaunchArgument(
        'listening_timeout',
        default_value='10.0',
        description='Timeout for voice command in LISTENING state (seconds)'
    )

    target_lost_timeout_arg = DeclareLaunchArgument(
        'target_lost_timeout',
        default_value='2.0',
        description='Timeout before resuming search when target lost (seconds)'
    )

    arrival_distance_arg = DeclareLaunchArgument(
        'arrival_distance',
        default_value='0.5',
        description='Distance threshold for target reached (meters)'
    )

    # Visual servo arguments
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.3',
        description='Minimum obstacle distance for safety reflex (meters)'
    )

    servo_kp_yaw_arg = DeclareLaunchArgument(
        'servo_kp_yaw',
        default_value='1.0',
        description='Proportional gain for yaw control'
    )

    servo_kp_distance_arg = DeclareLaunchArgument(
        'servo_kp_distance',
        default_value='0.5',
        description='Proportional gain for distance control'
    )

    # ============================================================
    # MISSION EXECUTIVE NODE
    # ============================================================
    # Finite State Machine that orchestrates:
    #   - IDLE: Waiting for trigger (L2+A safety combination)
    #   - LISTENING: Waiting for voice command
    #   - SEARCHING: Exploring with explore_lite
    #   - TRACKING: Visual servo to target (velocity handled by visual_servo_node)
    #   - ARRIVED: Mission complete
    #   - FAILED: Exploration exhausted
    #
    # Subscriptions:
    #   /wirelesscontroller - Trigger detection
    #   /mission/target_object - Voice command result
    #   /detection/bbox - Object detections
    #   /detection/target_pose - 3D target position
    #   /odom - Robot odometry
    #
    # Publications:
    #   /cmd_posture - Posture commands
    #   /mission/state - Current FSM state (used by visual_servo_node)
    # ============================================================
    mission_executive_node = Node(
        package='go2_mission',
        executable='mission_executive_node',
        name='mission_executive_node',
        output='screen',
        parameters=[
            mission_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'listening_timeout': LaunchConfiguration('listening_timeout'),
                'target_lost_timeout': LaunchConfiguration('target_lost_timeout'),
                'arrival_distance': LaunchConfiguration('arrival_distance'),
            }
        ],
    )

    # ============================================================
    # VOICE COMMAND NODE
    # ============================================================
    # Offline speech recognition using Vosk ASR engine.
    # Listens to Go2's built-in microphone and recognizes target
    # object keywords from a constrained vocabulary.
    #
    # Subscriptions:
    #   /audiohub/data - Raw audio from Go2 microphone
    #   /mission/state - Current FSM state (only active in LISTENING)
    #
    # Publications:
    #   /mission/target_object - Recognized target object keyword
    # ============================================================
    voice_command_node = Node(
        package='go2_voice',
        executable='voice_command_node.py',
        name='voice_command_node',
        output='screen',
        parameters=[
            vosk_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # ============================================================
    # VISUAL SERVO NODE
    # ============================================================
    # Image-Based Visual Servoing (IBVS) controller with LiDAR safety.
    # Implements reactive control to track detected targets.
    #
    # Control Laws:
    #   Yaw:     ω_z = K_p_yaw * (u_center - u_target) / image_width
    #   Forward: v_x = clamp(K_p_dist * (d_current - d_goal), 0, v_max)
    #
    # Safety:
    #   If min(scan[frontal_cone]) < safety_distance: v_x = 0
    #
    # Subscriptions:
    #   /mission/state - Activate in TRACKING state
    #   /mission/target_object - Target class name
    #   /detection/bbox - 2D bounding boxes
    #   /detection/target_pose - 3D target position (for depth)
    #   /scan - LiDAR for safety
    #
    # Publications:
    #   /servo_cmd_vel - Velocity command (priority 2 in twist_mux)
    # ============================================================
    visual_servo_node = Node(
        package='go2_mission',
        executable='visual_servo_node',
        name='visual_servo_node',
        output='screen',
        parameters=[
            mission_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'safety_distance': LaunchConfiguration('safety_distance'),
                'servo_kp_yaw': LaunchConfiguration('servo_kp_yaw'),
                'servo_kp_distance': LaunchConfiguration('servo_kp_distance'),
                'goal_distance': LaunchConfiguration('arrival_distance'),
            }
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        listening_timeout_arg,
        target_lost_timeout_arg,
        arrival_distance_arg,
        safety_distance_arg,
        servo_kp_yaw_arg,
        servo_kp_distance_arg,

        # Mission Executive
        mission_executive_node,

        # Voice Command (Vosk ASR)
        voice_command_node,

        # Visual Servo (IBVS + Safety)
        visual_servo_node,
    ])


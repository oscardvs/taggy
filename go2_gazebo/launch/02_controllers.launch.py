#!/usr/bin/env python3
"""
Step 2: Load controllers, send standing pose, then unpause Gazebo
Run this AFTER 01_gazebo_spawn.launch.py

Uses proper event chain: load_controllers → pose_commands → wait → unpause
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # Step 1: Load joint state broadcaster
    load_joint_state = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen',
    )

    # Step 2: Load position controller
    load_position = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_group_position_controller'],
        output='screen',
    )

    # Step 3: Send standing pose CONTINUOUSLY
    # Joint order: rf, lf, rh, lh (matches quadropted_controller IK output)
    send_standing_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '50',
            '/joint_group_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]}'
        ],
        output='screen',
    )

    # Step 4: Unpause Gazebo - triggered AFTER pose publishing starts
    unpause_gazebo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
        output='screen',
    )

    # SEQUENTIAL CHAIN:
    # 1. Load joint_state_broadcaster (runs immediately)
    # 2. After 2s → load position controller  
    # 3. After 4s → start pose commands
    # 4. After 6s → unpause (pose has been sending for 2s already)
    
    delayed_load_position = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='>>> Loading joint_group_position_controller...'),
            load_position,
        ]
    )
    
    delayed_send_pose = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='>>> Starting standing pose commands at 50Hz...'),
            send_standing_pose,
        ]
    )
    
    delayed_unpause = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='>>> UNPAUSING GAZEBO NOW - pose commands already running!'),
            unpause_gazebo,
        ]
    )

    return LaunchDescription([
        LogInfo(msg='=== STEP 2: Loading controllers (sequential timers) ==='),
        LogInfo(msg='Timeline: 0s=joint_state, 2s=position_ctrl, 5s=pose_cmds, 8s=unpause'),
        load_joint_state,
        delayed_load_position,
        delayed_send_pose,
        delayed_unpause,
    ])

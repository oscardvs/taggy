#!/usr/bin/env python3
"""
Go2 Teleoperation Launch File

Launches keyboard or joystick teleoperation for the Go2 robot.
Works with both hardware and simulation modes.

Usage:
    # Keyboard teleop (custom for Go2)
    ros2 launch go2_bringup teleop.launch.py

    # Standard teleop_twist_keyboard
    ros2 launch go2_bringup teleop.launch.py teleop_type:=keyboard_twist

    # Joystick teleop
    ros2 launch go2_bringup teleop.launch.py teleop_type:=joystick

Controls (Go2 keyboard teleop):
    Movement: W/S (forward/back), A/D (strafe), Q/E (rotate)
    Posture: 1=Stand, 2=Sit, 3=Balance, 4=Recovery, 5=Hello
    Speed: +/- to adjust
    Emergency: X=E-stop, R=Release
    Quit: ESC

by Laelaps AI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    teleop_type_arg = DeclareLaunchArgument(
        'teleop_type',
        default_value='go2_keyboard',
        description='Teleop type: go2_keyboard, keyboard_twist, joystick'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device (for joystick teleop)'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Default linear speed (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Default angular speed (rad/s)'
    )

    # ============================================================
    # GO2 CUSTOM KEYBOARD TELEOP
    # ============================================================
    go2_keyboard_teleop = Node(
        package='go2_examples',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        output='screen',
        condition=LaunchConfigurationEquals('teleop_type', 'go2_keyboard'),
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
        }],
        prefix='xterm -e',
    )

    # ============================================================
    # STANDARD TELEOP_TWIST_KEYBOARD
    # ============================================================
    keyboard_twist_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        condition=LaunchConfigurationEquals('teleop_type', 'keyboard_twist'),
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
        prefix='xterm -e',
    )

    # ============================================================
    # JOYSTICK TELEOP
    # ============================================================
    joystick_group = GroupAction(
        condition=LaunchConfigurationEquals('teleop_type', 'joystick'),
        actions=[
            # Joy node
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'dev': LaunchConfiguration('joy_dev'),
                    'deadzone': 0.1,
                    'autorepeat_rate': 20.0,
                }],
            ),
            # Teleop joy node
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy',
                parameters=[{
                    'axis_linear.x': 1,
                    'axis_linear.y': 0,
                    'axis_angular.yaw': 3,
                    'scale_linear.x': 0.5,
                    'scale_linear.y': 0.3,
                    'scale_angular.yaw': 1.0,
                    'enable_button': 4,
                    'enable_turbo_button': 5,
                    'scale_linear_turbo.x': 1.0,
                }],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                ],
            ),
        ]
    )

    return LaunchDescription([
        teleop_type_arg,
        cmd_vel_topic_arg,
        joy_dev_arg,
        linear_speed_arg,
        angular_speed_arg,

        go2_keyboard_teleop,
        keyboard_twist_teleop,
        joystick_group,
    ])


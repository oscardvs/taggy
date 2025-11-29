#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 Navigation Launch File
Launches Nav2, SLAM Toolbox, explore_lite, and twist_mux for autonomous exploration

Usage:
    # Full autonomous exploration with SLAM
    ros2 launch go2_bringup full_navigation.launch.py
    
    # Navigation only (no auto-exploration, use RViz for manual goals)
    ros2 launch go2_bringup full_navigation.launch.py enable_exploration:=false
    
    # With sensors (if not already running)
    ros2 launch go2_bringup full_navigation.launch.py launch_sensors:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    go2_bringup_dir = get_package_share_directory('go2_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Config file paths
    nav2_params_file = os.path.join(go2_bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(go2_bringup_dir, 'config', 'slam_toolbox_params.yaml')
    explore_params_file = os.path.join(go2_bringup_dir, 'config', 'explore_params.yaml')
    twist_mux_params_file = os.path.join(go2_bringup_dir, 'config', 'twist_mux.yaml')

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    enable_exploration_arg = DeclareLaunchArgument(
        'enable_exploration',
        default_value='true',
        description='Enable autonomous frontier exploration'
    )

    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='false',
        description='Launch sensors (set true if sensors not already running)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    # ============================================================
    # SENSORS (Optional - if not already running)
    # ============================================================
    #sensors_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        PathJoinSubstitution([
    #            FindPackageShare('go2_bringup'),
    #            'launch',
    #            'sensors.launch.py'
    #        ])
    #    ]),
    #    condition=IfCondition(LaunchConfiguration('launch_sensors'))
    #)

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

    # ============================================================
    # NAV2 NAVIGATION STACK
    # ============================================================
    # Includes:
    #   - Controller Server (DWB local planner)
    #   - Planner Server (NavFn global planner)
    #   - Behavior Server (recovery behaviors)
    #   - BT Navigator (behavior tree execution)
    #   - Costmap nodes (local + global)
    #
    # Note: /cmd_vel is remapped to /nav_cmd_vel for twist_mux
    # ============================================================
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/nav_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'params_file': nav2_params_file,
                }.items()
            )
        ]
    )

    # ============================================================
    # TWIST MUX - Velocity Command Multiplexer
    # ============================================================
    # Prioritizes velocity commands from multiple sources:
    #   Priority 2: /servo_cmd_vel (visual servo)
    #   Priority 1: /nav_cmd_vel (Nav2 navigation)
    #   Priority 0: /teleop_cmd_vel (manual control)
    #
    # Output: /cmd_vel (to go2_control_node)
    # ============================================================
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file],
        remappings=[
            ('cmd_vel_out', 'cmd_vel')
        ]
    )

    # ============================================================
    # EXPLORE_LITE - Frontier-Based Exploration
    # ============================================================
    # Autonomous exploration using frontier detection
    # Sends NavigateToPose goals to Nav2
    # Can be preempted by mission executive when target detected
    # ============================================================
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_exploration'))
    )

    # ============================================================
    # LIFECYCLE MANAGER for SLAM
    # ============================================================
    # Note: Nav2 has its own lifecycle manager via nav2_bringup
    # SLAM Toolbox doesn't use lifecycle by default in async mode

    return LaunchDescription([
        # Arguments
        enable_exploration_arg,
        launch_sensors_arg,
        use_sim_time_arg,
        autostart_arg,

        # Optional sensors
        #sensors_launch,

        # SLAM
        slam_toolbox_node,

        # Velocity command multiplexer
        twist_mux_node,

        # Navigation (outputs to /nav_cmd_vel for twist_mux)
        nav2_bringup_launch,

        # Exploration
        explore_node,
    ])


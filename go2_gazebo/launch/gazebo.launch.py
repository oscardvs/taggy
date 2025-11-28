#!/usr/bin/env python3
"""
Go2 Gazebo Simulation Launch File - Single consolidated launch
Properly sequences: Gazebo(paused) -> Spawn -> Controllers -> Unpause

Usage:
    ros2 launch go2_gazebo gazebo.launch.py
    ros2 launch go2_gazebo gazebo.launch.py world:=outdoor
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    """Generate launch description with proper sequencing."""
    
    # Package paths
    pkg_go2_gazebo = get_package_share_directory('go2_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Resolve launch configurations
    world_name = LaunchConfiguration('world').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # ============================================================
    # STEP 1: PROCESS URDF/XACRO
    # ============================================================
    xacro_file = os.path.join(pkg_go2_gazebo, 'xacro', 'go2_gazebo.xacro')
    
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            'robot_name': robot_name,
            'use_camera': use_camera,
            'use_lidar': use_lidar,
        }
    ).toxml()

    robot_description = {'robot_description': robot_description_content}

    # ============================================================
    # STEP 2: START GAZEBO (PAUSED)
    # ============================================================
    world_file = os.path.join(pkg_go2_gazebo, 'worlds', f'{world_name}.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'true',  # CRITICAL: Start paused
        }.items()
    )

    # ============================================================
    # STEP 3: ROBOT STATE PUBLISHER
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time == 'true'}
        ],
    )

    # ============================================================
    # STEP 4: SPAWN ROBOT (while paused - no physics yet)
    # ============================================================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_go2',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            # NO -unpause here! We unpause after controllers are ready
        ],
        output='screen',
    )

    # ============================================================
    # STEP 5: LOAD CONTROLLERS (while still paused)
    # ============================================================
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen',
    )

    joint_group_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_group_position_controller'],
        output='screen',
    )

    # ============================================================
    # STEP 6: UNPAUSE GAZEBO (after controllers are ready)
    # ============================================================
    unpause_gazebo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
        output='screen',
    )

    # ============================================================
    # SEQUENCING: spawn -> controllers -> unpause
    # ============================================================
    
    # After spawn, wait 3s for ros2_control to initialize, then load joint_state_broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg='Robot spawned, waiting for ros2_control...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg='Loading joint_state_broadcaster...'),
                        joint_state_broadcaster_spawner,
                    ],
                )
            ],
        )
    )

    # After joint_state_broadcaster, load position controller
    load_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg='Loading joint_group_position_controller...'),
                joint_group_controller_spawner,
            ],
        )
    )

    # After position controller is loaded, UNPAUSE Gazebo
    start_physics = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_group_controller_spawner,
            on_exit=[
                LogInfo(msg='Controllers ready, unpausing Gazebo physics...'),
                unpause_gazebo,
            ],
        )
    )

    # ============================================================
    # QUADRUPED CONTROLLER (starts after physics)
    # ============================================================
    quadruped_controller = Node(
        package='quadropted_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'verbose': False,
            'robot_id': 1,
        }],
        remappings=[
            ('joint_group_controller/commands', '/joint_group_position_controller/commands'),
        ],
    )

    cmd_vel_bridge = Node(
        package='quadropted_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odometry_node = Node(
        package='quadropted_controller',
        executable='QuadrupedOdometryNode.py',
        name='quadruped_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'verbose': False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'enable_odom_tf': True,
        }],
    )

    # Start quadruped nodes after unpause
    start_quadruped = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=unpause_gazebo,
            on_exit=[
                LogInfo(msg='Physics started, launching quadruped controller...'),
                quadruped_controller,
                cmd_vel_bridge,
                odometry_node,
            ],
        )
    )

    # ============================================================
    # RVIZ (optional)
    # ============================================================
    rviz_config = os.path.join(pkg_go2_gazebo, 'rviz', 'go2_gazebo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(PythonExpression(['"', use_rviz, '" == "true"'])),
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
    )

    # ============================================================
    # RETURN ALL ACTIONS
    # ============================================================
    return [
        # Core launch
        gazebo,
        robot_state_publisher,
        spawn_robot,
        # Sequenced controller loading
        load_joint_state_broadcaster,
        load_position_controller,
        start_physics,
        # Quadruped after physics
        start_quadruped,
        # Visualization
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('world', default_value='empty', description='World file (empty, outdoor)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_camera', default_value='true', description='Enable RealSense camera'),
        DeclareLaunchArgument('use_lidar', default_value='true', description='Enable LiDAR'),
        DeclareLaunchArgument('robot_name', default_value='go2', description='Robot name'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial Y'),
        DeclareLaunchArgument('z', default_value='0.45', description='Initial Z (standing height)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'),
        # Execute setup
        OpaqueFunction(function=launch_setup),
    ])

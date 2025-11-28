from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.environ['AMENT_PREFIX_PATH'].split(':')[0], 'share', 'go2_simulation'
    )

    world_path = os.path.join(pkg_share, 'worlds', 'coworking.sdf')

    return LaunchDescription([
        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the Unitree GO2 robot
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=[
        #         '-entity', 'go2',
        #         '-file', os.path.join(pkg_share, 'models', 'go2_robot', 'robot.urdf')
        #     ],
        #     output='screen'
        # )
    ])

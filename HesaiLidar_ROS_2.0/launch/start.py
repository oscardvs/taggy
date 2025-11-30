from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('hesai_ros_driver'),
        'rviz',
        'rviz2.rviz'
    )

    # Static TF: base_link -> hesai_lidar
    # LiDAR position relative to robot center (Go2 IMU frame)
    # odom -> base_link is published by go2_odometry node
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hesai_lidar_tf',
        arguments=[
            '0.1710', '0.0', '0.0908',   # x, y, z (m)
            '0', '0', '0',               # roll, pitch, yaw (rad)
            'base_link',                 # parent
            'hesai_lidar'                # child
        ]
    )

    hesai_node = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen'
    )

    rviz = Node(
        namespace='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        lidar_tf,
        hesai_node,
        rviz
    ])


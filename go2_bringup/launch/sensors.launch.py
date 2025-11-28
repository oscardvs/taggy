#!/usr/bin/env python3
"""
EDTH Hackathon - Unitree Go2 Sensors Launch File
Launches RealSense D435i and Hesai XT16 LiDAR

Usage:
    # Launch camera only
    ros2 launch go2_bringup sensors.launch.py enable_lidar:=false
    
    # Launch LiDAR only  
    ros2 launch go2_bringup sensors.launch.py enable_camera:=false
    
    # Launch both (default)
    ros2 launch go2_bringup sensors.launch.py
    
    # Launch with RViz visualization
    ros2 launch go2_bringup sensors.launch.py enable_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense D435i camera'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable Hesai XT16 LiDAR'
    )

    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable RealSense depth pointcloud'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable RViz2 visualization (requires display)'
    )

    # ============================================================
    # REALSENSE D435i CAMERA
    # ============================================================
    # 
    # === COLOR STREAM (RGB Camera) ===
    #   /camera/color/image_raw       - RGB image 640x480 @ 30fps (sensor_msgs/Image)
    #   /camera/color/camera_info     - Color camera intrinsics (sensor_msgs/CameraInfo)
    #   /camera/color/metadata        - Frame metadata (realsense2_camera_msgs/Metadata)
    #
    # === DEPTH STREAM (Stereo IR) ===
    #   /camera/depth/image_rect_raw  - Depth image, rectified (sensor_msgs/Image, 16UC1 in mm)
    #   /camera/depth/camera_info     - Depth camera intrinsics (sensor_msgs/CameraInfo)
    #   /camera/depth/metadata        - Frame metadata (realsense2_camera_msgs/Metadata)
    #   /camera/depth/color/points    - Colored 3D pointcloud (sensor_msgs/PointCloud2)
    #
    # === ALIGNED DEPTH (Depth registered to other streams) ===
    #   /camera/aligned_depth_to_color/image_raw   - Depth aligned to color frame (sensor_msgs/Image)
    #   /camera/aligned_depth_to_color/camera_info - Camera info for aligned depth
    #   /camera/aligned_depth_to_infra1/image_raw  - Depth aligned to IR1 frame
    #   /camera/aligned_depth_to_infra1/camera_info
    #
    # === INFRARED STREAMS (IR stereo cameras) ===
    #   /camera/infra1/image_rect_raw - Left IR image, rectified (sensor_msgs/Image)
    #   /camera/infra1/camera_info    - Left IR camera intrinsics
    #   /camera/infra1/metadata       - Frame metadata
    #   /camera/infra2/image_rect_raw - Right IR image, rectified (sensor_msgs/Image)
    #   /camera/infra2/camera_info    - Right IR camera intrinsics
    #   /camera/infra2/metadata       - Frame metadata
    #
    # === IMU STREAMS (Accelerometer + Gyroscope) ===
    #   /camera/imu                   - Fused IMU data (sensor_msgs/Imu)
    #   /camera/accel/sample          - Raw accelerometer data (sensor_msgs/Imu)
    #   /camera/accel/imu_info        - Accelerometer info/intrinsics
    #   /camera/accel/metadata        - Accelerometer metadata
    #   /camera/gyro/sample           - Raw gyroscope data (sensor_msgs/Imu)
    #   /camera/gyro/imu_info         - Gyroscope info/intrinsics
    #   /camera/gyro/metadata         - Gyroscope metadata
    #
    # === EXTRINSICS (Transform calibrations between streams) ===
    #   /camera/extrinsics/depth_to_color  - Transform from depth to color frame
    #   /camera/extrinsics/depth_to_accel  - Transform from depth to accelerometer
    #   /camera/extrinsics/depth_to_gyro   - Transform from depth to gyroscope
    #   /camera/extrinsics/depth_to_infra1 - Transform from depth to left IR
    #   /camera/extrinsics/depth_to_infra2 - Transform from depth to right IR
    #
    # ============================================================
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[{
            'enable_color': True,              # RGB camera stream
            'enable_depth': True,              # Depth from stereo IR
            'enable_infra1': True,             # Left IR camera
            'enable_infra2': True,             # Right IR camera  
            'enable_gyro': True,               # Gyroscope (angular velocity)
            'enable_accel': True,              # Accelerometer (linear acceleration)
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
            'align_depth.enable': True,        # Align depth to color frame
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
        }],
        output='screen',
        emulate_tty=True,
    )

    # ============================================================
    # HESAI XT16 LIDAR
    # ============================================================
    # Official Hesai ROS2 driver for XT16 LiDAR
    #
    # === POINT CLOUD ===
    #   /lidar_points       - 3D point cloud 360° (sensor_msgs/PointCloud2)
    #                         Frame: hesai_lidar
    #                         ~300,000 points per scan at 10Hz
    #
    # === IMU (if available on LiDAR model) ===
    #   /lidar_imu          - LiDAR built-in IMU (sensor_msgs/Imu)
    #
    # === DIAGNOSTICS ===
    #   /lidar_packets_loss - UDP packet loss stats (hesai_ros_driver/LossPacket)
    #
    # Network Configuration:
    #   LiDAR IP: 192.168.1.201 (default)
    #   UDP Port: 2368
    #   Config:   src/HesaiLidar_ROS_2.0/config/config.yaml
    #
    # Troubleshooting:
    #   - Ensure network interface can reach 192.168.1.x subnet
    #   - Check: ping 192.168.1.201
    #   - Check: ros2 topic hz /lidar_points
    #
    # ============================================================
    hesai_lidar_node = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        output='screen',
    )

    # ============================================================
    # RVIZ2 VISUALIZATION (Optional)
    # ============================================================
    # Launch with: enable_rviz:=true
    # Requires a display (X11 forwarding or local display)
    # ============================================================
    rviz_config = os.path.join(
        get_package_share_directory('hesai_ros_driver'),
        'rviz',
        'rviz2.rviz'
    )
    
    rviz_node = Node(
        namespace='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # ============================================================
    # STATIC TRANSFORMS
    # ============================================================
    # Define sensor positions relative to robot base
    # Adjust these based on actual sensor mounting positions on Go2
    
    # Camera TF: mounted on front of robot
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.3', '0.0', '0.3', '0', '0', '0', 'base_link', 'camera_link'],
    )

    # LiDAR TF: XT-16 position in IMU coordinate system (no rotation)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0.1710', '0.0', '0.0908', '0', '0', '0', 'base_link', 'hesai_lidar'],
    )

    return LaunchDescription([
        # Arguments
        enable_camera_arg,
        enable_lidar_arg,
        enable_pointcloud_arg,
        enable_rviz_arg,
        
        # Sensor nodes
        realsense_node,
        hesai_lidar_node,
        
        # Visualization
        rviz_node,
        
        # Static transforms
        # camera_tf,
        lidar_tf,
    ])

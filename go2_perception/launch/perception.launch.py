#!/usr/bin/env python3
"""
EDTH Hackathon - Perception Node Launch File
Launches YOLOv8 TensorRT object detection with depth fusion

Usage:
    # Launch with default parameters
    ros2 launch go2_perception perception.launch.py
    
    # With custom model path
    ros2 launch go2_perception perception.launch.py \
        model_path:=/path/to/yolov8n.onnx
    
    # With custom thresholds
    ros2 launch go2_perception perception.launch.py \
        confidence_threshold:=0.5 \
        target_frame:=odom

Prerequisites:
    - RealSense camera running (ros2 launch go2_bringup sensors.launch.py)
    - YOLOv8 ONNX model downloaded to model_path
    - TensorRT engine will be auto-generated on first run if not found
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    go2_perception_dir = get_package_share_directory('go2_perception')

    # Config file path
    perception_params_file = os.path.join(
        go2_perception_dir, 'config', 'perception_params.yaml')

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
        default_value='/opt/yolo/yolov8n.onnx',
        description='Path to YOLOv8 ONNX model file'
    )

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value='/opt/yolo/yolov8n.engine',
        description='Path to TensorRT engine file (auto-generated if not found)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.6',
        description='Minimum confidence for detections [0.0-1.0]'
    )

    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS IoU threshold [0.0-1.0]'
    )

    use_fp16_arg = DeclareLaunchArgument(
        'use_fp16',
        default_value='true',
        description='Enable FP16 inference (recommended for Jetson)'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame for depth points'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target frame for 3D pose output'
    )

    # ============================================================
    # PERCEPTION NODE
    # ============================================================
    # YOLOv8 object detection with TensorRT optimization
    # Fuses RGB detections with aligned depth for 3D localization
    #
    # Subscriptions:
    #   /camera/color/image_raw - RGB camera stream
    #   /camera/aligned_depth_to_color/image_raw - Aligned depth
    #   /camera/color/camera_info - Camera intrinsics
    #   /mission/target_object - Target object to track
    #
    # Publications:
    #   /detection/bbox - All detections (vision_msgs/Detection2DArray)
    #   /detection/target_pose - Target 3D pose (geometry_msgs/PoseStamped)
    # ============================================================
    perception_node = Node(
        package='go2_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            perception_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': LaunchConfiguration('model_path'),
                'engine_path': LaunchConfiguration('engine_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'use_fp16': LaunchConfiguration('use_fp16'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
            }
        ],
        # Remap topics if needed (uncomment to customize)
        # remappings=[
        #     ('/camera/color/image_raw', '/custom/rgb'),
        #     ('/camera/aligned_depth_to_color/image_raw', '/custom/depth'),
        # ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        model_path_arg,
        engine_path_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        use_fp16_arg,
        camera_frame_arg,
        target_frame_arg,

        # Perception Node
        perception_node,
    ])


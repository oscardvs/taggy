# Perception Architecture

YOLOv8 TensorRT object detection with depth fusion for 3D localization on Unitree Go2.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              PERCEPTION PIPELINE                                         │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│   ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐  │
│   │  RealSense      │    │  TensorRT       │    │  Depth          │    │  TF2        │  │
│   │  D435i          │───▶│  YOLOv8         │───▶│  Fusion         │───▶│  Transform  │  │
│   │  Camera         │    │  Inference      │    │  3D Point       │    │  to Map     │  │
│   └─────────────────┘    └─────────────────┘    └─────────────────┘    └─────────────┘  │
│          │                      │                      │                      │          │
│          │                      │                      │                      │          │
│          ▼                      ▼                      ▼                      ▼          │
│   ┌─────────────────────────────────────────────────────────────────────────────────┐   │
│   │                           perception_node                                        │   │
│   │                                                                                  │   │
│   │   Subscriptions:                         Publications:                           │   │
│   │     • /camera/color/image_raw              • /detection/bbox                     │   │
│   │     • /camera/aligned_depth_to_color/...   • /detection/target_pose              │   │
│   │     • /camera/color/camera_info                                                  │   │
│   │     • /mission/target_object                                                     │   │
│   │                                                                                  │   │
│   │   TF Lookup: camera_color_optical_frame → map                                    │   │
│   └─────────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
                    ┌───────────────────────────────────────────────────────────────┐
                    │                     RealSense D435i                            │
                    │                                                                │
                    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐ │
                    │  │  RGB Camera  │  │ Depth Sensor │  │ Camera Intrinsics    │ │
                    │  │  640x480     │  │  640x480     │  │ fx, fy, cx, cy       │ │
                    │  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘ │
                    └─────────┼─────────────────┼─────────────────────┼─────────────┘
                              │                 │                     │
                              ▼                 ▼                     ▼
                    /camera/color/    /camera/aligned_     /camera/color/
                     image_raw        depth_to_color/       camera_info
                              │        image_raw            (one-time)
                              │                 │                     │
                              └────────┬────────┘                     │
                                       │                              │
                    ┌──────────────────▼──────────────────────────────▼──────────────┐
                    │              ApproximateTimeSynchronizer                        │
                    │                  (message_filters)                              │
                    └──────────────────────────┬──────────────────────────────────────┘
                                               │
                                               ▼
┌──────────────────┐              ┌─────────────────────────┐
│ /mission/        │              │                         │
│  target_object   │─────────────▶│     PREPROCESSING       │
│ (from voice)     │              │                         │
└──────────────────┘              │  • Resize to 640x640    │
                                  │  • BGR → RGB            │
                                  │  • Normalize [0,1]      │
                                  │  • HWC → CHW            │
                                  └───────────┬─────────────┘
                                              │
                                              ▼
                                  ┌─────────────────────────┐
                                  │                         │
                                  │   TENSORRT INFERENCE    │
                                  │                         │
                                  │  • YOLOv8n/s engine     │
                                  │  • FP16 precision       │
                                  │  • ~30 FPS on Orin      │
                                  └───────────┬─────────────┘
                                              │
                                              ▼
                                  ┌─────────────────────────┐
                                  │                         │
                                  │    POST-PROCESSING      │
                                  │                         │
                                  │  • Parse 8400 anchors   │
                                  │  • Filter conf > 0.6    │
                                  │  • NMS (IoU 0.45)       │
                                  │  • Scale to original    │
                                  └───────────┬─────────────┘
                                              │
                                              ▼
                                  ┌─────────────────────────┐
                                  │                         │
                                  │   TARGET MATCHING       │
                                  │                         │
                                  │  • Match class_name     │
                                  │    to target_object     │
                                  │  • Select highest conf  │
                                  └───────────┬─────────────┘
                                              │
                         ┌────────────────────┴────────────────────┐
                         │                                         │
                         ▼                                         ▼
              ┌─────────────────────┐                  ┌─────────────────────┐
              │  All Detections     │                  │  Target Detection   │
              │                     │                  │  (if found)         │
              └──────────┬──────────┘                  └──────────┬──────────┘
                         │                                        │
                         ▼                                        ▼
              ┌─────────────────────┐                  ┌─────────────────────┐
              │                     │                  │                     │
              │  /detection/bbox    │                  │   DEPTH FUSION      │
              │  Detection2DArray   │                  │                     │
              │                     │                  │  • Centroid (u,v)   │
              └─────────────────────┘                  │  • 5x5 median depth │
                         │                            │  • Filter invalid   │
                         │                            └──────────┬──────────┘
                         │                                       │
                         │                                       ▼
                         │                            ┌─────────────────────┐
                         │                            │                     │
                         │                            │   DEPROJECTION      │
                         │                            │                     │
                         │                            │  X = (u-cx)*Z/fx    │
                         │                            │  Y = (v-cy)*Z/fy    │
                         │                            │  Z = depth          │
                         │                            └──────────┬──────────┘
                         │                                       │
                         │                                       ▼
                         │                            ┌─────────────────────┐
                         │                            │                     │
                         │                            │   TF2 TRANSFORM     │
                         │                            │                     │
                         │                            │  camera_optical     │
                         │                            │       ↓             │
                         │                            │     map             │
                         │                            └──────────┬──────────┘
                         │                                       │
                         ▼                                       ▼
              ┌─────────────────────┐                  ┌─────────────────────┐
              │  mission_executive  │                  │ /detection/         │
              │  (state transitions)│                  │  target_pose        │
              │                     │◀─────────────────│  PoseStamped        │
              └─────────────────────┘                  └──────────┬──────────┘
                                                                  │
                                                                  ▼
                                                       ┌─────────────────────┐
                                                       │  visual_servo_node  │
                                                       │  (IBVS controller)  │
                                                       │  (depth for v_x)    │
                                                       └─────────────────────┘
```

## TensorRT Engine Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           TensorRT Engine Initialization                             │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   ┌─────────────┐     ┌──────────────────────────────────────────────────────────┐  │
│   │ Engine File │     │                                                          │  │
│   │ Exists?     │─YES─▶│  Deserialize .engine file (~1-2 seconds)               │  │
│   └──────┬──────┘     │                                                          │  │
│          │            └──────────────────────────────────────────────────────────┘  │
│          NO                                                                         │
│          │                                                                          │
│          ▼                                                                          │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                        Build Engine from ONNX                                │   │
│   │                                                                              │   │
│   │   1. Parse ONNX model (yolov8n.onnx)                                        │   │
│   │   2. Create network definition                                               │   │
│   │   3. Configure builder:                                                      │   │
│   │      • Set workspace memory (1GB)                                           │   │
│   │      • Enable FP16 precision (Tensor Cores)                                 │   │
│   │   4. Build optimized engine (5-10 minutes on Jetson)                        │   │
│   │   5. Serialize and save to .engine file                                     │   │
│   │                                                                              │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                         Allocate GPU Buffers                                 │   │
│   │                                                                              │   │
│   │   Input:  [1, 3, 640, 640] float32  →  ~4.9 MB                              │   │
│   │   Output: [1, 84, 8400]    float32  →  ~2.8 MB                              │   │
│   │                                                                              │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## YOLOv8 Output Format

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           YOLOv8 Output Tensor [1, 84, 8400]                         │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   For each of 8400 anchor predictions:                                              │
│                                                                                      │
│   ┌────────────────────────────────────────────────────────────────────────────┐    │
│   │  Index 0:  cx (center x)           ─┐                                      │    │
│   │  Index 1:  cy (center y)            ├── Bounding Box                       │    │
│   │  Index 2:  w  (width)               │   (in model input coords 0-640)      │    │
│   │  Index 3:  h  (height)             ─┘                                      │    │
│   │  Index 4:  class_0 score (person)  ─┐                                      │    │
│   │  Index 5:  class_1 score (bicycle)  │                                      │    │
│   │  Index 6:  class_2 score (car)      ├── 80 COCO Class Scores               │    │
│   │  ...                                │   (confidence values 0-1)            │    │
│   │  Index 83: class_79 score          ─┘                                      │    │
│   └────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                      │
│   Anchors from 3 detection heads:                                                   │
│     • 80x80 grid (small objects)  → 6400 anchors                                    │
│     • 40x40 grid (medium objects) → 1600 anchors                                    │
│     • 20x20 grid (large objects)  →  400 anchors                                    │
│     • Total: 8400 anchors                                                           │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Depth Fusion Mathematics

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           3D Localization from 2D Detection                          │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   Given:                                                                            │
│     • Detection bounding box center: (u_c, v_c) in pixels                           │
│     • Camera intrinsics: fx, fy (focal lengths), cx, cy (principal point)           │
│     • Aligned depth image: D(u, v) in millimeters                                   │
│                                                                                      │
│   Step 1: Median-Filtered Depth Sampling                                            │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                                                                             │   │
│   │   Sample 5x5 kernel around (u_c, v_c)                                       │   │
│   │   Filter out zero/invalid depth values                                      │   │
│   │   Z_cam = median(valid_depths) / 1000.0  [convert mm → meters]              │   │
│   │                                                                             │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
│   Step 2: Deprojection (Pinhole Camera Model)                                       │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                                                                             │   │
│   │              (u_c - cx) · Z_cam                                             │   │
│   │   X_cam  =  ─────────────────────                                           │   │
│   │                    fx                                                       │   │
│   │                                                                             │   │
│   │              (v_c - cy) · Z_cam                                             │   │
│   │   Y_cam  =  ─────────────────────                                           │   │
│   │                    fy                                                       │   │
│   │                                                                             │   │
│   │   Z_cam  =  depth (forward distance)                                        │   │
│   │                                                                             │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
│   Step 3: Coordinate Frame Conversion                                               │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                                                                             │   │
│   │   Camera Optical Frame:          Robot/Map Frame:                           │   │
│   │     X → right                      X → forward                              │   │
│   │     Y → down                       Y → left                                 │   │
│   │     Z → forward                    Z → up                                   │   │
│   │                                                                             │   │
│   │   Conversion:                                                               │   │
│   │     X_robot =  Z_cam    (forward)                                           │   │
│   │     Y_robot = -X_cam    (left)                                              │   │
│   │     Z_robot = -Y_cam    (up)                                                │   │
│   │                                                                             │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
│   Step 4: TF2 Transform to Map Frame                                                │
│   ┌─────────────────────────────────────────────────────────────────────────────┐   │
│   │                                                                             │   │
│   │   camera_color_optical_frame → base_link → odom → map                       │   │
│   │                                                                             │   │
│   │   Uses tf2_ros::Buffer::transform() for automatic chain lookup              │   │
│   │                                                                             │   │
│   └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Integration with Mission System

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              Mission System Integration                              │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   ┌──────────────┐        ┌──────────────┐        ┌──────────────┐                  │
│   │    IDLE      │───────▶│  LISTENING   │───────▶│  SEARCHING   │                  │
│   └──────────────┘  L2+A  └──────────────┘ voice  └──────┬───────┘                  │
│                                   │                      │                          │
│                                   │                      │ target detected          │
│                                   ▼                      ▼                          │
│                           ┌──────────────┐        ┌──────────────┐                  │
│                           │voice_command │        │  perception  │                  │
│                           │    _node     │        │    _node     │                  │
│                           └──────┬───────┘        └──────┬───────┘                  │
│                                  │                       │                          │
│                                  ▼                       ▼                          │
│                     /mission/target_object      /detection/bbox                     │
│                     (e.g., "bottle")            /detection/target_pose              │
│                                  │                       │                          │
│                                  │                       │                          │
│                                  └───────────┬───────────┘                          │
│                                              │                                      │
│                           ┌──────────────────┼──────────────────┐                   │
│                           │                  │                  │                   │
│                           ▼                  ▼                  ▼                   │
│               ┌──────────────────┐  ┌──────────────────┐  ┌────────────────┐        │
│               │ mission_executive│  │ visual_servo_node│  │    /scan       │        │
│               │                  │  │                  │  │   (LiDAR)      │        │
│               │ • State machine  │  │ • IBVS control   │  └───────┬────────┘        │
│               │ • Orchestration  │  │ • LiDAR safety   │          │                 │
│               │ • State → /state │  │ • /servo_cmd_vel │◀─────────┘                 │
│               └────────┬─────────┘  └────────┬─────────┘                            │
│                        │                     │                                      │
│                        │                     │                                      │
│                        ▼                     ▼                                      │
│               ┌──────────────────────────────────────────┐                          │
│               │               TRACKING State             │                          │
│               │                                          │                          │
│               │  mission_executive: state & transitions  │                          │
│               │  visual_servo_node: velocity commands    │                          │
│               │    - Yaw:   ω_z = K_p * image_error      │                          │
│               │    - Fwd:   v_x = K_p * depth_error      │                          │
│               │    - Safety: LiDAR reflex inhibits v_x   │                          │
│               └──────────────────────────────────────────┘                          │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Topic Reference

### Subscriptions

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense | RGB image 640x480 @ 30fps |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense | Depth aligned to RGB (16UC1 mm) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense | Intrinsic matrix K |
| `/mission/target_object` | `std_msgs/String` | voice_command_node | Target class name |

### Publications

| Topic | Type | Subscribers | Description |
|-------|------|-------------|-------------|
| `/detection/bbox` | `vision_msgs/Detection2DArray` | mission_executive, visual_servo_node | All detections with class + confidence |
| `/detection/target_pose` | `geometry_msgs/PoseStamped` | mission_executive, visual_servo_node | 3D pose of target in map frame |

## Performance Characteristics

| Metric | YOLOv8n (Nano) | YOLOv8s (Small) |
|--------|----------------|-----------------|
| Inference FPS | ~30-40 | ~15-25 |
| mAP@0.5 | 37.3% | 44.9% |
| Model Size | 6.3 MB | 22.5 MB |
| Engine Build Time | ~5 min | ~8 min |
| GPU Memory | ~200 MB | ~400 MB |

Recommended: **YOLOv8n** for real-time robotics on Jetson Orin.


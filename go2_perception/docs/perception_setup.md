# Perception Setup Guide

Setup instructions for YOLOv8 TensorRT perception on laptop (development) and Jetson Orin (robot).

## Prerequisites

### Common Requirements

- ROS2 Foxy installed
- OpenCV 4.x
- Python 3.8+

### Robot (Jetson Orin) Additional Requirements

- JetPack 5.x (includes CUDA, cuDNN, TensorRT)
- RealSense D435i camera

### Laptop (Development) Additional Requirements

- NVIDIA GPU with CUDA support (optional, for TensorRT)
- Or use CPU-only mode for testing

---

## 1. Robot Setup (Jetson Orin)

### 1.1 Install ROS2 Dependencies

```bash
# Vision and image processing
sudo apt update && sudo apt install -y \
    ros-foxy-vision-msgs \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-message-filters \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs

# RealSense camera driver
sudo apt install -y ros-foxy-realsense2-camera
```

### 1.2 Verify TensorRT Installation

JetPack should have TensorRT pre-installed:

```bash
# Check TensorRT version
dpkg -l | grep tensorrt

# Verify libraries exist
ls /usr/lib/aarch64-linux-gnu/libnvinfer.so*
ls /usr/lib/aarch64-linux-gnu/libnvonnxparser.so*

# Check CUDA
nvcc --version
```

Expected output: TensorRT 8.x, CUDA 11.x

### 1.3 Download YOLOv8 Model

```bash
# Create model directory
sudo mkdir -p /opt/yolo
sudo chown $USER:$USER /opt/yolo

# Download YOLOv8 Nano ONNX model
cd /opt/yolo
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx

# Verify download
ls -la /opt/yolo/
# Should show: yolov8n.onnx (~12MB)
```

### 1.4 Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash

# Build perception package
colcon build --packages-select go2_perception

# Source workspace
source install/setup.bash
```

### 1.5 First Run - Engine Build

The first launch will build the TensorRT engine (5-10 minutes):

```bash
# Launch sensors first
ros2 launch go2_bringup sensors.launch.py

# In another terminal, launch perception
ros2 launch go2_perception perception.launch.py
```

Watch for these log messages:
```
[perception_node]: Loading engine from ONNX: /opt/yolo/yolov8n.onnx
[TensorRT] Building engine... (this may take several minutes)
[TensorRT] FP16 mode enabled
[TensorRT] Engine saved to: /opt/yolo/yolov8n.engine
[perception_node]: Engine initialized successfully
```

Subsequent launches will load the cached engine in ~2 seconds.

### 1.6 Verify Perception is Working

```bash
# Check topics are publishing
ros2 topic list | grep detection
# Should show:
#   /detection/bbox
#   /detection/target_pose

# Monitor detections
ros2 topic echo /detection/bbox

# Check frequency
ros2 topic hz /detection/bbox
# Should be ~10-30 Hz
```

---

## 2. Laptop Setup (Development/Testing)

### 2.1 Option A: With NVIDIA GPU (Recommended)

If your laptop has an NVIDIA GPU:

```bash
# Install CUDA toolkit
sudo apt install -y nvidia-cuda-toolkit

# Install TensorRT (Ubuntu 20.04)
# Add NVIDIA repo first, then:
sudo apt install -y libnvinfer8 libnvinfer-dev libnvonnxparsers8 libnvonnxparsers-dev
```

Then follow the same steps as robot setup.

### 2.2 Option B: Without GPU (Mock Testing)

For development without TensorRT:

```bash
# Install ROS2 dependencies only
sudo apt install -y \
    ros-foxy-vision-msgs \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport

# Build will fail on TensorRT - this is expected
# Use mock publisher for testing instead
```

**Mock Detection Publisher** (for testing mission executive):

```bash
# Publish fake detection for testing
ros2 topic pub /detection/bbox vision_msgs/msg/Detection2DArray "{
  header: {frame_id: 'camera_color_optical_frame'},
  detections: [{
    bbox: {center: {position: {x: 320.0, y: 240.0}}, size_x: 100.0, size_y: 150.0},
    results: [{hypothesis: {class_id: 'bottle', score: 0.85}}]
  }]
}" -r 10
```

### 2.3 Test with ROS Bag

Record a bag on the robot, play back on laptop:

```bash
# On robot - record camera data
ros2 bag record /camera/color/image_raw \
    /camera/aligned_depth_to_color/image_raw \
    /camera/color/camera_info \
    -o perception_test

# Transfer to laptop
scp -r unitree@192.168.123.18:~/perception_test .

# On laptop - playback
ros2 bag play perception_test --loop
```

---

## 3. Configuration

### 3.1 Model Selection

Edit `/opt/yolo/` or launch argument:

| Model | Use Case | Command |
|-------|----------|---------|
| YOLOv8n | Real-time (recommended) | `model_path:=/opt/yolo/yolov8n.onnx` |
| YOLOv8s | Higher accuracy | `model_path:=/opt/yolo/yolov8s.onnx` |

Download alternative models:
```bash
# YOLOv8 Small (more accurate, slower)
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.onnx -P /opt/yolo/
```

### 3.2 Threshold Tuning

```bash
# Higher confidence = fewer false positives
ros2 launch go2_perception perception.launch.py confidence_threshold:=0.7

# Lower confidence = detect more objects (may include false positives)
ros2 launch go2_perception perception.launch.py confidence_threshold:=0.4
```

### 3.3 Frame Configuration

```bash
# Use odom frame instead of map (before SLAM is running)
ros2 launch go2_perception perception.launch.py target_frame:=odom

# Custom camera frame
ros2 launch go2_perception perception.launch.py \
    camera_frame:=camera_depth_optical_frame
```

---

## 4. Troubleshooting

### TensorRT Build Fails

```
Error: Failed to parse ONNX file
```

**Solution**: Ensure ONNX file is not corrupted:
```bash
# Re-download
rm /opt/yolo/yolov8n.onnx
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx -P /opt/yolo/
```

### Engine Not Compatible

```
Error: Serialized engine version mismatch
```

**Solution**: Delete old engine and rebuild:
```bash
rm /opt/yolo/yolov8n.engine
# Restart perception node - will rebuild
```

### No Detections Published

Check camera topics:
```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

If no data, restart camera:
```bash
ros2 launch go2_bringup sensors.launch.py
```

### TF Transform Errors

```
Warning: TF transform failed: "map" passed to lookupTransform does not exist
```

**Solution**: Ensure SLAM is running:
```bash
ros2 launch go2_bringup slam.launch.py
```

Or use odom frame:
```bash
ros2 launch go2_perception perception.launch.py target_frame:=odom
```

### Low FPS

Check GPU utilization:
```bash
tegrastats  # On Jetson
nvidia-smi  # On laptop
```

Reduce model size:
```bash
# Use Nano instead of Small
ros2 launch go2_perception perception.launch.py model_path:=/opt/yolo/yolov8n.onnx
```

---

## 5. Quick Reference

### Launch Commands

```bash
# Perception only (requires camera running)
ros2 launch go2_perception perception.launch.py

# Full mission stack (includes perception)
ros2 launch go2_bringup mission_bringup.launch.py

# With custom settings
ros2 launch go2_perception perception.launch.py \
    model_path:=/opt/yolo/yolov8n.onnx \
    confidence_threshold:=0.6 \
    target_frame:=map
```

### Useful Commands

```bash
# Monitor detections
ros2 topic echo /detection/bbox

# Monitor target pose
ros2 topic echo /detection/target_pose

# Check perception rate
ros2 topic hz /detection/bbox

# Set target manually (for testing)
ros2 topic pub /mission/target_object std_msgs/String "data: 'bottle'" -1

# List detected classes in last message
ros2 topic echo /detection/bbox --field detections
```

### File Locations

| File | Path |
|------|------|
| ONNX Model | `/opt/yolo/yolov8n.onnx` |
| TensorRT Engine | `/opt/yolo/yolov8n.engine` |
| Config | `go2_perception/config/perception_params.yaml` |
| Launch | `go2_perception/launch/perception.launch.py` |


# Taggy Setup Guide

## 1. Connect to Robot

```bash
# SSH into Go2 (from laptop connected to Go2 WiFi)
ssh unitree@192.168.123.18
# Password: 123
```

## 2. Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/oscardvs/taggy .
```

## 3. System Dependencies

```bash
# ROS2 Foxy (if not installed)
sudo apt update && sudo apt install -y ros-foxy-desktop

# Nav2 & SLAM
sudo apt install -y \
    ros-foxy-nav2-bringup \
    ros-foxy-nav2-bt-navigator \
    ros-foxy-nav2-map-server \
    ros-foxy-slam-toolbox \
    ros-foxy-twist-mux \
    ros-foxy-pointcloud-to-laserscan \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    ros-foxy-tf2-ros \
    ros-foxy-vision-msgs

# Perception (image processing)
sudo apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-message-filters

# explore_lite (may need from source if not available)
sudo apt install -y ros-foxy-explore-lite || echo "Install explore_lite from source"

# RealSense (if using D435i)
sudo apt install -y ros-foxy-realsense2-camera

# Audio (for voice commands - optional on robot)
sudo apt install -y portaudio19-dev
```

## 4. Python Dependencies

```bash
pip3 install vosk numpy pyaudio
```

## 5. Vosk Model

```bash
mkdir -p ~/vosk-models && cd ~/vosk-models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip
```

## 6. YOLOv8 Model (Perception)

```bash
# Create model directory
sudo mkdir -p /opt/yolo
sudo chown $USER:$USER /opt/yolo

# Download YOLOv8 Nano model
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx -P /opt/yolo/

# TensorRT engine will be auto-built on first run (~5-10 min)
```

> **Note**: TensorRT is pre-installed on Jetson with JetPack. Verify with `dpkg -l | grep tensorrt`

## 7. Build Workspace

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 8. Launch (Step by Step)

**Terminal 1: Robot Control**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup go2_control.launch.py
```

**Terminal 2: Sensors (LiDAR + Camera)**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup sensors.launch.py
```

**Terminal 3: SLAM**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup slam.launch.py
```

**Terminal 4: Navigation**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup navigation.launch.py
```

**Terminal 5: Exploration**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup explore.launch.py
```

**Terminal 6: Perception (YOLOv8)**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_perception perception.launch.py
# First run builds TensorRT engine (~5-10 min)
```

**Terminal 7: Mission Executive**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_mission mission.launch.py
```

## 9. Alternative: All-in-One Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup mission_bringup.launch.py
```

## 10. Trigger Mission

1. Hold **L2 + A** on controller → Robot enters LISTENING mode
2. Say target object (e.g., "bottle", "backpack", "chair")
3. Robot searches and navigates to target

## Quick Checks

```bash
# Check topics
ros2 topic list

# Check controller
ros2 topic echo /wirelesscontroller

# Check mission state
ros2 topic echo /mission/state

# Check detections
ros2 topic echo /detection/bbox

# Check perception FPS
ros2 topic hz /detection/bbox

# Check visual servo velocity commands
ros2 topic echo /servo_cmd_vel

# Check LiDAR (used by visual servo safety)
ros2 topic echo /scan --once

# Manual LISTENING trigger (testing)
ros2 topic pub /mission/state std_msgs/String "data: 'LISTENING'" -1

# Manual target object (testing)
ros2 topic pub /mission/target_object std_msgs/String "data: 'bottle'" -1

# Manual TRACKING trigger (testing visual servo)
ros2 topic pub /mission/state std_msgs/String "data: 'TRACKING'" -1
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Build fails | `rosdep install --from-paths src --ignore-src -r -y` |
| No LiDAR data | Check `/scan` topic, verify LiDAR power |
| No camera data | Check `/camera/color/image_raw`, restart RealSense |
| No controller | Check `/wirelesscontroller`, ensure controller on |
| Vosk model error | Verify `~/vosk-models/vosk-model-small-en-us-0.15` exists |
| Nav2 not ready | Wait 10-15s for costmaps to initialize |
| No detections | Check camera topics, verify YOLO model at `/opt/yolo/` |
| TensorRT build slow | Normal on first run (5-10 min), engine cached after |
| Engine version error | Delete `/opt/yolo/*.engine` and restart perception |
| TF transform error | Ensure SLAM running, or use `target_frame:=odom` |

## Perception Details

See `go2_perception/docs/` for detailed documentation:
- `perception_architecture.md` - System design and data flow
- `perception_setup.md` - Detailed setup for laptop and robot


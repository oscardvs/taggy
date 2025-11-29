# Taggy Setup Guide

> **ROS2 Foxy** | Ubuntu 20.04 | Python 3.8 | **ARM64 (Jetson) supported**

---

## Platform Notes

| Platform | Architecture | Notes |
|----------|-------------|-------|
| **Go2 Robot** | ARM64 (Jetson) | Uses JetPack's PyTorch/OpenCV/CUDA |
| **Laptop/Desktop** | x86_64 | Standard pip installation |

The installer automatically detects architecture and installs appropriate packages.

> **⚠️ Jetson Users**: Do NOT pip install `torch` or `opencv-python` - use the pre-installed JetPack versions!

---

## Quick Install (Recommended)

```bash
# 1. SSH into Go2 (from laptop connected to Go2 WiFi)
ssh unitree@192.168.123.18
# Password: 123

# 2. Clone repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/oscardvs/taggy .

# 3. Run automated installer
cd scripts
./install.sh
```

The installer will:
- Install all ROS2 Foxy dependencies
- Create Python virtual environment at `~/taggy_venv`
- Download ML models (YOLO11, Vosk)
- Build the workspace

After installation, activate the environment:

```bash
source /opt/ros/foxy/setup.bash
source ~/taggy_venv/bin/activate
source ~/ros2_ws/install/setup.bash
```

---

## Manual Installation (Alternative)

If you prefer manual installation or the automated script fails:

### 1. Connect to Robot

```bash
# SSH into Go2 (from laptop connected to Go2 WiFi)
ssh unitree@192.168.123.18
# Password: 123
```

### 2. Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/oscardvs/taggy .
```

### 3. System Dependencies

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

### 4. Python Virtual Environment & Dependencies

```bash
# Create virtual environment (--system-site-packages needed for Jetson)
python3 -m venv ~/taggy_venv --system-site-packages
source ~/taggy_venv/bin/activate
pip install --upgrade pip setuptools wheel
```

**For Jetson (Go2 Robot):**
```bash
# Use JetPack's PyTorch/OpenCV - do NOT pip install them!
pip install numpy PyYAML
pip install ultralytics --no-deps  # Skip torch dependency
pip install vosk
pip install matplotlib pandas seaborn tqdm pillow requests psutil py-cpuinfo
```

**For Laptop (x86_64):**
```bash
pip install -r ~/ros2_ws/src/requirements.txt
pip install opencv-python opencv-contrib-python
```

### 5. Vosk Model

```bash
mkdir -p ~/vosk-models && cd ~/vosk-models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip
```

### 6. YOLO11 Model (Perception & Tracking)

```bash
# Create model directory
sudo mkdir -p /opt/yolo
sudo chown $USER:$USER /opt/yolo

# Download YOLO11n via ultralytics (recommended - ensures correct weights)
python3 -c "
from ultralytics import YOLO
model = YOLO('yolo11n.pt')  # Downloads automatically
model.export(format='onnx', imgsz=640)  # Export to ONNX for TensorRT
"

# Move models to correct locations
mv yolo11n.pt ~/ros2_ws/src/
mv yolo11n.onnx /opt/yolo/

# TensorRT engine will be auto-built from ONNX on first run (~5-10 min)
```

> **Note**: TensorRT is pre-installed on Jetson with JetPack. Verify with `dpkg -l | grep tensorrt`

### 7. Build Workspace

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

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

**Terminal 6: Perception (TensorRT)**
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

## 10. Trigger Mission (Perception Pipeline)

1. Hold **L2 + A** on controller → Robot enters LISTENING mode
2. Say target object (e.g., "bottle", "backpack", "chair")
3. Robot searches and navigates to target

---

## Pipeline Comparison

| Feature | Perception Pipeline | Tracker Pipeline |
|---------|--------------------|--------------------|
| Detection | go2_perception (TensorRT) | go2_tracker (YOLO11) |
| Control | Visual servo (reactive) | Nav2 (path-planned) |
| Target Selection | Voice command | YAML config |
| State Machine | Yes (mission_executive) | No (always tracking) |
| Use Case | Voice-triggered search | Pre-configured follow |
| Obstacle Avoidance | LiDAR reflex only | Full Nav2 costmaps |

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

---

## Alternative: Tracker + Follower Pipeline (Nav2-based following)

This pipeline uses go2_tracker for detection and go2_follower for Nav2 path-planned following.
No state machine required - simpler standalone operation.

### Configure Tracker Target

Edit `go2_tracker/config/tracker_config.yaml`:
```yaml
tracker:
  ros__parameters:
    # Camera topics (verify these match your sensor setup)
    rgb_topic: "/camera/color/image_raw"
    depth_topic: "/camera/aligned_depth_to_color/image_raw"
    camera_info_topic: "/camera/color/camera_info"
    
    # Target to track (COCO class name)
    threat_id: "person"
    confidence_threshold: 0.5
```

> **Note**: Verify camera topics match your RealSense output with `ros2 topic list | grep camera`

### Launch Tracker Pipeline

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

**Terminal 3: SLAM + Navigation (disable exploration)**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_bringup full_navigation.launch.py enable_exploration:=false
```

**Terminal 4: Tracker**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_tracker tracker.launch.py
```

**Terminal 5: Follower**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_follower target_follower.launch.py perception_source:=tracker
```

### Follower Parameters

Override at launch or edit `go2_follower/config/follower_params.yaml`:
```bash
# Custom follow distance (default: 1.0m)
ros2 launch go2_follower target_follower.launch.py \
    perception_source:=tracker \
    follow_distance:=1.5

# Parameters:
#   follow_distance   - Distance to maintain from target [m]
#   replan_threshold  - Min target movement to trigger replan [m]
#   replan_rate       - Max replan frequency [Hz]
```

### Quick Checks (Tracker Pipeline)

```bash
# Check tracker status
ros2 topic echo /tracked_object/status

# Check tracked pose (camera frame)
ros2 topic echo /tracked_object/pose

# Check follower status
ros2 topic echo /follower/status

# Check follower goal (map frame)
ros2 topic echo /follower/goal

# View tracker debug image
ros2 run rqt_image_view rqt_image_view /tracked_object/debug_image
```

---

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
| Tracker no pose | Check `/tracked_object/status`, verify target in view |
| Follower not moving | Check Nav2 running, verify `/follower/goal` publishing |
| Follower TF error | Ensure SLAM provides `map` frame, check TF tree |
| Unknown threat_id | Use valid COCO class name (person, dog, bottle, etc.) |

## Perception Details

See `go2_perception/docs/` for detailed documentation:
- `perception_architecture.md` - System design and data flow
- `perception_setup.md` - Detailed setup for laptop and robot


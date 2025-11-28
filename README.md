# EDTH Hackathon - Unitree Go2 ROS2 Starter Pack

**Organized by Laelaps AI**

ROS2 Foxy starter pack for the EDTH Defense Tech Hackathon.

---


## 📦 Package Overview

```
src/
├── go2_bringup/        # Launch files and sensor configs
├── go2_control/        # C++ velocity control node (SportClient API)
├── go2_interfaces/     # Python utilities and message types
├── go2_examples/       # Keyboard teleop for testing
├── unitree_api/        # Unitree ROS2 message definitions
└── unitree_go/         # Unitree Go2 message definitions
```

---

## 🚀 Quick Start

### 1. Build the Workspace

```bash
cd ~/robot_ws/EDTH-ros2-starterpack
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Robot Control

```bash
# Start the velocity control node
ros2 launch go2_bringup go2_control.launch.py
```

### 3. Launch Sensors

```bash
# Launch both camera and LiDAR
ros2 launch go2_bringup sensors.launch.py

# Launch camera only
ros2 launch go2_bringup sensors.launch.py enable_lidar:=false

# Launch LiDAR only
ros2 launch go2_bringup sensors.launch.py enable_camera:=false

# Launch with RViz visualization (requires display)
ros2 launch go2_bringup sensors.launch.py enable_rviz:=true
```

### 4. Test with Keyboard Control

```bash
ros2 run go2_examples keyboard_teleop
```

**Keyboard Controls:**

| Key | Action |
|-----|--------|
| `W/S` | Forward / Backward |
| `A/D` | Strafe Left / Right |
| `Q/E` | Rotate Left / Right |
| `1` | Stand Up |
| `2` | Sit Down |
| `3` | Balance Stand |
| `4` | Recovery Stand (get up from fallen) |
| `5` | Hello (wave) |
| `+/-` | Increase / Decrease speed |
| `SPACE` | Stop movement |
| `X` | Emergency Stop (disables motors!) |
| `R` | Release emergency stop |
| `ESC` | Quit |

---

## 🎮 Robot Control

### Velocity Commands

Send velocity commands to `/cmd_vel`:

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5   # Forward (m/s), positive = forward
cmd.linear.y = 0.0   # Strafe (m/s), positive = left
cmd.angular.z = 0.3  # Rotation (rad/s), positive = counter-clockwise

publisher.publish(cmd)
```

**Velocity Limits:**
- Linear velocity: ±1.0 m/s (recommended: start with 0.3 m/s)
- Angular velocity: ±1.0 rad/s

### Posture Commands

Send posture commands to `/cmd_posture`:

```bash
# Stand up
ros2 topic pub /cmd_posture std_msgs/String "data: 'up'" --once

# Sit down
ros2 topic pub /cmd_posture std_msgs/String "data: 'down'" --once

# Balance stand (active balancing)
ros2 topic pub /cmd_posture std_msgs/String "data: 'balance'" --once

# Recovery stand (get up from fallen)
ros2 topic pub /cmd_posture std_msgs/String "data: 'recovery'" --once

# Say hello (wave gesture)
ros2 topic pub /cmd_posture std_msgs/String "data: 'hello'" --once
```

**Available posture commands:** `up`, `down`, `balance`, `recovery`, `sit`, `hello`, `stretch`, `stop`

### Emergency Stop

```bash
# Activate emergency stop (disables motors!)
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: false" --once
```

---

## 📡 ROS2 Topics

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | **Velocity commands (PUBLISH HERE)** |
| `/cmd_posture` | `std_msgs/String` | Posture commands (up/down/balance/recovery) |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop trigger |
| `/api/sport/request` | `unitree_api/Request` | Raw Unitree API commands |

### Camera (RealSense D435i)

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB image (640x480 @ 30fps) |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Aligned depth image |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Colored point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

### LiDAR (Hesai XT16)

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | 3D point cloud (360°, ~300k points @ 10Hz) |
| `/lidar_imu` | `sensor_msgs/Imu` | LiDAR built-in IMU |

### TF Frames

```
base_link
 ├── camera_link
 │    └── camera_color_optical_frame
 └── hesai_lidar
```

---

## 🐍 Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MyPackage(Node):
    def __init__(self):
        super().__init__('my_package')
        
        self.bridge = CvBridge()
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for posture commands
        self.posture_pub = self.create_publisher(String, '/cmd_posture', 10)
        
        # Publisher for emergency stop
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscriber for camera images
        self.img_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        
        # Stand up on start
        self.posture_pub.publish(String(data='up'))
    
    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # TODO: Your object detection here!
        # detected, x, y = detect_target(cv_image)
        
        # TODO: Your logic here!
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward
        self.cmd_pub.publish(cmd)
    
    def emergency_stop(self):
        """Call this if something goes wrong!"""
        self.estop_pub.publish(Bool(data=True))
        self.cmd_pub.publish(Twist())  # Zero velocity

def main():
    rclpy.init()
    node = MyPackage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---


## 📁 Creating Your Package

```bash
cd ~/robot_ws/EDTH-ros2-starterpack/src
ros2 pkg create --build-type ament_python my_package \
    --dependencies rclpy geometry_msgs sensor_msgs cv_bridge std_msgs

# Edit your code in my_package/my_package/
# Then build:
cd ~/robot_ws/EDTH-ros2-starterpack
colcon build --packages-select my_package
source install/setup.bash
```

---

## ⚠️ Safety

1. **Always have someone ready to catch the robot**
2. **Start with low speeds** (0.2-0.3 m/s)
3. **Test in open areas first**
4. **Know the emergency stop:**
   ```bash
   ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
   ```

---

## 🏗️ Architecture

```
┌─────────────────────┐     ┌──────────────────────┐
│   Your Package      │     │    Keyboard Teleop   │
│      Node           │     │                      │
└─────────┬───────────┘     └──────────┬───────────┘
          │                            │
          │  /cmd_vel                  │  /cmd_vel
          │  /cmd_posture              │
          ▼                            ▼
┌─────────────────────────────────────────────────────┐
│                 go2_control_node                    │
│  (Bridges ROS2 Twist to Unitree Sport API)          │
└─────────────────────────┬───────────────────────────┘
                          │
                          │  /api/sport/request
                          ▼
┌─────────────────────────────────────────────────────┐
│              Unitree Go2 Robot                      │
│         (sport_mode service on-board)               │
└─────────────────────────────────────────────────────┘
```

---

## 📚 Resources

- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [RealSense ROS2](https://github.com/IntelRealSense/realsense-ros)
- [Hesai LiDAR ROS2 Driver](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)

---

**Good luck! 🚀**

*Laelaps AI - EDTH Defense Tech Hackathon*

# Taggy

Taggy is a patrol robot prototype on a Unitree Go2 quadruped. It was built in 48 hours
at the European Defense Tech Hackathon (EDTH) in Zurich, organised by Laelaps AI, in
November 2025. The goal was a quadruped that patrols an area and autonomously detects
and tracks targets.

This repository is the ROS 2 workspace as it stood at the end of the hackathon. It holds
the Laelaps AI Go2 starter pack (velocity and posture control, sensor bringup, keyboard
teleop, Unitree message definitions), the vendored Hesai LiDAR driver, and a Gazebo
simulation package added during the event.

## Status

Hackathon prototype, November 2025. Not maintained. Only the robot bringup, control, and
simulation packages are in this repository; there is no perception or navigation code
here.

## Hardware

- Unitree Go2 quadruped, commanded through the Unitree Sport API (`/api/sport/request`),
  which is handled by the robot's onboard `sport_mode` service
- Intel RealSense D435i camera (RGB, depth, point cloud, IMU)
- Hesai XT16 LiDAR (360-degree point cloud at about 10 Hz, built-in IMU)

Software: ROS 2 Foxy.

## Packages

```
go2_bringup/          Launch files and sensor configs
go2_control/          C++ velocity control node (SportClient API)
go2_interfaces/       Python utilities and message types
go2_examples/         Keyboard teleop for testing
go2_simulation/       Gazebo world (coworking.sdf) and launch file
unitree_api/          Unitree ROS 2 message definitions
unitree_go/           Unitree Go2 message definitions
HesaiLidar_ROS_2.0/   Hesai LiDAR ROS 2 driver (vendored)
```

## Quick start

### 1. Build the workspace

```bash
cd ~/robot_ws/EDTH-ros2-starterpack
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch robot control

```bash
# Start the velocity control node
ros2 launch go2_bringup go2_control.launch.py
```

### 3. Launch sensors

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

`full_bringup.launch.py` starts sensors and control together and takes `robot_ip:=`
(default `192.168.123.161`):

```bash
ros2 launch go2_bringup full_bringup.launch.py
```

### 4. Keyboard teleop

```bash
ros2 run go2_examples keyboard_teleop
```

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
| `X` | Emergency Stop (disables motors) |
| `R` | Release emergency stop |
| `ESC` | Quit |

## Robot control

### Velocity commands

Send velocity commands to `/cmd_vel`:

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5   # Forward (m/s), positive = forward
cmd.linear.y = 0.0   # Strafe (m/s), positive = left
cmd.angular.z = 0.3  # Rotation (rad/s), positive = counter-clockwise

publisher.publish(cmd)
```

Velocity limits:
- Linear velocity: ±1.0 m/s (start with 0.3 m/s)
- Angular velocity: ±1.0 rad/s

### Posture commands

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

Available posture commands: `up`, `down`, `balance`, `recovery`, `sit`, `hello`, `stretch`, `stop`

### Emergency stop

```bash
# Activate emergency stop (disables motors)
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: false" --once
```

## ROS 2 topics

### Control topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (publish here) |
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

### TF frames

```
base_link
 ├── camera_link
 │    └── camera_color_optical_frame
 └── hesai_lidar
```

## Python example

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

## Creating a package

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

## Simulation

`go2_simulation` launches Gazebo with the `coworking.sdf` world:

```bash
ros2 launch go2_simulation gazebo_sim.launch.py
```

The robot spawn in `gazebo_sim.launch.py` is commented out, so this currently opens the
world without a robot model.

## Safety

1. Have someone ready to catch the robot.
2. Start with low speeds (0.2-0.3 m/s).
3. Test in open areas first.
4. Know the emergency stop:
   ```bash
   ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
   ```

## Architecture

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

## Credits

- The starter pack (control node, launch files, interface scripts, keyboard teleop, and
  the original README this one is based on) was written by Laelaps AI, who organised the
  hackathon.
- The Gazebo simulation package was added by Manolis Efthymiou.
- The repository was set up by Oscar Devos (initial import and build fixes).
- `unitree_api` and `unitree_go` are Unitree's message definitions. `HesaiLidar_ROS_2.0`
  is Hesai Technology's driver, vendored with its own README.

## Resources

- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [RealSense ROS2](https://github.com/IntelRealSense/realsense-ros)
- [Hesai LiDAR ROS2 Driver](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)

No license file is included at the root of this repository.

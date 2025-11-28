# Unitree Go2 ROS2 Workspace

This workspace provides a **modular architecture** for the Unitree Go2 robot that supports both **hardware (real robot)** and **simulation (Gazebo)** modes.

## Quick Start

### Hardware Mode (Real Robot)
```bash
# Source workspace
source /home/user/ros2_ws/install/setup.bash

# Launch with real robot (connect to Go2 via Ethernet first)
ros2 launch go2_bringup full_bringup.launch.py

# Or specify robot IP
ros2 launch go2_bringup full_bringup.launch.py robot_ip:=192.168.123.161
```

### Simulation Mode (Gazebo)
```bash
# Source workspace
source /home/user/ros2_ws/install/setup.bash

# Launch simulation
ros2 launch go2_bringup full_bringup.launch.py use_sim:=true

# With outdoor world and RViz
ros2 launch go2_bringup full_bringup.launch.py use_sim:=true world:=outdoor enable_rviz:=true

# Alternative: Direct Gazebo launch
ros2 launch go2_gazebo gazebo.launch.py
ros2 launch go2_bringup simulation.launch.py
```

### Teleoperation
```bash
# In a new terminal (works for both hardware and simulation)
source /home/user/ros2_ws/install/setup.bash

# Custom Go2 keyboard teleop (recommended)
ros2 launch go2_bringup teleop.launch.py

# Standard teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Package Structure

```
ros2_ws/src/
├── go2_bringup/          # Main launch files (modular hw/sim switching)
├── go2_control/          # Hardware control (Unitree API bridge)
├── go2_gazebo/           # Gazebo simulation package
├── go2_description/      # Robot URDF, meshes, xacro (from go_sim_py)
├── go2_examples/         # Examples including keyboard teleop
├── go2_interfaces/       # SDK interfaces
├── HesaiLidar_ROS_2.0/   # Hesai LiDAR driver (hardware)
├── unitree_api/          # Unitree DDS messages
├── unitree_go/           # Unitree Go2 messages
├── go_sim_py/            # Quadruped controller (gait control)
└── autonomy_stack_go2/   # Navigation, SLAM, path planning
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan (2D) |
| `/lidar_points` | `sensor_msgs/PointCloud2` | LiDAR pointcloud (3D) |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth image |
| `/imu/data` | `sensor_msgs/Imu` | IMU data |
| `/joint_states` | `sensor_msgs/JointState` | Joint states |

## Keyboard Teleop Controls

```
╔══════════════════════════════════════════════════════════════╗
║            UNITREE GO2 KEYBOARD TELEOPERATION                ║
╠══════════════════════════════════════════════════════════════╣
║   Movement:                     Rotation:                    ║
║       W                             Q    E                   ║
║     A   D                        (CCW)  (CW)                 ║
║       S                                                      ║
║                                                              ║
║   Posture Commands:                                          ║
║   1 - Stand Up          4 - Recovery (get up from fallen)   ║
║   2 - Sit Down          5 - Hello (wave)                    ║
║   3 - Balance Stand     6 - Stretch                         ║
║                                                              ║
║   Speed Control:        +/- : Increase/Decrease speed       ║
║   SPACE : Stop          X : Emergency Stop   R : Release    ║
╚══════════════════════════════════════════════════════════════╝
```

## Simulation Features

- **Gazebo Classic (Gazebo 11)** - Compatible with ROS2 Foxy
- **Simulated Sensors:**
  - 2D/3D LiDAR (simulates Hesai XT16)
  - RealSense D435i (RGB, Depth, IR, IMU)
  - Body IMU
- **ros2_control** with position controllers
- **Quadruped gait controller** for walking

## Launch Arguments

### `full_bringup.launch.py`
| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `false` | Use simulation mode |
| `robot_ip` | `192.168.123.161` | Robot IP (hardware mode) |
| `world` | `empty` | Gazebo world: `empty`, `outdoor` |
| `enable_camera` | `true` | Enable camera |
| `enable_lidar` | `true` | Enable LiDAR |
| `enable_rviz` | `false` | Launch RViz |

### `gazebo.launch.py`
| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty` | Gazebo world file |
| `use_camera` | `true` | Simulated camera |
| `use_lidar` | `true` | Simulated LiDAR |
| `use_rviz` | `true` | Launch RViz |
| `x`, `y`, `z` | `0, 0, 0.5` | Initial robot position |

## Building

```bash
cd /home/user/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Dependencies

Install required packages:
```bash
sudo apt update
sudo apt install -y \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-xacro \
  ros-foxy-joint-state-publisher \
  ros-foxy-robot-state-publisher \
  ros-foxy-teleop-twist-keyboard \
  ros-foxy-teleop-twist-joy \
  ros-foxy-joy \
  ros-foxy-perception-pcl \
  ros-foxy-pcl-ros
```

## Troubleshooting

### Robot not responding to commands
1. Ensure robot is in standing position (press `1` in teleop)
2. Check `/cmd_vel` topic is publishing: `ros2 topic echo /cmd_vel`
3. In simulation, wait for controllers to load (~5 seconds after spawn)

### Gazebo crashes or hangs
1. Try with empty world: `ros2 launch go2_gazebo gazebo.launch.py world:=empty`
2. Reduce physics update rate if CPU limited

### No sensor data
1. Check topics: `ros2 topic list | grep -E "scan|camera|imu"`
2. In simulation, sensors publish under `/go2/` namespace

---
*Created by Laelaps AI*

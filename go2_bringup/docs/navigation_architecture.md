# Go2 Navigation Architecture

Full navigation stack for autonomous exploration with SLAM.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              EXPLORATION LAYER                                       │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │                         explore_node                                         │    │
│  │                    (explore_lite/explore)                                    │    │
│  │                                                                              │    │
│  │  Subscribes:  /map                                                           │    │
│  │  Publishes:   /explore/frontiers (MarkerArray)                               │    │
│  │  Actions:     /navigate_to_pose (client)                                     │    │
│  └──────────────────────────────┬──────────────────────────────────────────────┘    │
│                                 │                                                    │
│                                 │ NavigateToPose Action Goal                         │
│                                 ▼                                                    │
└─────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              NAVIGATION LAYER (Nav2)                                 │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │                        bt_navigator                                          │    │
│  │                  (nav2_bt_navigator)                                         │    │
│  │                                                                              │    │
│  │  Actions:  /navigate_to_pose (server)                                        │    │
│  │            /navigate_through_poses (server)                                  │    │
│  │  Subscribes: /odom                                                           │    │
│  └──────────────────────────────┬──────────────────────────────────────────────┘    │
│                                 │                                                    │
│            ┌────────────────────┼────────────────────┐                              │
│            │                    │                    │                              │
│            ▼                    ▼                    ▼                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐                      │
│  │  planner_server │  │controller_server│  │ behavior_server │                      │
│  │   (NavFn)       │  │    (DWB)        │  │  (recoveries)   │                      │
│  │                 │  │                 │  │                 │                      │
│  │ /compute_path   │  │ /follow_path    │  │ /spin           │                      │
│  │ _to_pose        │  │                 │  │ /backup         │                      │
│  │                 │  │ Pub: /cmd_vel   │  │ /wait           │                      │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────┘                      │
│           │                    │                                                     │
│           │                    │                                                     │
│  ┌────────┴────────────────────┴────────────────────────────────────────────────┐   │
│  │                           COSTMAPS                                            │   │
│  │  ┌─────────────────────────┐    ┌─────────────────────────────────────────┐  │   │
│  │  │    global_costmap       │    │         local_costmap                   │  │   │
│  │  │                         │    │                                         │  │   │
│  │  │  Frame: map             │    │  Frame: odom                            │  │   │
│  │  │  Subscribes: /map       │    │  Rolling window: 3x3m                   │  │   │
│  │  │              /scan      │    │  Subscribes: /scan                      │  │   │
│  │  │                         │    │                                         │  │   │
│  │  │  Plugins:               │    │  Plugins:                               │  │   │
│  │  │   - static_layer        │    │   - voxel_layer                         │  │   │
│  │  │   - obstacle_layer      │    │   - inflation_layer                     │  │   │
│  │  │   - inflation_layer     │    │                                         │  │   │
│  │  └─────────────────────────┘    └─────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              SLAM LAYER                                              │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │                        slam_toolbox                                          │    │
│  │              (async_slam_toolbox_node)                                       │    │
│  │                                                                              │    │
│  │  Mode: Online Async (Lifelong Mapping)                                       │    │
│  │                                                                              │    │
│  │  Subscribes:  /scan (LaserScan)                                              │    │
│  │  Publishes:   /map (OccupancyGrid)                                           │    │
│  │               /map_metadata                                                  │    │
│  │  TF:          map → odom                                                     │    │
│  └─────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              SENSOR LAYER                                            │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌───────────────────────────┐    ┌─────────────────────────────────────────────┐   │
│  │   hesai_ros_driver_node   │    │       pointcloud_to_laserscan               │   │
│  │      (Hesai XT16)         │    │                                             │   │
│  │                           │    │  Subscribes: /lidar_points                  │   │
│  │  Publishes:               │───▶│  Publishes:  /scan                          │   │
│  │   /lidar_points           │    │                                             │   │
│  │   (PointCloud2)           │    │  Params:                                    │   │
│  │                           │    │   - min_height: -0.15m                      │   │
│  │  Frame: hesai_lidar       │    │   - max_height: 0.15m                       │   │
│  └───────────────────────────┘    └─────────────────────────────────────────────┘   │
│                                                                                      │
│  ┌───────────────────────────┐                                                      │
│  │   realsense2_camera_node  │                                                      │
│  │      (D435i)              │                                                      │
│  │                           │                                                      │
│  │  Publishes:               │                                                      │
│  │   /camera/color/image_raw │                                                      │
│  │   /camera/depth/image_raw │                                                      │
│  │   /camera/depth/points    │                                                      │
│  │   /camera/imu             │                                                      │
│  └───────────────────────────┘                                                      │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              ROBOT LAYER                                             │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌───────────────────────────┐    ┌─────────────────────────────────────────────┐   │
│  │    go2_odometry           │    │         go2_control_node                    │   │
│  │                           │    │                                             │   │
│  │  Publishes:               │    │  Subscribes:                                │   │
│  │   /odom (Odometry)        │    │   /cmd_vel (Twist)                          │   │
│  │   /imu  (Imu)             │    │   /cmd_posture (String)                     │   │
│  │                           │    │   /emergency_stop (Bool)                    │   │
│  │  TF: odom → base_link     │    │                                             │   │
│  │                           │    │  Publishes:                                 │   │
│  │  Source: Go2 leg          │    │   /api/sport/request                        │   │
│  │  kinematics + IMU         │    │                                             │   │
│  └───────────────────────────┘    └─────────────────────────────────────────────┘   │
│                                                                                      │
│  ┌───────────────────────────────────────────────────────────────────────────────┐  │
│  │                         Unitree Go2 Robot                                      │  │
│  │                    (sport_mode service on-board)                               │  │
│  └───────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## TF Tree

```
map
 │
 │  (slam_toolbox)
 │
 └── odom
      │
      │  (go2_odometry)
      │
      └── base_link
           │
           ├── hesai_lidar      (static_transform_publisher)
           │
           └── camera_link      (static_transform_publisher)
                │
                └── camera_color_optical_frame
                └── camera_depth_optical_frame
```

## Topics Summary

### Sensor Topics
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/lidar_points` | `PointCloud2` | hesai_ros_driver | 3D LiDAR point cloud |
| `/scan` | `LaserScan` | pointcloud_to_laserscan | 2D laser scan (from LiDAR) |
| `/camera/color/image_raw` | `Image` | realsense2_camera | RGB camera image |
| `/camera/depth/image_rect_raw` | `Image` | realsense2_camera | Depth image |

### Odometry Topics
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/odom` | `Odometry` | go2_odometry | Robot odometry (leg kinematics + IMU) |
| `/imu` | `Imu` | go2_odometry | IMU data |

### SLAM Topics
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/map` | `OccupancyGrid` | slam_toolbox | Occupancy grid map |
| `/map_metadata` | `MapMetaData` | slam_toolbox | Map metadata |

### Navigation Topics
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/cmd_vel` | `Twist` | controller_server | Velocity commands to robot |
| `/global_costmap/costmap` | `OccupancyGrid` | global_costmap | Global costmap |
| `/local_costmap/costmap` | `OccupancyGrid` | local_costmap | Local costmap |
| `/plan` | `Path` | planner_server | Global path |

### Exploration Topics
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/explore/frontiers` | `MarkerArray` | explore_node | Frontier visualization |

### Control Topics (twist_mux)

Velocity commands are multiplexed by `twist_mux` with priority:

| Topic | Priority | Source | Description |
|-------|----------|--------|-------------|
| `/servo_cmd_vel` | 2 (highest) | visual_servo_node | Visual servoing (IBVS + LiDAR safety) |
| `/nav_cmd_vel` | 1 | Nav2 controller_server | Autonomous navigation |
| `/teleop_cmd_vel` | 0 (lowest) | keyboard_teleop | Manual control |
| `/cmd_vel` | output | twist_mux | Final command to go2_control_node |

### Other Control Topics
| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/cmd_posture` | `String` | go2_control_node | Posture commands |
| `/emergency_stop` | `Bool` | go2_control_node | Emergency stop (locks twist_mux) |

## Action Servers

| Action | Type | Server | Description |
|--------|------|--------|-------------|
| `/navigate_to_pose` | `NavigateToPose` | bt_navigator | Navigate to a single pose |
| `/navigate_through_poses` | `NavigateThroughPoses` | bt_navigator | Navigate through waypoints |
| `/compute_path_to_pose` | `ComputePathToPose` | planner_server | Compute global path |
| `/follow_path` | `FollowPath` | controller_server | Execute path following |
| `/spin` | `Spin` | behavior_server | Recovery: spin in place |
| `/backup` | `BackUp` | behavior_server | Recovery: back up |
| `/wait` | `Wait` | behavior_server | Recovery: wait |

## Launch Files

| Launch File | Components | Description |
|-------------|------------|-------------|
| `sensors.launch.py` | LiDAR, Camera, pointcloud_to_laserscan | Sensor drivers |
| `go2_control.launch.py` | go2_control_node, go2_odometry | Robot control & odometry |
| `slam.launch.py` | slam_toolbox | SLAM mapping only |
| `navigation.launch.py` | Nav2 stack | Navigation only |
| `explore.launch.py` | explore_lite | Autonomous exploration only |
| `full_navigation.launch.py` | SLAM + Nav2 + explore_lite | Complete navigation stack |

## Typical Startup Sequence

```bash
# Terminal 1: Robot control and odometry
ros2 launch go2_bringup go2_control.launch.py

# Terminal 2: Sensors
ros2 launch go2_bringup sensors.launch.py

# Terminal 3: Full navigation (SLAM + Nav2 + Exploration)
ros2 launch go2_bringup full_navigation.launch.py

# Or separately:
# Terminal 3: SLAM
ros2 launch go2_bringup slam.launch.py

# Terminal 4: Navigation
ros2 launch go2_bringup navigation.launch.py

# Terminal 5: Exploration (optional)
ros2 launch go2_bringup explore.launch.py
```

## Key Parameters

### Go2 Robot
- **Footprint**: ~0.6m x 0.3m
- **Robot radius**: 0.35m (for costmap)
- **Inflation radius**: 0.55m
- **Max velocity**: 0.5 m/s (forward), 0.3 m/s (strafe), 1.0 rad/s (rotation)

### SLAM Toolbox
- **Mode**: Online Async (lifelong mapping)
- **Resolution**: 0.05m (5cm)
- **Max laser range**: 20m

### Nav2
- **Local costmap**: 3x3m rolling window
- **Controller**: DWB (Dynamic Window Approach)
- **Planner**: NavFn (A* based)


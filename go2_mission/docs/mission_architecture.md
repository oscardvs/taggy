# Mission Executive Architecture

Finite State Machine for autonomous search-and-find behavior on Unitree Go2.

## System Overview

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                    MISSION LAYER                                        │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   ┌─────────────────────────────────────────────────────────────────────────────────┐  │
│   │                         mission_executive_node                                   │  │
│   │                        (Finite State Machine)                                    │  │
│   │                                                                                  │  │
│   │   ┌────────┐    ┌───────────┐    ┌───────────┐    ┌──────────┐    ┌─────────┐  │  │
│   │   │  IDLE  │───▶│ LISTENING │───▶│ SEARCHING │───▶│ TRACKING │───▶│ ARRIVED │  │  │
│   │   └────────┘    └───────────┘    └───────────┘    └──────────┘    └─────────┘  │  │
│   │       ▲              │                │                │               │        │  │
│   │       │          timeout            fail            lost             delay      │  │
│   │       └──────────────┴────────────────┴────────────────┴───────────────┘        │  │
│   │                                                                                  │  │
│   │   Subscriptions:                      Publications:                              │  │
│   │     • /wirelesscontroller               • /cmd_posture                           │  │
│   │     • /mission/target_object            • /mission/state                         │  │
│   │     • /detection/bbox                   • /explore/resume                        │  │
│   │     • /detection/target_pose                                                     │  │
│   │     • /odom                                                                      │  │
│   │     • /explore/exploring                                                         │  │
│   └─────────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                         │
│   ┌─────────────────────────────────────────────────────────────────────────────────┐  │
│   │                          visual_servo_node                                       │  │
│   │              (IBVS Controller + LiDAR Safety Wrapper)                            │  │
│   │                                                                                  │  │
│   │   Control Laws:                                                                  │  │
│   │     Yaw:     ω_z = K_p_yaw * (u_center - u_target) / image_width                │  │
│   │     Forward: v_x = clamp(K_p_dist * (d_current - d_goal), 0, v_max)             │  │
│   │                                                                                  │  │
│   │   Safety:                                                                        │  │
│   │     if min(scan[frontal_cone ±30°]) < 0.3m: v_x = 0                             │  │
│   │                                                                                  │  │
│   │   Subscriptions:                      Publications:                              │  │
│   │     • /mission/state                    • /servo_cmd_vel                         │  │
│   │     • /mission/target_object                                                     │  │
│   │     • /detection/bbox                                                            │  │
│   │     • /detection/target_pose                                                     │  │
│   │     • /scan                                                                      │  │
│   └─────────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                         │
└────────────────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              VELOCITY MUX LAYER                                         │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   ┌─────────────────────────────────────────────────────────────────────────────────┐  │
│   │                              twist_mux                                           │  │
│   │                     (Priority-based Velocity Multiplexer)                        │  │
│   │                                                                                  │  │
│   │    Priority 2 ──▶ /servo_cmd_vel ──┐                                            │  │
│   │                                     │                                            │  │
│   │    Priority 1 ──▶ /nav_cmd_vel ────┼──▶ /cmd_vel ──▶ go2_control_node           │  │
│   │                                     │                                            │  │
│   │    Priority 0 ──▶ /teleop_cmd_vel ─┘                                            │  │
│   │                                                                                  │  │
│   │    Lock ────────▶ /emergency_stop (disables all when True)                      │  │
│   └─────────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                         │
└────────────────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              NAVIGATION LAYER                                           │
├────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   ┌───────────────────────┐         ┌───────────────────────────────────────────────┐  │
│   │     explore_node      │         │              Nav2 Stack                        │  │
│   │    (explore_lite)     │         │                                                │  │
│   │                       │  Goal   │  bt_navigator ◀── /navigate_to_pose (action)  │  │
│   │  Subscribes: /map     │────────▶│       │                                        │  │
│   │  Publishes:           │         │       ▼                                        │  │
│   │    /explore/frontiers │         │  controller_server ──▶ /nav_cmd_vel           │  │
│   │    /explore/exploring │         │                                                │  │
│   └───────────────────────┘         └───────────────────────────────────────────────┘  │
│                                                                                         │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

## State Machine Diagram

```
                              ┌─────────────────────────────────┐
                              │                                 │
                              ▼                                 │
                        ┌──────────┐                            │
           ┌───────────│   IDLE   │◀───────────────┐            │
           │           └──────────┘                │            │
           │                 │                     │            │
           │           L2+A pressed               │            │
           │                 │                     │            │
           │                 ▼                     │            │
           │           ┌──────────┐          5s delay           │
           │  timeout  │LISTENING │                │            │
           │  (10s)    └──────────┘                │            │
           │                 │                     │            │
           └─────────────────┤                     │            │
                             │ target_object       │            │
                             │ received            │            │
                             ▼                     │            │
                       ┌──────────┐          ┌──────────┐       │
           ┌──────────│SEARCHING │─────────▶│  FAILED  │───────┤
           │          └──────────┘  no       └──────────┘       │
           │                │      frontiers                    │
           │                │                                   │
           │          target detected                           │
           │                │                                   │
           │                ▼                                   │
           │          ┌──────────┐                              │
           │  lost    │ TRACKING │  ◀── visual_servo_node       │
           │  (>2s)   └──────────┘      handles velocity        │
           │                │                                   │
           └────────────────┤                                   │
                            │ distance < 0.5m                   │
                            │                                   │
                            ▼                                   │
                      ┌──────────┐                              │
                      │ ARRIVED  │──────────────────────────────┘
                      └──────────┘
                           5s delay
```

## State Descriptions

| State | Entry Action | Behavior | Exit Conditions |
|-------|--------------|----------|-----------------|
| **IDLE** | `stand` posture, disable exploration | Monitors `/wirelesscontroller` for L2+A | L2+A pressed → LISTENING |
| **LISTENING** | `balance` posture | Waits for `/mission/target_object` | Object received → SEARCHING, 10s timeout → IDLE |
| **SEARCHING** | Enable explore_lite | Nav2 + explore_lite active, monitors `/detection/bbox` | Target detected → TRACKING, No frontiers → FAILED |
| **TRACKING** | Cancel Nav2 goal, disable explore | visual_servo_node handles IBVS control + safety | Distance < 0.5m → ARRIVED, Lost > 2s → SEARCHING |
| **ARRIVED** | `sit` posture | Terminal action | 5s delay → IDLE |
| **FAILED** | `sit` posture | Mission unsuccessful | 5s delay → IDLE |

## Topic Reference

### Subscriptions (Mission Executive)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/wirelesscontroller` | `unitree_go/WirelessController` | Go2 robot | Controller button states (keys bitmask) |
| `/mission/target_object` | `std_msgs/String` | voice_command_node | Target object name from voice |
| `/detection/bbox` | `vision_msgs/Detection2DArray` | perception_node | Object detections with class IDs |
| `/detection/target_pose` | `geometry_msgs/PoseStamped` | perception_node | 3D target position (for arrival detection) |
| `/odom` | `nav_msgs/Odometry` | go2_odometry | Robot position and velocity |
| `/explore/exploring` | `std_msgs/Bool` | explore_lite | Exploration status (false = complete) |

### Publications (Mission Executive)

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/cmd_posture` | `std_msgs/String` | go2_control_node | Posture commands: `stand`, `balance`, `sit` |
| `/mission/state` | `std_msgs/String` | visual_servo_node, debugging | Current FSM state name |
| `/explore/resume` | `std_msgs/Bool` | explore_lite | Enable/disable exploration |

### Subscriptions (Visual Servo)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/mission/state` | `std_msgs/String` | mission_executive | Activate when state is "TRACKING" |
| `/mission/target_object` | `std_msgs/String` | voice_command_node | Target class name to track |
| `/detection/bbox` | `vision_msgs/Detection2DArray` | perception_node | 2D bounding boxes for image-space tracking |
| `/detection/target_pose` | `geometry_msgs/PoseStamped` | perception_node | 3D target position (depth for forward control) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | 2D laser scan for safety reflex |

### Publications (Visual Servo)

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/servo_cmd_vel` | `geometry_msgs/Twist` | twist_mux | Visual servo velocity commands |

### Velocity Topics (twist_mux)

| Topic | Priority | Source | Description |
|-------|----------|--------|-------------|
| `/servo_cmd_vel` | 2 (highest) | visual_servo_node | Visual servo tracking |
| `/nav_cmd_vel` | 1 | Nav2 controller_server | Autonomous navigation |
| `/teleop_cmd_vel` | 0 (lowest) | keyboard_teleop | Manual control |
| `/cmd_vel` | output | twist_mux → go2_control | Final velocity to robot |

### Action Clients

| Action | Type | Server | Description |
|--------|------|--------|-------------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | bt_navigator | Cancel navigation goals |

## Visual Servoing Control Laws

The `visual_servo_node` implements Image-Based Visual Servoing (IBVS):

### Lateral Control (Yaw)

Centers the target horizontally in the camera frame:

```
e_u = u_principal - u_target
ω_z = K_p_yaw * e_u / image_width
```

- `u_principal`: Image center x-coordinate (320 for 640px width)
- `u_target`: Detected target bounding box center x-coordinate
- `K_p_yaw`: Proportional gain (default: 1.0)

### Longitudinal Control (Forward)

Approaches target to goal distance:

```
e_d = d_current - d_goal
v_x = clamp(K_p_dist * e_d, 0, v_max)
```

- `d_current`: Actual depth from `/detection/target_pose`
- `d_goal`: Stopping distance (default: 0.5m)
- `K_p_dist`: Proportional gain (default: 0.5)
- `v_max`: Maximum linear velocity (default: 0.3 m/s)

### Safety Wrapper (LiDAR Reflex)

Prevents collisions during tracking:

```
d_obs = min(scan[frontal_cone ±30°])
if d_obs < d_safe: v_x = 0
```

- `d_safe`: Safety threshold (default: 0.3m)
- Frontal cone: ±30° from forward direction (60° total)
- Reflex-level inhibition overrides forward velocity

## Button Mapping

Unitree Go2 Wireless Controller `keys` bitmask:

```
Bit 0:  A       (0x0001)  ◀── Confirm
Bit 1:  B       (0x0002)
Bit 2:  X       (0x0004)
Bit 3:  Y       (0x0008)
Bit 4:  UP      (0x0010)
Bit 5:  DOWN    (0x0020)
Bit 6:  L1      (0x0040)
Bit 7:  R1      (0x0080)
Bit 8:  L2      (0x0100)  ◀── Safety Enable
Bit 9:  R2      (0x0200)
Bit 10: SELECT  (0x0400)
Bit 11: START   (0x0800)
```

**Trigger Mask:** `L2 + A = 0x0101` (Safety Enable + Confirm)

## Configuration Parameters

### Mission Executive

```yaml
mission_executive_node:
  ros__parameters:
    listening_timeout: 10.0    # seconds
    target_lost_timeout: 2.0   # seconds
    search_timeout: 300.0      # seconds (0=disabled)
    arrived_delay: 5.0         # seconds
    failed_delay: 5.0          # seconds
    arrival_distance: 0.5      # meters
```

**Note on explore_lite integration:** Standard explore_lite does NOT provide `/explore/resume` or `/explore/exploring` topics. The system handles this via:
1. `search_timeout` parameter for SEARCHING state timeout
2. `cancel_nav2_goal()` to stop active navigation
3. twist_mux priority (visual_servo at priority 2 overrides Nav2 at priority 1)

### Visual Servo

```yaml
visual_servo_node:
  ros__parameters:
    # IBVS gains
    servo_kp_yaw: 1.0          # Proportional gain for angular control
    servo_kp_distance: 0.5     # Proportional gain for linear control
    servo_max_linear_vel: 0.3  # m/s cap
    servo_max_angular_vel: 0.5 # rad/s cap
    
    # Safety
    safety_distance: 0.3       # Minimum obstacle distance (m)
    safety_cone_angle: 30.0    # Half-angle of frontal cone (degrees)
    goal_distance: 0.5         # Stopping distance from target (m)
    
    # Image
    image_width: 640
    image_height: 480
    detection_timeout: 0.5     # seconds
```

## Launch Commands

```bash
# Full autonomous stack
ros2 launch go2_bringup mission_bringup.launch.py

# Mission executive only (requires nav stack running)
ros2 launch go2_mission mission.launch.py

# With custom parameters
ros2 launch go2_mission mission.launch.py \
    listening_timeout:=15.0 \
    arrival_distance:=0.3 \
    safety_distance:=0.4 \
    servo_kp_yaw:=1.2
```

## Node Graph

```
                    ┌─────────────────────┐
                    │  /wirelesscontroller │
                    └──────────┬──────────┘
                               │
                               ▼
┌──────────────┐    ┌─────────────────────┐    ┌──────────────┐
│voice_command │───▶│ mission_executive   │───▶│ /cmd_posture │
│    _node     │    │       _node         │    └──────────────┘
└──────────────┘    └─────────────────────┘
       │                      │
       │                      │ /mission/state
       ▼                      ▼
/mission/target     ┌─────────────────────┐
    _object         │  visual_servo_node  │◀───────┐
       │            │  (IBVS + Safety)    │        │
       │            └─────────────────────┘        │
       │                      │                    │
       │                      │ /servo_cmd_vel     │ /scan
       │                      ▼                    │
       │            ┌─────────────────────┐    ┌───────────┐
       │            │     twist_mux       │    │  LiDAR    │
       │            └─────────────────────┘    │  Driver   │
       │                      │                └───────────┘
       │                      │ /cmd_vel
       │                      ▼
┌──────────────┐    ┌─────────────────────┐
│  perception  │───▶│   go2_control_node  │
│    _node     │    └─────────────────────┘
└──────────────┘              │
       │                      ▼
       │                Go2 Robot API
       │
       ├──▶ /detection/bbox
       └──▶ /detection/target_pose


┌──────────────┐    ┌─────────────────────┐
│ explore_node │───▶│     Nav2 Stack      │
│(explore_lite)│    │                     │
└──────────────┘    │  /nav_cmd_vel ──────┼───▶ twist_mux
       │            └─────────────────────┘
       ▼
/explore/exploring
```

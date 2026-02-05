# Lunabot Drive Package

Motor drive control and sensor integration for Lunabot rover using SparkFlex controllers via CAN bus and OAK-D S2 depth camera.

## Package Structure

```
lunabot_drive/
├── src/
│   ├── drive_node.cpp              # Motor control + wheel odometry
│   └── minimal_drive.cpp           # Minimal joystick test node
├── nodes/
│   └── bandwidth_monitor.py        # Competition bandwidth monitoring
├── launch/
│   ├── hardware_bringup.launch.py  # Unified hardware launch (Pi)
│   ├── motors_rviz.launch.py       # Motor control + URDF visualization in RViz
│   ├── oak_d_camera.launch.py      # OAK-D S2 camera launch
│   ├── oak_d_rviz.launch.py        # Camera + RViz visualization
│   ├── pc_teleop.launch.py         # Offboard PC (joy + teleop)
│   ├── pi_drive.launch.py          # Raspberry Pi (motor control)
│   ├── camera_only_nav2.launch.py  # Nav2 costmap test (stationary)
│   ├── camera_mapping_test.launch.py # Camera + pointcloud_to_laserscan
│   ├── motor_test_rviz.launch.py   # Motor test with RViz feedback
│   └── apriltag_detection.launch.py # AprilTag detection for localization
├── config/
│   ├── oak_d_camera.yaml           # Full camera config (RGB + Depth + PC)
│   ├── oak_d_pointcloud_only.yaml  # Bandwidth-optimized pointcloud
│   ├── oak_d_rgb_only.yaml         # RGB only (low latency)
│   ├── camera_only_nav2.yaml       # Nav2 costmap test params
│   ├── switch_pro.yaml             # Controller mapping
│   ├── params/
│   │   ├── nav2_params.yaml        # Full Nav2 stack parameters
│   │   └── ekf_params.yaml         # robot_localization EKF config
│   └── rviz/
│       ├── oak_d_camera.rviz       # Camera + robot model RViz config
│       └── camera_only_nav2.rviz   # Nav2 costmap RViz config
├── description/
│   ├── robot.urdf.xacro            # Main URDF (includes below)
│   ├── robot_core.xacro            # Chassis + 4 wheels
│   ├── oak_d_s2.xacro             # Camera body (optical frames from depthai)
│   └── inertial_macros.xacro       # Inertia calculation helpers
├── docs/
│   ├── OAK_D_S2_INTEGRATION.md    # Camera integration guide
│   └── CAMERA_ONLY_NAV2.md         # Nav2 costmap testing guide
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select lunabot_drive
source install/setup.bash
```

## Quick Start: Hardware Bringup

Launch all hardware on the Raspberry Pi:

```bash
# Full bringup (motors + camera + EKF + teleop)
ros2 launch lunabot_drive hardware_bringup.launch.py

# Without teleop (for autonomous operation)
ros2 launch lunabot_drive hardware_bringup.launch.py enable_teleop:=false

# Without EKF (drive_node handles odom TF directly)
ros2 launch lunabot_drive hardware_bringup.launch.py use_ekf:=false
```

## Individual Component Testing

### Motor Test (with RViz visualization)

```bash
# On Pi: launch motor control
ros2 launch lunabot_drive pi_drive.launch.py

# On PC: visualize + optionally teleop
ros2 launch lunabot_drive motor_test_rviz.launch.py enable_teleop:=true
```

### Camera Test (pointcloud + mapping prep)

```bash
# On Pi: launch camera with pointcloud config
ros2 launch lunabot_drive oak_d_camera.launch.py \
  config:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml

# On PC: visualize + pointcloud_to_laserscan
ros2 launch lunabot_drive camera_mapping_test.launch.py
```

### AprilTag Detection

```bash
ros2 launch lunabot_drive apriltag_detection.launch.py
```

### Bandwidth Monitoring

```bash
ros2 run lunabot_drive bandwidth_monitor.py
```

## Motor Visualization

The `motors_rviz.launch.py` file provides real-time visualization of the robot's wheels and motor positions in RViz2 using the robot URDF model.

### What It Does

This launch file combines:
- **drive_node** - Motor control + joint state publishing (wheel positions/velocities)
- **robot_state_publisher** - Converts URDF + joint states into TF transforms
- **rviz2** - 3D visualization of the robot model with moving wheels

### Quick Start

**Typical Setup (motors on Pi, RViz on PC):**
```bash
# On Raspberry Pi (publishes joint states):
export ROS_DOMAIN_ID=42
ros2 launch lunabot_drive motors_rviz.launch.py

# On PC (visualization):
export ROS_DOMAIN_ID=42
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/rviz/oak_d_camera.rviz
```

**Single Machine with Display (if Pi has display):**
```bash
ros2 launch lunabot_drive motors_rviz.launch.py rviz:=true
```

**With Teleop Control:**
```bash
# Terminal 1 (PC): Launch joystick + teleop
ros2 launch lunabot_drive pc_teleop.launch.py

# Terminal 2 (Pi): Launch motors + visualization
ros2 launch lunabot_drive motors_rviz.launch.py
```

### Launch Arguments

- `rviz:=true|false` - Enable/disable RViz2 (default: false, since Pi is headless)
- `rviz_config:=<path>` - Custom RViz config file path

### Topics

**Published:**
- `/joint_states` - Wheel positions and velocities
- `/tf`, `/tf_static` - Robot coordinate transforms

**Subscribed:**
- `/cmd_vel` - Velocity commands for motors

### Prerequisites

- CAN interface must be up: `sudo ip link set can0 up type can`
- SparkFlex motor controllers connected to CAN bus
- Motor IDs must match configuration (1-4 by default)
- For network operation: ROS_DOMAIN_ID must match on all machines

## Camera Topics

All camera topics use the `/oak/` prefix (node name: `oak`):
- `/oak/rgb/image_raw` - RGB camera feed
- `/oak/stereo/image_raw` - Depth image
- `/oak/points` - PointCloud2
- `/oak/imu/data` - IMU (BNO086, 100 Hz)

## Drive Node Topics

- **Subscribes:** `/cmd_vel` (geometry_msgs/Twist)
- **Publishes:** `/joint_states` (sensor_msgs/JointState), `/odom` (nav_msgs/Odometry)
- **TF:** `odom -> base_link` (when `publish_odom_tf:=true`)

## TF Frame Tree

```
map -> odom -> base_link -> chassis -> oak -> {optical frames from depthai}
                  |-> left_front_wheel
                  |-> right_front_wheel
                  |-> left_wheel (rear left)
                  |-> right_wheel (rear right)
```

## Parameters (drive_node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | CAN bus interface name |
| `left_front_id` | `1` | CAN ID for left front motor |
| `right_front_id` | `2` | CAN ID for right front motor |
| `left_rear_id` | `3` | CAN ID for left rear motor |
| `right_rear_id` | `4` | CAN ID for right rear motor |
| `wheel_base` | `0.762` | Distance between left/right wheels (m) |
| `wheel_radius` | `0.1778` | Wheel radius (m) |
| `gear_ratio` | `1.0` | Motor-to-wheel gear ratio |
| `max_duty_cycle` | `0.8` | Maximum motor duty cycle (0.0-1.0) |
| `joint_state_rate` | `50.0` | Joint state publish rate (Hz) |
| `publish_odom_tf` | `true` | Publish odom->base_link TF (set false when EKF runs) |

## Controller Mapping

- **Left Stick Y**: Forward/Backward
- **Right Stick X**: Turn Left/Right
- **R Button (hold)**: Enable motor control (safety feature)
- **L Button (hold)**: Turbo mode (faster speeds)

## Safety Features

- **Watchdog timer**: Motors stop if no command received for 500ms
- **Enable button**: Must hold R button to control motors
- **Speed limiting**: Commands clamped to max_duty_cycle

## Network Setup

Both computers must be on the same network and use the same ROS_DOMAIN_ID:
```bash
export ROS_DOMAIN_ID=42
```

## Troubleshooting

**No /joy topic:**
- Check controller is connected: `cat /proc/bus/input/devices | grep "Pro Controller"`
- Verify ROS_DOMAIN_ID matches on both machines

**Motors not responding:**
- Check CAN interface: `ip link show can0`
- Verify CAN IDs match motor configuration
- Ensure you're holding the R button (enable button)

**No pointcloud:**
- Check USB 3.0: `lsusb -t | grep 5000M`
- Check topic: `ros2 topic hz /oak/points`
- Verify camera node: `ros2 node list | grep oak`

**TF errors:**
- View frame tree: `ros2 run tf2_tools view_frames`
- Check for broken links in the `map -> odom -> base_link -> oak` chain

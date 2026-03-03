# Lunabot Drive

ROS2 Jazzy package for Northwestern's NASA Lunabotics 2025-2026 rover. Runs on Raspberry Pi 5 with SparkFlex motors via CAN bus, OAK-D S2 depth camera, and Firgelli linear actuators for the bucket mechanism.

```
sudo slcand -o -s8 /dev/ttyACM0 can0 && sudo ip link set can0 up
```

---

## Package Structure

```
src/
  drive_node.cpp              # Motor control + wheel odometry (CAN)
  minimal_drive.cpp           # Minimal joystick test node
nodes/
  actuator_driver_node.py     # Bucket lift/tilt linear actuator driver
  bucket_teleop_node.py       # Joystick → bucket commands (run on PC)
  bandwidth_monitor.py        # Competition bandwidth monitoring
launch/
  hardware_bringup.launch.py  # Unified hardware launch (Pi)
  bucket_bringup.launch.py    # Both bucket actuators
  actuator_test.launch.py     # Single actuator test
  oak_d_camera.launch.py      # OAK-D S2 camera
  pc_teleop.launch.py         # Offboard PC (joy + teleop)
  pi_drive.launch.py          # Pi motor control only
  motor_test_rviz.launch.py   # Motor test with RViz feedback
  camera_only_nav2.launch.py  # Nav2 costmap test (no motors)
  camera_mapping_test.launch.py
  apriltag_detection.launch.py
config/
  bucket_actuators.yaml       # Lift + tilt actuator parameters
  oak_d_pointcloud_only.yaml  # Bandwidth-optimized pointcloud
  oak_d_camera.yaml           # Full camera config
  switch_pro.yaml             # Controller mapping
  params/
    nav2_params.yaml
    ekf_params.yaml
description/
  robot.urdf.xacro            # Full robot URDF
docs/
  OAK_D_S2_INTEGRATION.md
  CAMERA_ONLY_NAV2.md
  motor_logging.md            # Planned motor CSV logging (not yet implemented)
```

---

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select lunabot_drive
source install/setup.bash
```

---

## Quick Start

### Full Hardware Bringup (Pi)

```bash
ros2 launch lunabot_drive hardware_bringup.launch.py
# Options:
#   enable_teleop:=false    (for autonomous operation)
#   use_ekf:=false          (drive_node handles odom TF directly)
```

### Drive Motors + Teleop

```bash
# Pi
ros2 launch lunabot_drive pi_drive.launch.py

# PC
ros2 launch lunabot_drive pc_teleop.launch.py
```

### Bucket Actuators

```bash
# Pi — both actuators
ros2 launch lunabot_drive bucket_bringup.launch.py

# Pi — single actuator (for testing)
ros2 launch lunabot_drive actuator_test.launch.py which:=lift
ros2 launch lunabot_drive actuator_test.launch.py which:=tilt home_on_startup:=false

# PC — joystick teleop for bucket
ros2 run joy joy_node
ros2 run lunabot_drive bucket_teleop_node
```

**Bucket teleop controls (Switch Pro Controller):**

| Input | Action |
|---|---|
| D-pad UP / DOWN | Lift ±10 mm |
| D-pad RIGHT / LEFT | Tilt ±10 mm |
| Y | Home both actuators |
| X | Scoop preset (lift=0, tilt=max) |
| B | Dump preset (lift=max, tilt=0) |

### Camera

```bash
ros2 launch lunabot_drive oak_d_camera.launch.py
```

### Bandwidth Monitor

```bash
ros2 run lunabot_drive bandwidth_monitor.py
```

---

## Topics

### Drive Node

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | Subscribed |
| `/joint_states` | sensor_msgs/JointState | Published |
| `/odom` | nav_msgs/Odometry | Published |

### Bucket Actuators (per namespace: `/bucket/lift`, `/bucket/tilt`)

| Topic | Type | Description |
|---|---|---|
| `~/command` | std_msgs/Float64 | Target position in mm |
| `~/position` | std_msgs/Float64 | Current position in mm |
| `~/status` | diagnostic_msgs/DiagnosticStatus | State + diagnostics |
| `/joint_states` | sensor_msgs/JointState | Merged with drive_node |

Services: `~/home` (Trigger), `~/stop` (Trigger)

### Camera (`/oak/` prefix)

- `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/points`, `/oak/imu/data`

---

## TF Frame Tree

```
map → odom → base_link → chassis → oak → {optical frames from depthai}
                 |→ left_front_wheel
                 |→ right_front_wheel
                 |→ left_wheel
                 |→ right_wheel
```

---

## Key Parameters

### drive_node

| Parameter | Default | Description |
|---|---|---|
| `can_interface` | `can0` | CAN bus interface |
| `left_front_id` | 2 | SparkFlex CAN ID |
| `right_front_id` | 1 | SparkFlex CAN ID |
| `left_rear_id` | 3 | SparkFlex CAN ID |
| `right_rear_id` | 4 | SparkFlex CAN ID |
| `wheel_base` | 0.762 m | Left–right wheel distance |
| `wheel_radius` | 0.1778 m | Wheel radius |
| `gear_ratio` | 100.0 | Motor-to-wheel ratio |
| `max_duty_cycle` | 0.8 | Max motor output |
| `publish_odom_tf` | true | Set false when EKF runs |

### Controller Mapping

- **Left Stick Y**: Forward/Backward
- **Right Stick X**: Turn
- **R Button** (hold): Enable motors
- **L Button** (hold): Turbo mode

---

## Hardware Reference

See `PI5_PIN_ASSIGNMENTS.md` for all GPIO, USB, and power wiring.
See `BUCKET_ACTUATOR_SOFTWARE_SPEC.md` for actuator hardware details and ROS interface.

---

## Troubleshooting

**Motors not responding:**
```bash
ip link show can0          # Check CAN interface is up
# Hold R button to enable teleop
```

**No pointcloud:**
```bash
lsusb -t | grep 5000M     # Camera must be on USB 3.0
ros2 topic hz /oak/points  # Should be ~10 Hz
```

**TF errors:**
```bash
ros2 run tf2_tools view_frames
```

**No /joy topic:**
```bash
cat /proc/bus/input/devices | grep "Pro Controller"
# Check ROS_DOMAIN_ID matches on both machines
```

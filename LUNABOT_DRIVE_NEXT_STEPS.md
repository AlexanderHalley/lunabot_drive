# Lunabot_drive — Next Steps for Claude Code

> **Context:** This is the `lunabot_drive` ROS2 Jazzy package for Northwestern's NASA Lunabotics 2025-2026 team. The robot runs on a Raspberry Pi 5, uses SparkFlex motor controllers via CAN bus, and an OAK-D S2 depth camera connected over USB-C. Competition is May 2026 at Kennedy Space Center. The arena is 6.8m × 5.0m with BP-1 regolith simulant. There is a strict 4 Mbps average bandwidth limit — lower usage scores more points (formula: `TAB * -30 + 120`, max 120 pts). The robot must stay under 80 kg. GPS, compass, and ultrasonic sensors are prohibited.

---

## PRIORITY 1: Fix OAK-D S2 Camera Pointcloud Configuration

The camera is physically connected and the `depthai_ros_driver` node runs, but the pointcloud configuration has had recurring issues: namespace mismatches, wrong parameter names, and excessive point counts (650k+ per frame). The goal is a **working, bandwidth-efficient pointcloud** suitable for Nav2 obstacle avoidance on a Pi 5.

### 1.1 Determine the Correct Node Name and Namespace

The depthai_ros_driver creates a node whose name depends on how it's launched. The YAML config namespace **must match** the running node's name.

```bash
# Step 1: Launch the camera with the default pointcloud launch file
ros2 launch depthai_ros_driver pointcloud.launch.py

# Step 2: Check what node name is running
ros2 node list
# Expected output might be: /oak or /oak_d

# Step 3: Check what parameters it actually loaded
ros2 param list /oak  # (or whatever node name appeared)

# Step 4: Check if pointcloud topic has publishers
ros2 topic info /oak/points
# MUST show "Publisher count: 1" — if 0, the pointcloud node isn't enabled
```

**The YAML namespace must match the node name exactly.** If the node is `/oak`, use `/oak:`. If it's `/oak_d` (because `name='oak_d'` was set in a custom launch file), use `/oak_d:`. Using `/**:` as the namespace is unreliable with this driver.

### 1.2 Write the Correct YAML Configuration

Based on the **official depthai_ros_driver parameter documentation** (https://docs.luxonis.com/software-v3/depthai/ros/parameters/), here are the **verified correct parameter names**. Many parameters we tried before used wrong names — these are from the official docs:

Create/replace `config/oak_d_pointcloud_only.yaml`:

```yaml
# OAK-D-S2 — Pointcloud-Only Configuration (Bandwidth Optimized)
# IMPORTANT: Change the top-level namespace to match your node name.
# Run `ros2 node list` after launching to confirm — use /oak: or /oak_d: accordingly.
/oak:
  ros__parameters:
    # ── Driver-level ──
    i_usb_speed: 'SUPER'          # Must be on a USB 3.0 (blue) port
    i_enable_ir: false

    # ── Pipeline ──
    pipeline_gen:
      i_pipeline_type: 'Depth'    # Depth-only pipeline — no RGB overhead
      i_nn_type: 'none'
      i_enable_imu: true

    # ── Left & Right Sensors (control stereo input resolution) ──
    left:
      i_publish_topic: false       # Don't publish raw left image
      i_width: 640
      i_height: 400
      i_fps: 10.0

    right:
      i_publish_topic: false       # Don't publish raw right image
      i_width: 640
      i_height: 400
      i_fps: 10.0

    # ── Stereo Depth Node ──
    stereo:
      i_publish_topic: true        # MUST be true — pointcloud reads from stereo depth output
      i_fps: 10.0
      i_depth_preset: 'FAST_ACCURACY'  # Options: FAST_ACCURACY, DEFAULT, HIGH_DETAIL, ROBOTICS
      i_subpixel: false            # Saves bandwidth and compute
      i_extended_disp: false
      i_lrc_threshold: 10
      i_stereo_conf_threshold: 200 # Higher = fewer but more confident depth values

      # Decimation filter — KEY bandwidth saver, reduces resolution by factor in each axis
      i_enable_decimation_filter: true
      i_decimation_filter_decimation_factor: 2         # 2 = quarter the points (2x each axis)
      i_decimation_filter_decimation_mode: 'PIXEL_SKIPPING'

      # Spatial filter — smooths depth, removes noise
      i_enable_spatial_filter: true
      i_spatial_filter_alpha: 0.5
      i_spatial_filter_delta: 20
      i_spatial_filter_num_iterations: 1
      i_spatial_filter_hole_filling_radius: 2

      # Temporal filter — reduces flickering, slight latency tradeoff
      i_enable_temporal_filter: true
      i_temporal_filter_alpha: 0.4
      i_temporal_filter_delta: 20
      i_temporal_filter_persistency: 'VALID_2_IN_LAST_4'

      # Speckle filter — removes isolated noise points
      i_enable_speckle_filter: true
      i_speckle_filter_speckle_range: 12   # Hardware max confirmed from prior testing

      # Threshold filter — limits range (values in MILLIMETERS)
      i_enable_threshold_filter: true
      i_threshold_filter_min_range: 200    # 0.2m — ignore anything closer
      i_threshold_filter_max_range: 5000   # 5.0m — arena diagonal is ~8.4m, 5m is plenty

    # ── IMU ──
    imu:
      i_imu_update_rate: 100       # Reduced from 400 to save bandwidth
```

### 1.3 Verify USB 3.0 Connection

The camera **must** be on a USB 3.0 port. On Pi 5, the blue ports are 3.0. Many USB-C cables are 2.0-only (especially phone charging cables).

```bash
# Check USB speed
lsusb -t | grep -A2 "5000M"
# Camera (Movidius MyriadX, ID 03e7:2485) should be under a 5000M hub

# If it shows up under 480M, try:
# 1. Move to a blue USB 3.0 port
# 2. Use a known USB 3.0 cable (data cable, not charge-only)
# 3. Try a powered USB hub if the Pi can't supply enough current
```

### 1.4 Launch and Validate

```bash
# Option A: Use depthai's built-in pointcloud launch (recommended first test)
ros2 launch depthai_ros_driver pointcloud.launch.py \
    params_file:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml

# Option B: Use the custom launch file in lunabot_drive
ros2 launch lunabot_drive oak_d_camera.launch.py
```

**Validation checklist:**

```bash
# 1. Confirm the pointcloud topic has a publisher
ros2 topic info /oak/points
# Should show: Publisher count: 1

# 2. Check pointcloud rate
ros2 topic hz /oak/points
# Should be ~10 Hz (matching i_fps)

# 3. Check point count per message
ros2 topic echo /oak/points --field width --once
# With 640x400 + decimation factor 2: expect ~320x200 = ~64,000 points
# With threshold/speckle filtering: likely 30,000–50,000 valid points

# 4. Check bandwidth
ros2 topic bw /oak/points
# Target: < 1.5 Mbps for pointcloud alone
# Total system target: < 2 Mbps to maximize bandwidth score

# 5. Visualize in RViz2 (on offboard computer)
ros2 run rviz2 rviz2
# Add PointCloud2 display, set topic to /oak/points
# Set fixed frame to "oak" or whatever the camera TF frame is
```

### 1.5 Update the Custom Launch File

The current `launch/oak_d_camera.launch.py` uses `name='oak_d'` which creates a namespace mismatch with configs targeting `/oak:`. Either:

**Option A** — Change the launch file node name to match the YAML:
```python
Node(
    package='depthai_ros_driver',
    executable='camera',
    name='oak',  # Changed from 'oak_d' to match YAML namespace
    ...
)
```

**Option B** — Change the YAML top-level namespace to `/oak_d:` to match the launch file.

Pick one and be consistent. Option A (using `oak`) is recommended since it matches depthai defaults.

### 1.6 If Pointcloud Still Has 0 Publishers

If `ros2 topic info /oak/points` still shows 0 publishers after the config fix:

```bash
# Check if the pointcloud parameter was actually loaded
ros2 param get /oak pointcloud.i_enable
# Should return: Boolean value is: True

# If it returns an error or False, the YAML section wasn't loaded.
# Verify YAML indentation — pointcloud: must be at the same level as stereo:, left:, etc.

# Try setting it at runtime as a diagnostic test:
ros2 param set /oak pointcloud.i_enable true
# Then restart the node — some i_ params only apply at init
```

### 1.7 Bandwidth Tuning Knobs (Ordered by Impact)

If bandwidth is still too high after the base config:

| Knob | Current | Aggressive | Bandwidth Impact |
|------|---------|------------|------------------|
| `i_decimation_filter_decimation_factor` | 2 | 3 or 4 | ~4x → ~9x or ~16x fewer points |
| `left/right i_fps` + `stereo i_fps` | 10 | 5 | 2x reduction |
| `left/right i_width × i_height` | 640×400 | 480×300 | ~1.8x fewer points |
| `i_threshold_filter_max_range` | 5000 | 3000 | Removes distant points |
| `i_stereo_conf_threshold` | 200 | 230 | More aggressive filtering |
| `stereo i_low_bandwidth` | false | true | MJPEG-encodes depth before USB transfer |

---

## PRIORITY 2: Integrate Pointcloud into Nav2 Local Costmap

Once the pointcloud is publishing reliably, feed it into Nav2 for obstacle avoidance.

### 2.1 Add Pointcloud as a Costmap Layer

In `config/params/nav2_params.yaml`, add or update the local costmap observation source:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleCostmapPlugin"
        enabled: true
        observation_sources: camera_pointcloud
        camera_pointcloud:
          topic: /oak/points
          data_type: "PointCloud2"
          sensor_model: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.05    # Ignore ground-level noise
          max_obstacle_height: 0.5     # Ignore points above robot height
          obstacle_max_range: 4.0      # Match threshold filter
          obstacle_min_range: 0.2      # Match threshold filter
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
```

### 2.2 Verify TF Tree

Nav2 needs a complete TF chain: `map → odom → base_link → camera_link → camera_link_optical`

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# The depthai driver publishes its own TF frames (oak, oak_left_camera, etc.)
# You may need to add a static transform connecting depthai's frames to your robot's base_link:
ros2 run tf2_ros static_transform_publisher \
    --x 0.9652 --y 0 --z 0.069 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id oak
```

Consider adding this as a node in the launch file rather than running it manually.

---

## PRIORITY 3: IMU Integration from OAK-D S2

The OAK-D S2 has an onboard BNO086 IMU. This is the **only IMU on the system** (no separate IMU module). It should be fused with wheel odometry via `robot_localization` EKF.

### 3.1 Verify IMU Data

```bash
# Check IMU topic
ros2 topic echo /oak/imu/data --once

# Check IMU rate
ros2 topic hz /oak/imu/data
# Should be ~100 Hz with our config
```

### 3.2 Competition Rule: Compass Must Be Disabled

Per the Lunabotics rules (Section 4, Rule 7): "Teams have to explain to the judges how the compass feature will be switched off, or the compass data is subtracted to ensure the internal calculations do not make use of the compass."

The BNO086 has a magnetometer. In the depthai_ros_driver, the default IMU message type does NOT include magnetometer data. Verify:

```bash
# The driver parameter i_enable_rotation controls magnetometer
# Make sure this is false (or not set — default is false)
ros2 param get /oak imu.i_enable_rotation
# Should be: False
```

If for some reason magnetometer data appears in the IMU message, configure robot_localization to ignore it (don't fuse any magnetometer axes).

### 3.3 EKF Configuration

The existing `config/params/nav2_params.yaml` already has an EKF config. Ensure the IMU topic matches:

```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /oak/imu/data    # Update if the actual topic differs
    # Only fuse yaw orientation and yaw rate from IMU
    imu0_config: [false, false, false,   # x, y, z position
                  false, false, true,    # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  false, false, true,    # roll_rate, pitch_rate, yaw_rate
                  true,  false, false]   # ax, ay, az (linear accel)
    imu0_remove_gravitational_acceleration: true
```

---

## PRIORITY 4: Create a Unified Hardware Bringup Launch File as well as individual launch files for testing (if they don't exist yet)

Individual test launch files should be made to test the camera individually (including inital attempts at mapping), and another for motor controls which give feedback and can be seen in rviz.
Also create a file to detect apriltags and output which tag has been detected.
Currently there's no single launch file that brings up all hardware. Create `launch/hardware_bringup.launch.py`:

```python
# This launch file should start:
# 1. SparkFlex CAN motor driver node (drive_node)
# 2. OAK-D S2 camera with pointcloud config
# 3. Static TF from base_link to camera
# 4. robot_localization EKF (fuses wheel odom + camera IMU)
# 5. Joystick teleop (for manual control testing)
#
# Usage:
#   ros2 launch lunabot_drive hardware_bringup.launch.py
#   ros2 launch lunabot_drive hardware_bringup.launch.py enable_teleop:=false
```

### Files to create:
- `launch/hardware_bringup.launch.py` — orchestrates all hardware nodes
- Update `config/oak_d_pointcloud_only.yaml` — corrected camera config (from Priority 1)

---

## PRIORITY 5: Wheel Odometry Calibration

The drive node publishes `/odom` from SparkFlex encoder feedback. This needs calibration.

### 5.1 Indoor Calibration Protocol

```bash
# 1. Launch hardware
ros2 launch lunabot_drive hardware_bringup.launch.py

# 2. Record a bag while driving a known pattern
ros2 bag record /odom /joint_states /cmd_vel -o calibration_test

# 3. Drive forward exactly 1 meter (measure with tape measure)
# 4. Check what odom reports:
ros2 topic echo /odom --field pose.pose.position.x --once

# 5. Adjust wheel_radius in drive_node parameters until odom matches reality
# 6. Repeat for rotation: execute a 360° turn, check if odom theta ≈ 2π
#    Adjust wheel_separation if not
```

Key parameters to calibrate in the drive node:
- `wheel_radius` — affects linear distance
- `wheel_separation` — affects turning radius
- Both of these should be in the URDF and/or as node parameters

---

## PRIORITY 6: Improve URDF for the Robot

The repo currently has a basic URDF. This is needed for proper TF tree, RViz visualization.

### 6.1 Minimum Viable URDF

Create `description/robot.urdf.xacro` with:
- `base_link` — robot center
- `chassis` — visual mesh or box approximation
- `left_wheel` and `right_wheel` — with correct radius and separation
- `camera_link` → `camera_link_optical` — at the correct position on the chassis

The existing `description/depth_camera.xacro` has camera placement at `xyz="0.9652 0 0.069"` — verify this matches the physical robot.

---

## PRIORITY 7: Prepare Autonomous Behavior Foundation

These are further out but should be planned now.

### 7.1 Beacon-Based Localization

GPS is prohibited. The rules allow beacons/fiducials on the arena frame along the starting zone perimeter (2 sides). Plan for:
- AprilTag or ArUco fiducials mounted on the bin frame
- Camera-based detection using the OAK-D S2 RGB stream (enable RGB only during localization phases)
- Triangulation from multiple beacon observations

### 7.2 Autonomous Behavior State Machine

Competition scoring tiers:
- **Excavation Automation (75 pts):** Hands-free excavation in the excavation zone
- **Dump Automation (50 pts):** Hands-free dumping in the construction zone
- **Travel Automation (250 pts):** Hands-free obstacle field crossing (excavation → construction zone)
- **Full Autonomy One Cycle (450 pts):** Complete cycle hands-free
- **Full Autonomy Two+ Cycles (600 pts):** Two or more complete cycles, hands-free from start

Obstacle contact in the obstacle zone costs 30 pts each (max 90 pt reduction). Travel automation after a remote-control traverse costs 50 pts ("breadcrumb" penalty).

### 7.3 Key Rule Constraints for Autonomy Software

- Cannot use arena walls for navigation/collision avoidance
- Cannot update autonomy program to account for obstacle locations mid-run
- Telemetry viewing is allowed but no control input during autonomous periods

---

## File Structure Target

```
lunabot_drive/
├── config/
│   ├── oak_d_pointcloud_only.yaml     # UPDATED — correct parameter names
│   ├── oak_d_camera.yaml              # EXISTING — general camera config
│   └── params/
│       ├── nav2_params.yaml           # UPDATE — add pointcloud costmap source
│       └── mapper_params_with_depth.yaml
├── description/
│   ├── robot.urdf.xacro               # NEW — full robot URDF
│   └── depth_camera.xacro             # EXISTING
├── launch/
│   ├── hardware_bringup.launch.py     # NEW — unified hardware launch
│   ├── oak_d_camera.launch.py         # UPDATE — fix node name
├── src/
│   └── drive_node.cpp                 # EXISTING — SparkFlex motor control
├── nodes/
│   ├── depth_obstacle_detector.py     # NEW — depth image to costmap (backup to pointcloud)
│   └── bandwidth_monitor.py           # NEW — monitor total bandwidth for competition
├── docs/
│   └── OAK_D_S2_INTEGRATION.md       # EXISTING
├── package.xml
└── CMakeLists.txt
```


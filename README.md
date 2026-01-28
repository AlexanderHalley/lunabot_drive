# Lunabot Drive Package

Motor drive control for Lunabot rover using SparkFlex controllers via CAN bus, with wheel odometry and EKF sensor fusion.
UPDATE WHEEL DIAMETER AND HALL SENSOR COUNT ETC
## Package Structure

```
lunabot_drive/
├── src/
│   └── drive_node.cpp          # Main motor control node + wheel odometry
├── launch/
│   ├── pc_teleop.launch.py     # Launch file for offboard PC (joy + teleop)
│   ├── pi_drive.launch.py      # Launch file for Raspberry Pi5 (motor control + EKF)
│   └── oak_d_camera.launch.py  # Launch file for OAK-D-S2 camera
├── config/
│   ├── ekf.yaml                # robot_localization EKF config (wheel odom + IMU fusion)
│   ├── oak_d_camera.yaml       # OAK-D-S2 camera parameters
│   └── switch_pro.yaml         # Nintendo Switch Pro Controller configuration
├── docs/
│   └── OAK_D_S2_INTEGRATION.md # OAK-D-S2 camera integration guide
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

## Usage

### On Offboard Computer (where controller is connected)

1. Connect Nintendo Switch Pro Controller via Bluetooth
2. Set ROS_DOMAIN_ID (must match Pi5):
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. Launch joy and teleop nodes:
   ```bash
   ros2 launch lunabot_drive pc_teleop.launch.py
   ```

### On Raspberry Pi5 (rover)

1. Ensure CAN interface is up:
   ```bash
   sudo slcand -o -s8 /dev/ttyACM0
   sudo ip link set can0 up type can
   ```
2. Set ROS_DOMAIN_ID (must match PC):
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. Launch drive node:
   ```bash
   ros2 launch lunabot_drive pi_drive.launch.py
   ```

## Odometry and Sensor Fusion

The drive node publishes wheel odometry on `/wheel_odom` using encoder feedback from the front motors. An EKF node from `robot_localization` fuses wheel odometry with IMU data from the OAK-D S2 camera (BNO086) and broadcasts the `odom -> base_link` TF transform.

### Verifying Odometry

```bash
# Check wheel odom publishing
ros2 topic echo /wheel_odom

# Check IMU data (with camera launched)
ros2 topic echo /oak/imu/data

# Check EKF fused output
ros2 topic echo /odometry/filtered

# Check TF broadcast
ros2 run tf2_ros tf2_echo odom base_link
```

## Controller Mapping

- **Left Stick Y**: Forward/Backward
- **Right Stick X**: Turn Left/Right
- **R Button (hold)**: Enable motor control (safety feature)
- **L Button (hold)**: Turbo mode (faster speeds)

## Parameters (drive_node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | CAN bus interface name |
| `left_front_id` | `2` | CAN ID for left front motor |
| `right_front_id` | `1` | CAN ID for right front motor |
| `left_rear_id` | `3` | CAN ID for left rear motor |
| `right_rear_id` | `4` | CAN ID for right rear motor |
| `wheel_base` | `0.5` | Distance between left/right wheels (meters) |
| `max_duty_cycle` | `0.5` | Maximum motor duty cycle (0.0-1.0) |
| `wheel_radius` | `0.075` | Wheel radius in meters |
| `encoder_cpr` | `42` | Hall sensor counts per revolution (NEO Vortex) |
| `gear_ratio` | `100.0` | Motor gear ratio |
| `odom_rate` | `30.0` | Odometry publish rate in Hz |
| `odom_frame_id` | `odom` | Odometry frame ID |
| `odom_child_frame_id` | `base_link` | Odometry child frame ID |

## Safety Features

- **Watchdog timer**: Motors stop if no command received for 500ms
- **Enable button**: Must hold R button to control motors
- **Speed limiting**: Commands clamped to max_duty_cycle

## Network Setup

Both computers must be on the same network and use the same ROS_DOMAIN_ID.

To set permanently, add to `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=42
```

## Troubleshooting

**No /joy topic:**
- Check controller is connected: `cat /proc/bus/input/devices | grep "Pro Controller"`
- Verify ROS_DOMAIN_ID matches on both machines
- Check joy_node is running: `ros2 node list`

**Motors not responding:**
- Check CAN interface: `ip link show can0`
- Verify CAN IDs match motor configuration
- Check drive_node logs: `ros2 node list` and look for errors
- Ensure you're holding the R button (enable button)

**Wrong controller axis:**
- Test controller: `ros2 topic echo /joy`
- Adjust axis mappings in `config/switch_pro.yaml`

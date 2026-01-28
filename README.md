# Lunabot Drive Package

Motor drive control for Lunabot rover using SparkFlex controllers via CAN bus.

## Package Structure

```
lunabot_drive/
├── src/
│   └── drive_node.cpp          # Main motor control node
├── launch/
│   ├── pc_teleop.launch.py     # Launch file for offboard PC (joy + teleop)
│   └── pi_drive.launch.py      # Launch file for Raspberry Pi5 (motor control)
├── config/
│   └── switch_pro.yaml         # Nintendo Switch Pro Controller configuration
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
sudo slcand -o -s8 /dev/ttyACM0
1. Ensure CAN interface is up:
   ```bash
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

## Controller Mapping

- **Left Stick Y**: Forward/Backward
- **Right Stick X**: Turn Left/Right
- **R Button (hold)**: Enable motor control (safety feature)
- **L Button (hold)**: Turbo mode (faster speeds)

## Parameters (drive_node)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | CAN bus interface name |
| `left_front_id` | `1` | CAN ID for left front motor |
| `right_front_id` | `2` | CAN ID for right front motor |
| `left_rear_id` | `3` | CAN ID for left rear motor |
| `right_rear_id` | `4` | CAN ID for right rear motor |
| `wheel_base` | `0.5` | Distance between left/right wheels (meters) |
| `max_duty_cycle` | `0.8` | Maximum motor duty cycle (0.0-1.0) |

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

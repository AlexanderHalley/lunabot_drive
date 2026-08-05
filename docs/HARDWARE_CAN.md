# Drivetrain: SparkFlex over SocketCAN

Four REV SparkFlex controllers, one per wheel, on a single CAN bus. Driven by
`lunabot_hardware/SparkFlexSystem`, a `ros2_control` hardware interface.

---

## Read this first: the odometry is known-bad

The drivetrain has **no encoders**. `SparkFlexSystem` therefore runs with `use_motor_feedback:
false`, in which `read()` echoes the commanded velocity back as though it had been measured, and
integrates it to get position.

The `/odom` this produces has the right frames, the right units and the right topology. It is also
**dead reckoning from command, not from measurement**, and it is wrong the instant a wheel slips.
On regolith that is continuous. The rover can sit spinning its wheels in place while `/odom`
confidently reports a straight line at 0.3 m/s.

This is stated loudly rather than buried because three things follow from it:

1. **Do not tune anything against `/odom` accuracy** until there is real feedback. You would be
   tuning a number that describes the command, not the robot.
2. **`robot_localization` ships in the skeleton, disabled** (`odom_source:=ekf`). Wheel odometry is
   known-bad *before* the first field test, so the fusion path needs to be wired and exercised in
   sim now rather than discovered in February.
3. **SLAM carries the load.** `map → odom` from rtabmap is what makes the pose usable; `odom →
   base_link` is a smoothing term, not a measurement.

### Turning it on when encoders exist

`SparkFlexMotor::read_velocity()` returns NaN and `on_activate` **refuses to start** with
`use_motor_feedback: true`. That refusal is deliberate — NaN state would propagate into
`diff_drive_controller`, out into `/odom`, and into `/tf`, and NaN in a transform poisons the whole
tree in a way that is very hard to trace back to a motor driver.

To wire it up, follow the numbered comment in `src/spark_flex_motor.cpp`. The four things that
matter: confirm the getter name against your `sparkcan` build, confirm its units (usually motor
RPM, not rad/s), divide by `gear_ratio` to get wheel velocity, and **check the sign against
`SetInverted`** — if the controller reports pre-inversion velocity, the right side reads backwards
and the rover appears to drive in circles.

---

## Bring-up

### CAN interface

```bash
# USB-to-CAN adapter (the 2026 setup)
sudo slcand -o -s8 /dev/ttyACM0 can0
sudo ip link set can0 up type can

# Native CAN peripheral
sudo ip link set can0 up type can bitrate 1000000

ip -details link show can0     # confirm state UP
```

`-s8` is 1 Mbit/s, which is what the SparkFlex controllers expect.

### Testing with no motors attached

This is how the plugin gets exercised without the robot, and it is worth doing before every field
session:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ros2 launch lunabot_bringup robot.launch.py hw:=real can_interface:=vcan0
```

In another terminal:

```bash
candump vcan0
```

You should see heartbeat frames at roughly 100 Hz with no command traffic, and additional frames
appear when you publish to `/cmd_vel`. If `candump` is silent, the component never activated —
check the `controller_manager` log, not the wiring.

---

## Configuration

All of it lives in `lunabot_description/urdf/ros2_control/lunabot.ros2_control.xacro`, sourced from
`urdf/common/properties.xacro`. There are no CAN parameters in any launch file, which is a change
from 2026 — the old `pi_drive.launch.py` hardcoded IDs that disagreed with `drive_node.cpp`'s own
defaults *and* with the README table.

| Parameter | Default | Notes |
|---|---|---|
| `can_interface` | `can0` | `vcan0` for bench testing |
| `use_motor_feedback` | `false` | See above. Activation refuses if true |
| `max_wheel_rad_s` | 16.0 | Wheel rad/s at full duty. Sets the whole command scale |
| `max_duty_cycle` | 0.8 | Carried over from 2026 |
| `gear_ratio` | 20.0 | Motor revs per wheel rev |
| `ramp_rate` | 0.1 | Seconds 0→full. Carried over from 2026 |

### Wheel wiring

| Joint | CAN ID | Inverted |
|---|---|---|
| `front_left_wheel_joint` | 2 | no |
| `front_right_wheel_joint` | 1 | yes |
| `rear_left_wheel_joint` | 3 | no |
| `rear_right_wheel_joint` | 4 | yes |

Front-left is **2** and front-right is **1**. That looks like a typo and is not: it is what
`pi_drive.launch.py` actually ran after the 2026 commit *"Updated pi wheel allocation for proper
turning"*. `drive_node.cpp`'s defaults said the opposite and were never the values in use.

Inversion is a per-joint URDF parameter. On the 2026 robot it was `right_front_->SetInverted(true)`
in C++, so rewiring the robot meant recompiling it.

---

## Design notes

### The heartbeat comes for free

SparkFlex controllers fault out without a keep-alive roughly every 50 ms. `SparkFlexSystem` sends
one from `write()`, which `controller_manager` calls every 10 ms at `update_rate: 100` — whenever
the component is **active**, independent of whether any controller is running.

That is strictly better than the 2026 arrangement of an independent 50 ms wall timer, which could
drift out of phase with the control loop and had no relationship to whether commands were flowing.

**Consequence:** if `update_rate` in `controllers.yaml` ever drops below about 40 Hz, the motors
fault. That parameter is not free to tune.

### Ported settings, and why they are not negotiable

```cpp
SetIdleMode(IdleMode::kBrake);        // a coasting rover on a slope keeps going;
                                      // brake mode is also what makes a watchdog stop mean anything
SetMotorType(MotorType::kBrushless);  // } what the drivetrain physically is; wrong values
SetSensorType(SensorType::kHallSensor); // } give a motor that stutters or does not turn
SetRampRate(0.1);                     // a step command breaks traction, and a rover that has
                                      // broken traction has fictional odometry
```

**Nothing is burned to flash.** `configure()` runs on every activation and flash has finite write
endurance. Persist these once at the bench, never from the control loop. The 2026 code carried the
same warning.

### Lifecycle, and why the bus opens in `on_configure`

| Callback | Does |
|---|---|
| `on_init` | Parses `HardwareInfo`. **Touches no hardware** — this runs during URDF parsing, so opening a socket here would make `ros2 control list_hardware_components` a side-effecting command |
| `on_configure` | Opens SocketCAN. A down interface becomes a clean transition failure that `controller_manager` reports |
| `on_activate` | Applies the settings above, zeroes state |
| `read` | Feedback or command-echo, integrates position |
| `write` | Duty cycle to all four, then heartbeat |
| `on_deactivate` | Zeroes duty, keeps brake, sends one final heartbeat so the controllers see a deliberate stop rather than a dead bus |

The 2026 node opened CAN in its constructor and threw on failure, producing a stack trace that
never mentioned CAN.

### The command watchdog moved up a layer

`diff_drive_controller`'s `cmd_vel_timeout: 0.5` replaces the 2026 node's hand-rolled 500 ms
watchdog. Same timeout, but now the controller commands zero velocity and the hardware layer just
obeys — so the stop is visible in `/odom` and in the controller's own state rather than happening
silently below it.

---

## Not implemented yet

`/drive/status` (`lunabot_msgs/DriveStatus`) appears in
[`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md) but **nothing publishes it**. The messages
exist; the broadcaster does not. Publishing it needs either a `controller_interface` broadcaster or
a node subscribing to the hardware's state interfaces, and most of the fields it carries (bus
voltage, current, temperature, faults) are unavailable until motor telemetry is read at all — so it
is blocked behind the same work as `use_motor_feedback`.

---

## Troubleshooting

**`candump` silent, no errors in the log.** The component parsed but never activated. Check
`ros2 control list_hardware_components` — it should show `LunabotSystem` as `active`.

**"failed to open SparkFlex id N".** The interface is down, or another process holds it. `ip link
show can0`; check for a stale `slcand`.

**Motors fault after a few seconds.** Heartbeat starvation. Check `update_rate` in
`controllers.yaml` is still 100, and that the component is `active` rather than merely `configured`
— `write()` is only called while active.

**One wheel does not respond.** Check for duplicate CAN IDs first. The plugin rejects duplicates at
parse time, so if it activated, the IDs are unique in the URDF — which points at the physical
controller's configured ID instead.

**Rover turns the wrong way, or one side fights the other.** Inversion applied twice. It belongs in
the URDF `invert` parameter, which reaches the controller via `SetInverted()`; the duty cycle
written in `set_velocity()` must not be negated again.

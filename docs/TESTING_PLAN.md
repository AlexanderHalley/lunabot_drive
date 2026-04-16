# Lunabot Drive — Testing Plan

**Machines**
- `lunapi` — Raspberry Pi 5 (robot onboard computer)
- `dreamfyre` — MCC laptop (offboard operator PC)

**Both machines must have in `~/.bashrc`:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

All xterm commands below are run on **dreamfyre** unless stated otherwise.
The `-T` flag sets the window title; `-hold` keeps the window open if the
command exits so you can read the last output.

---

## Test 1 — GUI + Software Stack (no hardware)

**Goal:** verify the rosbridge → dashboard → actuator mux pipeline end-to-end
without any hardware attached. No CAN bus, no actuators, no motors needed.

**Hardware required:** lunapi powered on, reachable on LAN (`ping lunapi`).

### Launch

```bash
# Pi side — software stack only (no drive_node)
xterm -T "lunapi — Pi stack" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py enable_drive:=false\"'" &

# Open dashboard in browser
chromium-browser "file://$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/dashboard/dashboard.html" &
```

### Monitoring windows

```bash
xterm -T "monitor: lift driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/lift/lift_driver/command'" &

xterm -T "monitor: tilt driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/tilt/tilt_driver/command'" &

xterm -T "monitor: autonomy state" -hold \
  -e "bash -l -c 'ros2 topic echo /autonomy_state'" &
```

### Pass criteria

| Action | Expected |
|--------|----------|
| Dashboard loads | State banner shows **TELEOP** in grey |
| Hold **Lift ▼ Extend** button | `/bucket/lift/lift_driver/command` publishes `data: 1.0` at ~10 Hz |
| Release button | One `data: 0.0` then silence |
| Hold **Tilt ▶ Extend** button | `/bucket/tilt/tilt_driver/command` publishes `data: 1.0` |
| Click **ARM AUTONOMY** | Banner transitions to **READY** (blue) |
| Click **ABORT** | Banner returns to **TELEOP** |
| Click **⚠ E-STOP** | Banner flashes red **E-STOP ACTIVE** |
| Click **ACKNOWLEDGE** | Banner returns to **TELEOP** |

---

## Test 2 — Actuator Control (GUI + joystick, no drive motors)

**Goal:** physically move the lift and tilt actuators via both the dashboard
GUI and the Switch Pro Controller d-pad. Verify priority (GUI wins over d-pad).

**Hardware required:** lunapi with actuators wired to GPIO. No CAN/motors needed.

### Launch

```bash
# Pi side — software stack only (enable_drive:=false)
xterm -T "lunapi — Pi stack" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py enable_drive:=false\"'" &

# PC side — joystick teleop
xterm -T "dreamfyre — PC teleop" \
  -e "bash -l -c 'ros2 launch lunabot_drive proof_of_life_bringup_pc.launch.py'" &

# Dashboard
chromium-browser "file://$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/dashboard/dashboard.html" &
```

### Monitoring windows

```bash
xterm -T "monitor: lift driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/lift/lift_driver/command'" &

xterm -T "monitor: tilt driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/tilt/tilt_driver/command'" &

xterm -T "monitor: lift position" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/lift_position'" &

xterm -T "monitor: tilt position" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/tilt_position'" &
```

### Pass criteria

| Action | Expected |
|--------|----------|
| D-pad UP | Lift physically retracts; `lift_driver/command` shows `-1.0` |
| D-pad DOWN | Lift physically extends; `lift_driver/command` shows `+1.0` |
| D-pad RIGHT | Tilt extends |
| D-pad LEFT | Tilt retracts |
| Hold d-pad UP **and** GUI **▼ Extend** simultaneously | GUI wins (priority 10 > 5) — lift extends despite d-pad up |
| Release GUI button while d-pad still held | D-pad takes over after GUI timeout (~0.5 s) |
| `/bucket/lift_position` | Rises towards `1.0` while extending; falls while retracting |

---

## Test 3 — Full Drive + Actuators

**Goal:** drive the robot with the joystick and control actuators simultaneously.
Verify `/cmd_vel` → `drive_node` → `/odom` pipeline and that the drive and
actuator stacks do not interfere.

**Hardware required:** CAN bus up, all four SparkFlex motors connected, actuators wired.

### One-time CAN initialisation (lunapi, run on every boot)

```bash
xterm -T "lunapi — CAN init" \
  -e "ssh -t lunapi 'bash -l -c \"cd ~/ros2_ws/src/lunabot_drive && sudo ./scripts/initialise_can\"'" &
```

Verify CAN is up before proceeding:
```bash
ssh lunapi 'ip link show can0'
# Expected: state UP
```

### Launch

```bash
# Pi side — full hardware stack
xterm -T "lunapi — Pi stack" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py\"'" &

# PC side — joystick teleop
xterm -T "dreamfyre — PC teleop" \
  -e "bash -l -c 'ros2 launch lunabot_drive proof_of_life_bringup_pc.launch.py'" &

# Dashboard
chromium-browser "file://$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/dashboard/dashboard.html" &
```

### Monitoring windows

```bash
xterm -T "monitor: /cmd_vel" -hold \
  -e "bash -l -c 'ros2 topic echo /cmd_vel'" &

xterm -T "monitor: /odom" -hold \
  -e "bash -l -c 'ros2 topic echo /odom --no-arr'" &

xterm -T "monitor: node health" -hold \
  -e "bash -l -c 'ros2 topic echo /node_health'" &

xterm -T "monitor: CAN status" -hold \
  -e "bash -l -c 'ros2 topic echo /can_status'" &
```

### Pass criteria

| Action | Expected |
|--------|----------|
| Hold R button + left stick forward | `/cmd_vel` shows positive `linear.x`; robot drives forward |
| Left stick back | Robot reverses; `/odom` twist shows negative `linear.x` |
| Right stick left/right | Robot turns; `/odom` shows `angular.z` |
| D-pad UP/DOWN | Lift retracts/extends independently of drive |
| D-pad LEFT/RIGHT | Tilt retracts/extends independently of drive |
| GUI drive arrows | Robot responds; `/cmd_vel` shows values (GUI priority 10 > joystick 5) |
| Node health panel | `drive_node` dot goes green |
| **⚠ E-STOP** button | All motion stops; banner flashes red |

---

## Test 4 — Autonomy

**Goal:** run a full excavation and deposition sequence end-to-end under Nav2.

**Hardware required:** full Test 3 hardware plus OAK-D S2 camera and AprilTag
markers at known positions in the arena.

**Pre-flight (complete before launch):**
- [ ] Measure AprilTag centre positions; update `config/params/apriltag_localizer_params.yaml`
- [ ] Drive robot to dig zone, read `/odometry/filtered` pose, set `dig_x` / `dig_y`
- [ ] Drive robot to hopper, set `hopper_x` / `hopper_y`
- [ ] Drive robot to home pose, set `home_x` / `home_y`

### CAN initialisation

```bash
xterm -T "lunapi — CAN init" \
  -e "ssh -t lunapi 'bash -l -c \"cd ~/ros2_ws/src/lunabot_drive && sudo ./scripts/initialise_can\"'" &
```

### Launch

```bash
# Pi side — full autonomy stack (adjust waypoints as measured)
xterm -T "lunapi — autonomy stack" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive autonomy_bringup_pi.launch.py \
      dig_x:=4.0 dig_y:=2.5 hopper_x:=0.5 hopper_y:=2.5 home_x:=0.3 home_y:=2.5\"'" &

# PC side — joystick teleop
xterm -T "dreamfyre — PC teleop" \
  -e "bash -l -c 'ros2 launch lunabot_drive autonomy_bringup_pc.launch.py'" &

# Dashboard
chromium-browser "file://$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/dashboard/dashboard.html" &
```

### Bypass localization gate for bench/indoor testing (no AprilTags)

If AprilTags are not available, disable the localization gate after launch:
```bash
ssh lunapi 'bash -l -c "ros2 param set /mission_state_node require_localization false"'
```

### Monitoring windows

```bash
xterm -T "monitor: autonomy state" -hold \
  -e "bash -l -c 'ros2 topic echo /autonomy_state'" &

xterm -T "monitor: localization status" -hold \
  -e "bash -l -c 'ros2 topic echo /localization_status'" &

xterm -T "monitor: /odometry/filtered" -hold \
  -e "bash -l -c 'ros2 topic echo /odometry/filtered --no-arr'" &

xterm -T "monitor: lift driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/lift/lift_driver/command'" &

xterm -T "monitor: tilt driver cmd" -hold \
  -e "bash -l -c 'ros2 topic echo /bucket/tilt/tilt_driver/command'" &
```

### Sequence walkthrough

```
Dashboard ARM → READY
Dashboard CONFIRM START — Excavation → AUTONOMOUS
  Watch: lift extends (+1.0 for 18 s)
  Watch: robot navigates to dig zone
  Watch: lift retracts (-1.0 for 18 s)
  Watch: robot navigates home
  Banner → COMPLETE
Dashboard ACKNOWLEDGE → TELEOP

Dashboard ARM → READY
Dashboard CONFIRM START — Deposition → AUTONOMOUS
  Watch: robot navigates to hopper
  Watch: tilt extends (+1.0 for 20 s)
  Watch: wait 3 s
  Watch: tilt retracts (-1.0 for 20 s)
  Watch: robot navigates home
  Banner → COMPLETE
Dashboard ACKNOWLEDGE → TELEOP
  Cycle counter increments to 1
```

### Pass criteria

| Check | Expected |
|-------|----------|
| `/localization_status` | `LOCALIZED` when tag visible, `STALE` otherwise |
| `ARM` rejected within 15 s of last sequence | Log: `Arm rejected — actuator cooldown active: X.X s remaining` |
| `start_excavation` while `STALE` | Log: `Command rejected — localization status is "STALE"` |
| Full excavation sequence | State path: TELEOP → READY → AUTONOMOUS → COMPLETE → TELEOP |
| Joystick input during AUTONOMOUS | Robot responds immediately (mux priority 5 > Nav2 1) |
| **⚠ E-STOP** during AUTONOMOUS | Sequence aborts; nav goal cancelled; banner flashes |
| ACKNOWLEDGE after E-STOP | Returns to TELEOP; does not auto-continue |

---

## Test 5 — Camera + AprilTags

**Goal:** verify the full vision pipeline from OAK-D S2 → apriltag_ros →
apriltag_localizer_node → `/localization_status` and confirm the dashboard
robot camera feed works within bandwidth budget.

**This test can run before Test 3 — no motors or CAN required.**

### Option A — Camera + localization only (no motors)

```bash
# Pi side — software stack + camera (no drive_node)
xterm -T "lunapi — Pi + camera" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py \
      enable_drive:=false enable_camera:=true\"'" &
```

### Option B — AprilTag detection standalone

```bash
xterm -T "lunapi — apriltag detection" \
  -e "ssh -t lunapi 'bash -l -c \"ros2 launch lunabot_drive apriltag_detection.launch.py\"'" &
```

### Option C — Full autonomy stack (camera + localization + Nav2)

Use Test 4 launch commands with `require_localization:=true` (default).

### Dashboard camera feed

```bash
chromium-browser "file://$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/dashboard/dashboard.html" &
```
In the dashboard: **Bandwidth → Robot camera feed → ON**. The overlay should
appear in the top-right corner. Monitor the bandwidth bar — the stream adds
~0.3–0.8 Mbps at quality=50.

### Monitoring windows

```bash
xterm -T "monitor: localization status" -hold \
  -e "bash -l -c 'ros2 topic echo /localization_status'" &

xterm -T "monitor: map→odom TF" -hold \
  -e "bash -l -c 'ros2 run tf2_ros tf2_echo map odom'" &

xterm -T "monitor: bandwidth" -hold \
  -e "bash -l -c 'ros2 topic echo /bandwidth_mbps'" &

xterm -T "monitor: tag detections" -hold \
  -e "bash -l -c 'ros2 topic echo /tf --no-arr'" &
```

### Pass criteria

| Action | Expected |
|--------|----------|
| OAK-D pointing at Tag 0 or Tag 1 | `/localization_status` → `LOCALIZED` |
| Camera obscured | After 3 s: `/localization_status` → `STALE` |
| `map→odom` TF | Updates while tag visible; freezes (last value) when tag lost |
| Robot camera ON in dashboard | MJPEG feed visible, bandwidth bar increases |
| Camera ON + 2 arena cameras selected | Total bandwidth stays under 4.0 Mbps bar |

---

## Quick Reference — Teardown

To kill all xterm monitoring windows opened in a session:
```bash
pkill -f "xterm"
```

To kill only the Pi-side launch remotely:
```bash
ssh lunapi 'pkill -f "ros2 launch"'
```

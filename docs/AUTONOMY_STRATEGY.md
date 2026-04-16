# Autonomy Strategy — Lunabotics 2026
**Team:** Northwestern University  
**Competition:** NASA Lunabotics, Kennedy Space Center, May 2026  
**Robot:** Differential-drive rover, Raspberry Pi 5, SparkFlex CAN motors, OAK-D S2 camera

---

## Competition Goal

The rover must perform autonomous excavation and deposition cycles in a 6.8 m × 5.0 m arena filled with BP-1 lunar regolith simulant. Up to 600 points are available for autonomous operation. A full cycle consists of:

1. **Excavation** — drive to the dig zone, lower the bucket, scoop regolith, return home
2. **Deposition** — drive to the ISRU hopper, dump the bucket, return home

The operator triggers each phase from the Mission Control Center (MCC) dashboard. The rover executes the sequence fully autonomously; the operator can abort and resume teleop at any time.

---

## System Architecture

```
 MCC Dashboard (dreamfyre) ──rosbridge──► lunapi
                                              │
        ┌──────────────────────────────────────┤
        │                                      │
   cmd_vel_mux                          actuator_mux (lift)
  ┌──────────┐                         ┌──────────────┐
  │ gui      │ pri 10                  │ gui          │ pri 10
  │ joystick │ pri  5  ──► /cmd_vel   │ teleop       │ pri  5  ──► lift_driver
  │ nav2     │ pri  1                  │ autonomy     │ pri  1
  └──────────┘                         └──────────────┘
                                         (identical for tilt)
        │
   drive_node (CAN)
        │
   /odom ──► ekf_filter_node ──► /odometry/filtered
                                        │
                                   Nav2 stack
                                        │
                              mission_state_node
                          (orchestrates full cycle)
```

At every layer, teleop has the highest priority and can override autonomy at any time without an explicit abort.

---

## Localization

### Problem
Nav2 requires a `map→odom` TF. Wheel odometry alone drifts on regolith simulant. No GPS, compass, or ultrasonic sensors are permitted.

### Approach: AprilTag absolute positioning

Two AprilTag (tag36h11) markers are mounted on the arena perimeter at known positions. The OAK-D S2 RGB camera feeds `apriltag_ros`, which publishes a TF frame for each detected tag.

**`apriltag_localizer_node`** bridges tag detections into a continuously broadcast `map→odom` transform using:

```
T_map_base = T_map_tag × inv(T_base_tag)
T_map_odom = T_map_base × inv(T_odom_base)
```

Where:
- `T_map_tag` — ground-truth tag position in the map frame (configured in params)
- `T_base_tag` — tag pose relative to the robot (from TF chain via apriltag_ros)
- `T_odom_base` — robot pose in odom frame (from EKF)

When tags leave the camera frame, the last known `map→odom` transform continues to be broadcast. The robot dead-reckons on wheel odometry + IMU until the next tag detection.

Localization status is published to `/localization_status`:
- `UNLOCALIZED` — no tag seen yet since boot
- `LOCALIZED` — tag seen within `stale_timeout` seconds
- `STALE` — last detection is older than `stale_timeout` (3 s default)

**Tag configuration** (`config/params/apriltag_localizer_params.yaml`):

| Tag ID | Default position (map frame) | Notes |
|--------|------------------------------|-------|
| 0 | x=0.10 m, y=0.00 m, z=0.30 m | Arena perimeter |
| 1 | x=0.00 m, y=0.10 m, z=0.30 m | Arena perimeter |

> **Before every match:** measure actual tag positions with a tape measure. Localization accuracy is directly proportional to the accuracy of these values.

### EKF sensor fusion

`ekf_filter_node` (robot_localization) runs at 30 Hz and fuses:

| Source | Topics | States used |
|--------|--------|-------------|
| Wheel odometry | `/odom` | x, y, yaw, vx, vyaw |
| OAK-D S2 BNO086 IMU | `/oak/imu/data` | yaw, vyaw, ax |

Output: `/odometry/filtered` (fused pose with covariance) + TF `odom→base_link`.

`drive_node` runs with `publish_odom_tf: false` when the EKF is active — the EKF owns the `odom→base_link` transform.

> **Competition note:** The BNO086 magnetometer is disabled by default in depthai. This is intentional — Lunabotics rules prohibit compass/magnetometer use.

---

## Navigation

Nav2 with the DWB (Dynamic Window Based) local planner runs on the Pi.

| Component | Setting | Rationale |
|-----------|---------|-----------|
| Global planner | NavFn (Dijkstra) | Simple, reliable for known arena |
| Local planner | DWB | Handles differential drive cleanly |
| Max linear velocity | 0.5 m/s | Conservative for regolith surface |
| Max angular velocity | 1.5 rad/s | |
| XY goal tolerance | 0.15 m | Allows ~15 cm positional error at goal |
| Yaw goal tolerance | 0.25 rad | ~14° |
| Costmap obstacle source | `/oak/points` (PointCloud2) | OAK-D S2 stereo depth |
| Obstacle height filter | 0.05 m – 0.5 m | Ignores ground noise and above-robot obstacles |
| Inflation radius | 0.55 m | Keeps rover clear of obstacles |
| Recovery behaviours | Spin, BackUp, Wait | |

`controller_server` publishes to `/cmd_vel_mux/input/navigation` (priority 1). The joystick (priority 10) and dashboard GUI (priority 5) override Nav2 silently when active.

---

## Autonomy State Machine

Managed by `mission_state_node`. States:

```
TELEOP ──(arm)──► READY ──(start_excavation)──► AUTONOMOUS ──(complete)──► COMPLETE
                      └──(start_deposition)──►        │                         │
                                                   (abort)                 (acknowledge)
                                                       │                         │
                                                       ▼                         ▼
                                                    FAILED ──(5 s)──► TELEOP ◄──┘
                                                       │
                                                    (arm)──► READY

    Any state ──(emergency_stop rising edge)──► ESTOP ──(acknowledge)──► TELEOP
```

**State descriptions:**

| State | Meaning |
|-------|---------|
| `TELEOP` | Operator in full control. Autonomy is disarmed. |
| `READY` | Armed and waiting for the operator to confirm a sequence. |
| `AUTONOMOUS` | A sequence is running in a background thread. Teleop can still override actuators via the mux. |
| `COMPLETE` | Sequence finished successfully. Operator must acknowledge to return to TELEOP. |
| `FAILED` | Sequence failed or was aborted. Auto-recovers to TELEOP after 5 s, or operator can re-arm immediately. |
| `ESTOP` | E-STOP received. Latched — requires explicit `acknowledge` command to clear. |

**Commands** (published to `/autonomy_command` as `std_msgs/String`):

| Command | Valid from state | Effect |
|---------|-----------------|--------|
| `arm` | TELEOP, FAILED | → READY |
| `start_excavation` | READY | → AUTONOMOUS, start excavation sequence |
| `start_deposition` | READY | → AUTONOMOUS, start deposition sequence |
| `abort` | READY, AUTONOMOUS | → FAILED (cancels nav goal, stops actuators) |
| `acknowledge` | COMPLETE, ESTOP | → TELEOP |

Published topics: `/autonomy_state` (String, 5 Hz), `/autonomy_cycle` (Int32, 5 Hz).

---

## Excavation Sequence

Triggered by `start_excavation` from the READY state. Runs in a background daemon thread.

```
Step 1  Publish +1.0 to /bucket/lift_mux/input/autonomy at 10 Hz
        for lift_extend_s (default 18 s)          — lower bucket into digging position

Step 2  NavigateToPose → dig_pose (dig_x, dig_y)  — drive forward to scoop regolith

Step 3  Publish -1.0 to /bucket/lift_mux/input/autonomy at 10 Hz
        for lift_retract_s (default 18 s)          — lift bucket with collected material

Step 4  NavigateToPose → home_pose (home_x, home_y) — return to start zone

→ Transition to COMPLETE
```

**Default waypoints** (must be calibrated on-site):

| Waypoint | Default (m) | Description |
|----------|-------------|-------------|
| `dig_x`, `dig_y` | 4.0, 2.5 | Centre of excavation zone |
| `home_x`, `home_y` | 0.3, 2.5 | Start zone / staging area |

---

## Deposition Sequence

Triggered by `start_deposition` from the READY state.

```
Step 1  NavigateToPose → hopper_pose (hopper_x, hopper_y) — drive to ISRU bin

Step 2  Publish +1.0 to /bucket/tilt_mux/input/autonomy at 10 Hz
        for tilt_extend_s (default 20 s)           — tilt bucket to dump position

Step 3  Wait dump_wait_s (default 3 s)             — let material fall into hopper

Step 4  Publish -1.0 to /bucket/tilt_mux/input/autonomy at 10 Hz
        for tilt_retract_s (default 20 s)          — return bucket to flat

Step 5  NavigateToPose → home_pose                 — return to start zone

→ Transition to COMPLETE
```

**Default waypoints:**

| Waypoint | Default (m) | Description |
|----------|-------------|-------------|
| `hopper_x`, `hopper_y` | 0.5, 2.5 | ISRU collection bin |
| `home_x`, `home_y` | 0.3, 2.5 | Start zone |

---

## Actuator Command Routing

Actuator commands pass through `actuator_mux_node` (a custom Float64 priority/timeout mux mirroring the `cmd_vel_mux` pattern).

**Priority scheme (both lift and tilt):**

| Source | Topic | Priority | Timeout |
|--------|-------|----------|---------|
| Dashboard GUI | `/bucket/{lift,tilt}_mux/input/gui` | 10 | 0.5 s |
| Teleop (d-pad) | `/bucket/{lift,tilt}_mux/input/teleop` | 5 | 0.5 s |
| Autonomy sequence | `/bucket/{lift,tilt}_mux/input/autonomy` | 1 | 0.5 s |
| Mux output | `/bucket/{lift,tilt}/lift_driver/command` | — | — |

The mux publishes at 50 Hz. When no input has sent a message within its timeout window, the mux goes silent and the actuator driver's 2-second watchdog stops the motor.

Teleop always pre-empts autonomy instantaneously — the operator can grab manual control of the actuators at any point during a running sequence without an abort.

**Actuator run durations** (tuned for full stroke with load margin):

| Actuator | Stroke | No-load time | Configured time |
|----------|--------|-------------|-----------------|
| Lift | 203.2 mm | ~13.5 s | 18 s |
| Tilt | 254.0 mm | ~16.9 s | 20 s |

The physical hardware limit switches are the true hard stops. The actuator stalls safely against them.

---

## Abort Safety

At every step of both sequences:

- An `abort_event` (`threading.Event`) is checked before each actuator publish and after each navigation step.
- When `abort` is commanded or the E-STOP fires, the event is set immediately.
- Any active Nav2 goal is cancelled via `goal_handle.cancel_goal_async()`.
- Running `_run_actuator` loops detect the event, publish a stop command (`data: 0.0`), and return `False`.
- The sequence thread exits and the state transitions to `FAILED`.

E-STOP is **latched** — releasing the button does not auto-clear. The operator must publish `acknowledge` to return to TELEOP. This prevents an accidental momentary E-STOP from silently re-enabling autonomy.

---

## Operator Interface (Dashboard)

`dashboard/dashboard.html` connects to `rosbridge_websocket` on lunapi (port 9090) from the MCC laptop (dreamfyre).

**Autonomy controls (Panel 2):**
- ARM, CONFIRM EXCAVATION, CONFIRM DEPOSITION, ABORT, ACKNOWLEDGE buttons
- Buttons are enabled/disabled based on current state — the operator cannot issue an invalid command

**Manual override controls:**
- Lift/tilt hold-to-move buttons → `/bucket/{lift,tilt}_mux/input/gui` (priority 5)
- Arrow-pad drive controls → `/cmd_vel_mux/input/gui` (priority 5)
- Teleop joystick (Switch Pro Controller) takes priority 10 over both

**Live telemetry:**
- Autonomy state banner (colour-coded, flashing on E-STOP)
- Bandwidth bar (4 Mbps limit)
- Node health status (10 critical nodes monitored)
- Lift/tilt position bars
- Velocity, roll, pitch, EKF covariance, CAN status, WiFi RSSI
- Competition run timer (15-minute countdown)

---

## Full Launch Architecture

### lunapi (Raspberry Pi 5) — `autonomy_bringup_pi.launch.py`

| Node | Role |
|------|------|
| `drive_node` | SparkFlex CAN motors, publishes `/odom` |
| `robot_state_publisher` | URDF → TF |
| `ekf_filter_node` | Wheel odom + IMU → `/odometry/filtered`, `odom→base_link` TF |
| `oak` (depthai) | Stereo depth (obstacle costmap) + RGB (AprilTag detection) |
| `apriltag_node` | Detects tag36h11 tags, publishes `camera→tag` TF |
| `apriltag_localizer_node` | `map→odom` TF from tag detections |
| `controller_server` | DWB local planner → `/cmd_vel_mux/input/navigation` |
| `planner_server` | NavFn global planner |
| `behavior_server` | Recovery behaviours |
| `bt_navigator` | Behavior tree executor |
| `lifecycle_manager` | Nav2 autostart |
| `cmd_vel_mux` | Joystick (10) vs GUI (5) vs Nav2 (1) → `/cmd_vel` |
| `lift_mux` / `tilt_mux` | Actuator mux: teleop (10) vs GUI (5) vs autonomy (1) |
| `lift_driver` / `tilt_driver` | Actuator hardware drivers |
| `mission_state_node` | Autonomy state machine + sequences |
| `bandwidth_monitor_node` | Publishes `/bandwidth_mbps` |
| `health_monitor_node` | Publishes `/node_health` |
| `wifi_monitor_node` | Publishes `/wifi_rssi` |
| `rosbridge_websocket` | WebSocket bridge for dashboard (port 9090) |
| `web_video_server` | MJPEG camera stream for dashboard (port 8080) |

### dreamfyre (MCC laptop) — `autonomy_bringup_pc.launch.py`

| Node | Role |
|------|------|
| `joy_node` | Switch Pro Controller |
| `teleop_twist_joy` | Controller → `/cmd_vel_mux/input/joystick` (via LAN) |
| `bucket_teleop_node` | D-pad → `/bucket/{lift,tilt}_mux/input/teleop` (via LAN) |

ROS_DOMAIN_ID=42 is used on both machines for LAN communication.

---

## Pre-Competition Checklist

### Must complete before any autonomy run

- [ ] Measure AprilTag centre positions in the arena, update `apriltag_localizer_params.yaml`
- [ ] Measure AprilTag black-border size, update `apriltag_node size:` parameter
- [ ] Verify actuator direction: command extend, observe rod. If rod retracts, set `invert_direction: true` in `bucket_actuators.yaml`
- [ ] Set arena waypoints (`dig_x/y`, `hopper_x/y`, `home_x/y`) by driving robot to each location and reading `/odometry/filtered`
- [ ] Confirm `/localization_status` shows `LOCALIZED` after driving robot to a position where tags are visible

### Should complete before competition

- [ ] Tune `max_speed_pct` in `bucket_actuators.yaml` for digging under load
- [ ] Determine minimum lift extension depth for target dig depth (currently full stroke)
- [ ] Determine minimum tilt extension angle that fully empties bucket over hopper
- [ ] Tune EKF process noise covariance on actual regolith surface
- [ ] Run full excavation + deposition dry run and verify 15-minute run fits in competition window

### Deferred (post-competition if time allows)

- Closed-loop position control using Hall sensor feedback
- Custom C++ BT action nodes to integrate bucket actuation into Nav2 behavior trees
- Partial-stroke extend/retract with a target depth parameter

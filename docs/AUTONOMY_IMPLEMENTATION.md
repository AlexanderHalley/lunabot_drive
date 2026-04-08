# Autonomy Implementation — Lunabotics 2026
**Date:** April 2026  
**Branch:** bucket-actuators  
**Author:** Claude (session transcript → implementation)

---

## Overview

This document records all autonomy-related changes made to close the gaps identified in `AUTONOMY_GAPS.md`. Seven gaps were addressed. Each section states what the gap was, what was built, and what still needs physical testing or tuning before competition.

---

## Gap 1 — Localization (map→odom transform)

**Problem:** Nav2 requires a `map→odom` TF. Without it nothing in Nav2 works. Three options were identified; the chosen approach is AprilTag absolute pose using the OAK-D S2 camera and `apriltag_ros`.

### What was built

**`nodes/apriltag_localizer_node.py`** (new)

A ROS2 Python node that bridges `apriltag_ros` tag detections into a continuously broadcast `map→odom` TF transform.

**How it works:**

`apriltag_ros` publishes a TF frame for each detected tag with the camera optical frame as parent (e.g. `oak_rgb_camera_optical_frame → tag36h11:0`). The localizer uses three transforms to derive the robot's absolute pose:

```
T_map_tag   — known ground truth: where the tag is in the arena (from params)
T_base_tag  — where the tag is relative to the robot (from TF: base_link ← tag36h11:N)
T_odom_base — where base_link is in the odom frame (from EKF)

T_map_base = T_map_tag  * inv(T_base_tag)
T_map_odom = T_map_base * inv(T_odom_base)
```

The resulting `map→odom` transform is broadcast at 10 Hz via `tf2_ros.TransformBroadcaster`. When tags go out of frame the last known transform continues to be broadcast (dead-reckoning on wheel odom + IMU). Status is published to `/localization_status` (UNLOCALIZED / LOCALIZED / STALE).

**Tag positions configured for competition:**

| Tag ID | Map position (x, y, z) | Notes |
|--------|------------------------|-------|
| 0 | 0.10 m, 0.00 m, 0.30 m | Mounted on arena perimeter |
| 1 | 0.00 m, 0.10 m, 0.30 m | Mounted on arena perimeter |

Both tags use identity orientation as default. Adjust in params if tags are rotated relative to the map x-axis.

**`config/params/apriltag_localizer_params.yaml`** (new)

All tag poses are configurable without recompiling. Full parameter reference is in the file header. The flat `tag_poses` list format is `[x, y, z, qx, qy, qz, qw]` per tag in `tag_ids` order.

**Topics:**

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/detections` | `apriltag_msgs/AprilTagDetectionArray` | Sub | Detection events (timing only) |
| `/tf` | — | Sub | Tag pose frames from apriltag_ros |
| `/tf` | — | Pub | `map→odom` transform |
| `/localization_status` | `std_msgs/String` | Pub | UNLOCALIZED / LOCALIZED / STALE |

**Prerequisite:** `sudo apt install ros-jazzy-apriltag-ros`

### What still needs to be done

- **Measure actual tag positions** in the arena after setup. The (0.1, 0) and (0, 0.1) values are placeholders. Localization accuracy is directly proportional to how accurately these are measured.
- **Set tag orientation** in the params if tags are not facing the default direction (identity quaternion = tag z-axis pointing away from the wall toward the robot).
- **Set tag size** in the `apriltag_node` parameters: `size: 0.16` (metres). Measure the black border of the physical tags.

---

## Gap 2 — Full Nav2 Bringup Launch

**Problem:** No launch file started the complete Nav2 stack. `camera_only_nav2.launch.py` only tested the costmap with a stationary robot.

### What was built

The full Nav2 stack is now split correctly across two launch files (see Gap 2 + Gap 6 combined result below). The Nav2 nodes launched are:

- `controller_server` (DWB local planner, 10 Hz)
- `planner_server` (NavFn global planner)
- `behavior_server` (Spin, BackUp, Wait recoveries)
- `bt_navigator` (behavior tree executor)
- `lifecycle_manager` (autostart: true)

`nav2_params.yaml` was updated to use `/odometry/filtered` (EKF output) as the `odom_topic` for `bt_navigator` rather than raw `/odom`.

---

## Gap 3 — Behavior Trees

**Problem:** No BT XML files existed. `bt_navigator` had no custom behaviors.

### What was built

**`config/behavior_trees/excavation.xml`** (new)  
**`config/behavior_trees/deposition.xml`** (new)

Both BTs follow the standard Nav2 pattern: `RecoveryNode` wrapping a `PipelineSequence` of `ComputePathToPose` + `FollowPath`, with a `ReactiveFallback` recovery chain (clear costmaps → spin → wait → backup).

The BTs handle **navigation only**. Bucket actuation is orchestrated by `mission_state_node` (see Gap 5) before and after the navigation steps. This was a deliberate design choice: adding bucket control into BTs requires custom C++ BT action nodes, which are deferred.

```
excavation.xml:   Navigate to dig_pose → Navigate to home_pose
deposition.xml:   Navigate to hopper_pose → Navigate to home_pose
```

### What still needs to be done

When time permits, custom BT action nodes (C++) can be added to call the `~/extend` and `~/retract` services from within the XML. This would allow the full cycle to be expressed as a single BT action and handled entirely by `bt_navigator`.

---

## Gap 4 — Bucket Service Interface for BT/Autonomy

**Problem:** `actuator_driver_node` only accepted raw `Float64` topic commands. Behavior trees and sequential autonomy code need services that **block until the motion completes**.

### What was built

Two new Trigger services added to **`nodes/actuator_driver_node.py`**:

| Service | Behaviour |
|---------|-----------|
| `~/extend` | Drives actuator at +1.0 for up to `max_continuous_run_s`, then stops. Returns success. |
| `~/retract` | Drives actuator at −1.0 for up to `max_continuous_run_s`, then stops. Returns success. |

Both services block their callback thread for the duration of the motion. This is safe because the node uses `MultiThreadedExecutor` and `ReentrantCallbackGroup` — other callbacks (50 Hz control loop, 5 Hz status publisher) continue running on separate threads.

**Watchdog keepalive:** The services ping `_last_command_time` inside their wait loop every 50 ms. Without this, the 2 s command watchdog would fire approximately 2 s into an 18–20 s motion and cut the motor mid-stroke.

The `~/stop` service (already present) remains unchanged and can be called at any time to interrupt an in-progress extend or retract.

---

## Gap 5 — mission_state_node Wired to Nav2

**Problem:** `mission_state_node` transitioned to `AUTONOMOUS` on command but did nothing — no goal was sent to Nav2, no actuators were moved.

### What was built

**`nodes/mission_state_node.py`** was fully rewritten. The state machine logic is unchanged; what was added is the actual autonomy execution.

**New parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dig_x`, `dig_y` | 4.0, 2.5 | Excavation dig zone in map frame (m) |
| `hopper_x`, `hopper_y` | 0.5, 2.5 | ISRU hopper in map frame (m) |
| `home_x`, `home_y` | 0.3, 2.5 | Home pose in map frame (m) |
| `nav_timeout_s` | 120.0 | Cancel nav goal after this long |
| `service_timeout_s` | 35.0 | Max wait for actuator service response |
| `dump_wait_s` | 3.0 | Seconds to hold tilt extended while dumping |
| `lift_actuator_ns` | `/bucket/lift/actuator_driver` | Service namespace for lift |
| `tilt_actuator_ns` | `/bucket/tilt/actuator_driver` | Service namespace for tilt |

**Excavation sequence** (runs in background thread on `start_excavation` command):

```
1. /bucket/lift/actuator_driver/extend   — lower bucket into regolith position
2. NavigateToPose → dig_pose             — drive forward to scoop
3. /bucket/lift/actuator_driver/retract  — lift bucket with material
4. NavigateToPose → home_pose            — return to start area
→ transition to COMPLETE
```

**Deposition sequence** (runs in background thread on `start_deposition` command):

```
1. NavigateToPose → hopper_pose          — drive to ISRU bin
2. /bucket/tilt/actuator_driver/extend   — tilt bucket to dump
3. sleep dump_wait_s                     — let material fall
4. /bucket/tilt/actuator_driver/retract  — return bucket
5. NavigateToPose → home_pose            — return to start area
→ transition to COMPLETE
```

**Abort safety:** An `abort_event` threading flag is checked at every step. On abort command or E-STOP, the event is set and any active Nav2 goal is cancelled via `goal_handle.cancel_goal_async()`. The sequence thread exits cleanly and the state transitions to `FAILED`.

**Navigation helper** (`_navigate`): Uses `send_goal_async` + polling loop. Checks the abort event every 50 ms. Handles goal rejection, timeout, and non-SUCCESS status codes.

**Service helper** (`_call_service`): Calls any `Trigger` service async and polls until done. Checks abort event every 50 ms. Respects `service_timeout_s`.

### What still needs to be done

- **Set waypoints for the actual arena.** The defaults (dig at 4.0, 2.5; hopper at 0.5, 2.5; home at 0.3, 2.5) are placeholders. These must be measured relative to the map origin (robot start pose) in the competition arena.
- **Dig depth tuning** — the excavation sequence currently runs the lift to full extension. In practice you want a partial extension depth matched to the bucket geometry (see Gap 7 / actuator tuning).

---

## Gap 6 — cmd_vel Multiplexer

**Problem:** `cmd_vel_mux` existed in the workspace but was not wired into any launch file. Nav2 and `teleop_twist_joy` both published to `/cmd_vel`, causing conflicts.

### What was built

**`config/cmd_vel_mux_lunabot.yaml`** (new)

```yaml
subscribers:
  navigation:   topic: cmd_vel_mux/input/navigation   priority: 1   timeout: 0.5 s
  joystick:     topic: cmd_vel_mux/input/joystick     priority: 10  timeout: 0.15 s
output_topic: cmd_vel
```

Joystick priority 10 > Nav2 priority 1. When the controller is active (message within 0.15 s) it immediately pre-empts Nav2 without needing an explicit abort. When the controller goes idle (no message for 0.15 s) Nav2 resumes automatically.

The `controller_server` node is launched with `remappings=[('cmd_vel', '/cmd_vel_mux/input/navigation')]`.  
`teleop_twist_joy` is launched with `remappings=[('cmd_vel', '/cmd_vel_mux/input/joystick')]`.

---

## Gap 7 — EKF in Autonomy Path

**Problem:** The proof-of-life launch omitted the EKF intentionally (open-loop teleop). For Nav2, wheel odometry alone drifts too much across a full excavation + deposition cycle on regolith simulant.

### What was built

`autonomy_bringup_pi.launch.py` runs `ekf_filter_node` and sets `drive_node publish_odom_tf: false`. The EKF takes:

- `odom0: /odom` — wheel odometry from `drive_node` (x, y, yaw, vx, vyaw)
- `imu0: /oak/imu/data` — BNO086 on OAK-D S2 (yaw, vyaw, ax)

and publishes:

- `/odometry/filtered` — fused pose with covariance
- TF `odom→base_link`

`nav2_params.yaml` `bt_navigator.odom_topic` updated to `/odometry/filtered`.

---

## Launch File Architecture

### Pi side — `launch/autonomy_bringup_pi.launch.py`

Run on **lunapi** (Raspberry Pi 5). Everything with hardware dependencies or low-latency control loops.

```
drive_node          — CAN bus (can0), publish_odom_tf: false
robot_state_publisher
ekf_filter_node     — /odom + /oak/imu/data → /odometry/filtered
oak_d_camera        — pointcloud config (includes RGB for apriltag_ros)
apriltag_node       — detects tag36h11 tags, publishes camera→tag TF
apriltag_localizer  — map→odom TF from tag detections
controller_server   → remapped to /cmd_vel_mux/input/navigation
planner_server
behavior_server
bt_navigator
lifecycle_manager   — autostart: true
cmd_vel_mux         — joystick (pri 10) vs Nav2 (pri 1) → /cmd_vel
bucket_bringup      — lift + tilt actuator driver nodes
mission_state_node  — autonomy orchestrator
bandwidth_monitor_node
health_monitor_node
wifi_monitor_node
rosbridge_websocket — port 9090, for dashboard.html on dreamfyre
```

**Launch arguments** (all can be overridden at launch time):

| Argument | Default | Description |
|----------|---------|-------------|
| `dig_x` | 4.0 | Dig zone X (m) |
| `dig_y` | 2.5 | Dig zone Y (m) |
| `hopper_x` | 0.5 | Hopper X (m) |
| `hopper_y` | 2.5 | Hopper Y (m) |
| `home_x` | 0.3 | Home X (m) |
| `home_y` | 2.5 | Home Y (m) |

### PC side — `launch/autonomy_bringup_pc.launch.py`

Run on **dreamfyre** (MCC laptop). Joystick + bucket teleop only. All commands travel over the LAN via `ROS_DOMAIN_ID=42`.

```
joy_node            — Switch Pro controller
teleop_twist_joy    → remapped to /cmd_vel_mux/input/joystick  (LAN → Pi mux)
bucket_teleop_node  → /bucket/lift/actuator_driver/command     (LAN → Pi actuators)
                    → /bucket/tilt/actuator_driver/command
```

---

## Actuator Parameter Fixes

### `max_continuous_run_s` was too short

The previous value (15 s) was derived from the datasheet no-load speed (15 mm/s) without accounting for actual stroke lengths:

| Actuator | Stroke | No-load time | Old value | New value |
|----------|--------|-------------|-----------|-----------|
| Lift | 203.2 mm | 13.5 s | 15.0 s | **18.0 s** |
| Tilt | 254.0 mm | 16.9 s | 15.0 s | **20.0 s** |

The tilt at 15 s would have stopped 1.9 s before reaching full extension at no load — and further short under any drag load, meaning the bucket would never have fully deployed for dumping.

The new values add ~4 s of margin for load-induced slowdown. The built-in hardware limit switches are the true hard stops; the actuator stalls safely against them.

### `invert_direction` — requires bench verification

Both are currently `false`. On first power-on, command extend and observe physical direction. If the rod retracts, set `invert_direction: true` in `bucket_actuators.yaml` for that actuator.

---

## Files Changed

### New files

| File | Description |
|------|-------------|
| `nodes/apriltag_localizer_node.py` | AprilTag → map→odom TF bridge |
| `config/params/apriltag_localizer_params.yaml` | Tag positions and localizer params |
| `config/behavior_trees/excavation.xml` | Nav2 BT for excavation navigation |
| `config/behavior_trees/deposition.xml` | Nav2 BT for deposition navigation |
| `config/cmd_vel_mux_lunabot.yaml` | cmd_vel mux: joystick priority over Nav2 |
| `launch/autonomy_bringup_pi.launch.py` | Full autonomy stack for lunapi |
| `launch/autonomy_bringup_pc.launch.py` | Teleop-only launch for dreamfyre |

### Modified files

| File | What changed |
|------|-------------|
| `nodes/mission_state_node.py` | Full rewrite: added Nav2 action client, actuator service clients, excavation + deposition sequences, abort safety |
| `nodes/actuator_driver_node.py` | Added `~/extend` and `~/retract` blocking Trigger services with watchdog keepalive |
| `config/bucket_actuators.yaml` | `max_continuous_run_s`: lift 15→18 s, tilt 15→20 s |
| `config/params/nav2_params.yaml` | `odom_topic`: `/odom` → `/odometry/filtered`; added `default_nav_to_pose_bt_xml` note |
| `CMakeLists.txt` | Added `apriltag_localizer_node` install target |

---

## Items Remaining Before Competition

These require physical hardware access and cannot be done in software alone.

### Must do before any autonomy run

| Item | How |
|------|-----|
| Measure tag positions in arena | Tape measure from robot start origin to each tag centre |
| Verify `invert_direction` for lift and tilt | Command extend, observe physical motion; flip if backwards |
| Measure full-stroke travel time under load | Time the actuator from stop to limit switch at various `max_speed_pct` values |
| Set `apriltag_node size:` to actual tag size | Measure the black border of the printed tags in metres |
| Set arena waypoints (`dig_x/y`, `hopper_x/y`, `home_x/y`) | Drive robot to each location, read pose from `/odometry/filtered` |

### Should do before competition

| Item | How |
|------|-----|
| Tune `max_speed_pct` for autonomous digging | Start at 50%, increase if dig depth insufficient; watch for premature duty-cycle cutoff |
| Determine dig depth (partial lift extension) | With bucket geometry, find the extension at which the cutting edge is at target depth |
| Determine dump angle (partial tilt extension) | Test over hopper mockup: minimum tilt extension that empties the bucket cleanly |
| Verify `pulses_per_mm` = 17.4 | If implementing closed-loop position control: command 100 mm, count Hall pulses |

### Deferred (post-competition if time allows)

- Closed-loop position control using Hall sensor feedback (framework exists in `hall_sensors_read.py` but driver is still open-loop)
- Custom C++ BT action nodes to integrate bucket control into the BT XML
- Partial-stroke extend/retract services with a target depth parameter

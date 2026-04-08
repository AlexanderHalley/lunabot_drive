# Clanker 1 Operator Dashboard — Implementation Spec

**Project:** Northwestern University Lunabotics — NASA Lunabotics Challenge 2026  
**Robot:** Clanker 1  
**Rev:** 1.0 — March 2026  
**Runs on:** dreamfyre (MCC laptop) + lunapi (Pi 5)

---

## Overview

A lightweight browser-based operator dashboard that runs **alongside RViz2** on dreamfyre during competition. It does NOT replace RViz2. It fills the operational gaps RViz2 cannot cover:

- Autonomy state management with visible MCJ-facing banner
- Live bandwidth monitoring against the 4 Mbps competition cap
- Node health indicators for all critical system components
- Bucket actuator position readout
- Competition run telemetry strip

**Architecture:** rosbridge_suite on lunapi → WebSocket (port 9090) → single `dashboard.html` in Chromium on dreamfyre.

---

## Competition Rules That Drive Requirements

### MCC Rules
- Max 4 team members in MCC at any time
- No external comms once run starts
- Only competition-required devices allowed
- Conduct must match NASA operational standards

### Autonomy Scoring (up to 600 pts)
- Must **announce to MCJ with eye contact** before every autonomy attempt
- **Hands-free** = all team members release ALL equipment (laptops, controllers) for the entire attempt
- Failure to pre-announce = **0 points** for that attempt
- Must explicitly announce **failure** before resuming manual control
- Two full excavation+deposition cycles required for 600-point tier

### Bandwidth Scoring Formula
```
Total_Mbps = (num_arena_cameras × 0.2) + wifi_mbps
Score = Total_Mbps × -0.3 + 120
Score = 0 if Total_Mbps > 4.0
```

| WiFi Mbps | Cameras | Total | Score |
|-----------|---------|-------|-------|
| 0.3 | 0 | 0.3 | 119.91 |
| 0.5 | 0 | 0.5 | 119.85 |
| 0.5 | 1 | 0.7 | 119.79 |
| 1.0 | 0 | 1.0 | 119.70 |
| 2.0 | 0 | 2.0 | 119.40 |
| 4.0 | 0 | 4.0 | 118.80 |
| 4.1 | 0 | 4.1 | **0 — OVER LIMIT** |

### Mission Anomaly Declarations (must catch before judge)
- **Loss of Comm** — robot functional, comms degraded
- **Loss of Locomotion** — no movement for 5+ min
- **Loss of Excavation** — can't acquire regolith
- **Loss of Deposition** — can't offload regolith

---

## System Architecture

### Network Topology

```
lunapi (Pi 5)
├── All hardware nodes (sparkflex, oak_d, rplidar, imu, bucket_controller)
├── Nav2 stack (bt_navigator, controller_server, etc.)
├── rosbridge_server → ws://lunapi:9090
├── mission_state_node     → /autonomy_state, sub /autonomy_command
├── bandwidth_monitor_node → /bandwidth_mbps
├── health_monitor_node    → /node_health
└── wifi_monitor_node      → /wifi_rssi

dreamfyre (MCC laptop)
├── RViz2 (map, costmap, pose visualization — unchanged)
├── joy_node + teleop_twist_joy (Switch Pro controller)
└── Chromium → dashboard.html → ws://lunapi:9090
```

### Technology Decision

| Approach | Pros | Cons | Decision |
|---|---|---|---|
| rosbridge + HTML | Zero compilation, instant edits, any browser, easy debug | Needs rosbridge running on Pi | **CHOSEN** |
| rqt Plugin | Native ROS2 integration | Slow to develop, colcon build on every change | Deferred |
| PyQt5 Standalone | Full Python access, no rosbridge dep | No native topic binding, more boilerplate | Rejected |

### Key Constraints
- `ROS_DOMAIN_ID=42` used on all machines
- rosbridge traffic is LAN-only — does **not** count toward competition bandwidth cap
- All hardware-facing logic stays on lunapi; dashboard is read/trigger only

---

## Dashboard Layout

Six panels on a 1920×1080 display in a 2-column grid:

```
┌─────────────────────────────────────────────────────────────┐
│           PANEL 1: Autonomy State Banner (full width)        │
├──────────────────────┬──────────────────────────────────────┤
│  PANEL 3: Bandwidth  │   PANEL 2: Behavior Trigger Controls │
├──────────────────────┼──────────────────────────────────────┤
│  PANEL 4: Node Health│   PANEL 5: Bucket Subsystem          │
├──────────────────────┴──────────────────────────────────────┤
│           PANEL 6: Telemetry Strip (full width)              │
└─────────────────────────────────────────────────────────────┘
```

---

## Panel Specifications

### Panel 1: Autonomy State Banner (full width, top)

Most critical element. Must be readable from across the room and unambiguous to the MCJ.

| State | Color | Display Text | Triggered By |
|---|---|---|---|
| `TELEOP` | Gray `#555555` | `TELEOP — Manual Control Active` | Default / fallback |
| `READY` | Blue `#1A3A5C` | `READY — Press button to begin autonomy attempt` | ARM button pressed |
| `AUTONOMOUS` | Green `#00AA44` | `AUTONOMOUS — HANDS FREE — Cycle N of 2` | CONFIRM START pressed |
| `COMPLETE` | Teal `#009999` | `AUTONOMY COMPLETE — Declare to MCJ` | BT success callback |
| `FAILED` | Red `#CC2200` | `AUTONOMY FAILED — Announce failure, resume manual` | BT failure / timeout |
| `ESTOP` | Flashing Red | `E-STOP ACTIVE` | `/emergency_stop` received |

**Topic bindings:**
- Subscribes: `/autonomy_state` (`std_msgs/String`)
- Publishes: `/autonomy_command` (`std_msgs/String`) — values: `"arm"`, `"start_excavation"`, `"start_deposition"`, `"abort"`, `"acknowledge"`

---

### Panel 2: Behavior Trigger Controls (top right)

Large buttons. Each button is only active in the correct state to prevent accidental triggers.

| Button | Active In | Publishes | Effect |
|---|---|---|---|
| ARM AUTONOMY | `TELEOP` | `/autonomy_command: "arm"` | State → `READY` |
| CONFIRM START — Excavation | `READY` | `/autonomy_command: "start_excavation"` | Launches excavation BT, state → `AUTONOMOUS` |
| CONFIRM START — Deposition | `READY` | `/autonomy_command: "start_deposition"` | Launches deposition BT, state → `AUTONOMOUS` |
| ABORT — RESUME TELEOP | `READY` or `AUTONOMOUS` | `/autonomy_command: "abort"` | Cancels BT, state → `FAILED` then `TELEOP` |
| ACKNOWLEDGE COMPLETE | `COMPLETE` | `/autonomy_command: "acknowledge"` | Resets to `TELEOP`, increments cycle counter |

**Cycle counter** displayed below buttons: `Cycle 1 of 2`. Two complete excavation+deposition cycles = 600-point tier.

---

### Panel 3: Live Bandwidth Monitor (top left)

Polls real-time WiFi throughput and projects competition bandwidth score.

| Mbps Range | Color | Label |
|---|---|---|
| 0–1.5 | Green | `EXCELLENT — Max bandwidth score` |
| 1.5–3.0 | Yellow | `CAUTION — Reduce if possible` |
| 3.0–3.8 | Orange | `WARNING — Approaching limit` |
| 3.8–4.0 | Flashing Red | `CRITICAL — Near disqualification threshold` |
| > 4.0 | Solid Red | `OVER LIMIT — Zero bandwidth points` |

**Displays:**
- Current WiFi Mbps (numeric + horizontal bar)
- Arena camera count (manual toggle, 0–2)
- Computed total Mbps including camera penalty
- Projected bandwidth score (live)

**Implementation:** `bandwidth_monitor_node.py` on lunapi polls `/proc/net/dev` at 2 Hz, publishes `/bandwidth_mbps` (`std_msgs/Float32`). Dashboard subscribes via rosbridge.

---

### Panel 4: Node Health Monitor (middle left)

Green/red indicators updated every 3 seconds. A node missing for >3s turns red.

| Node | Package | Failure Impact |
|---|---|---|
| `sparkflex_driver` | `lunabot_one` | Loss of locomotion and odometry |
| `robot_localization` (EKF) | `robot_localization` | Loss of pose estimation |
| `apriltag_ros` | `apriltag_ros` | Loss of absolute localization |
| `rplidar_node` | `rplidar_ros` | Loss of obstacle detection |
| `oak_d_node` | `depthai_ros_driver` | Loss of depth perception |
| `bt_navigator` | `nav2_bt_navigator` | Loss of autonomous behavior |
| `controller_server` | `nav2_controller` | Loss of path following |
| `rosbridge_server` | `rosbridge_server` | Loss of dashboard (self-monitoring) |
| `bucket_controller` | `lunabot_one` | Loss of excavation actuation |

**Implementation:** `health_monitor_node.py` on lunapi calls `ros2 node list` via subprocess at 1 Hz, publishes JSON blob to `/node_health` (`std_msgs/String`).

```json
{
  "sparkflex_driver": true,
  "robot_localization": true,
  "apriltag_ros": false,
  ...
  "timestamp": 1234567890.123
}
```

---

### Panel 5: Bucket Subsystem Readout (middle right)

Shows Hall sensor feedback from the two Firgelli Super Duty linear actuators. Values come from `bucket_controller` node which reads gpiochip4 (RP1 southbridge) GPIO via voltage dividers.

| Channel | Topic | Type | Display |
|---|---|---|---|
| Lift Actuator | `/bucket/lift_position` | `std_msgs/Float32` (0.0–1.0) | Progress bar + mm estimate + state: `RETRACTED / EXTENDING / EXTENDED` |
| Tilt Actuator | `/bucket/tilt_position` | `std_msgs/Float32` (0.0–1.0) | Progress bar + mm estimate + state: `FLAT / TILTING / DUMPING` |
| Bucket State | `/bucket/state` | `std_msgs/String` | Composite: `IDLE / EXCAVATING / LOADED / DEPOSITING` |

---

### Panel 6: Telemetry Strip (full width, bottom)

Compact single-row status strip of secondary telemetry.

| Field | Topic | Type | Notes |
|---|---|---|---|
| Robot Mode | `/autonomy_state` | `std_msgs/String` | Compact mirror of Panel 1 |
| Linear Velocity | `/odom` | `nav_msgs/Odometry` | m/s from sparkflex_driver |
| IMU Roll/Pitch | `/imu/data` | `sensor_msgs/Imu` | From BMI088 — flags potential tipping |
| EKF Covariance | `/odometry/filtered` | `nav_msgs/Odometry` | Pose uncertainty: low/med/high |
| Run Timer | — | Local JS | Counts up from run start, stops at 15:00 |
| CAN Status | `/can_status` | `std_msgs/String` | `OK` / `ERROR` from sparkflex_driver heartbeat |
| WiFi RSSI | `/wifi_rssi` | `std_msgs/Int32` | dBm from iwconfig poll on lunapi |
| Competition Run # | — | Manual entry | Run 1 or Run 2; used for log labeling |

---

## Complete Topic Reference

| Topic | Type | Dir | Publisher | Panel |
|---|---|---|---|---|
| `/autonomy_state` | `std_msgs/String` | Sub | `mission_state_node` | 1, 6 |
| `/autonomy_command` | `std_msgs/String` | Pub | dashboard | 2 |
| `/bandwidth_mbps` | `std_msgs/Float32` | Sub | `bandwidth_monitor_node` | 3 |
| `/node_health` | `std_msgs/String` (JSON) | Sub | `health_monitor_node` | 4 |
| `/bucket/lift_position` | `std_msgs/Float32` | Sub | `bucket_controller` | 5 |
| `/bucket/tilt_position` | `std_msgs/Float32` | Sub | `bucket_controller` | 5 |
| `/bucket/state` | `std_msgs/String` | Sub | `bucket_controller` | 5 |
| `/odom` | `nav_msgs/Odometry` | Sub | `sparkflex_driver` | 6 |
| `/odometry/filtered` | `nav_msgs/Odometry` | Sub | `robot_localization` EKF | 6 |
| `/imu/data` | `sensor_msgs/Imu` | Sub | BMI088 driver | 6 |
| `/can_status` | `std_msgs/String` | Sub | `sparkflex_driver` | 6 |
| `/wifi_rssi` | `std_msgs/Int32` | Sub | `wifi_monitor_node` | 6 |

---

## New Nodes to Create on lunapi

All nodes are pure rclpy Python. No C++ compilation needed. Install as Python scripts in CMakeLists.txt.

### `mission_state_node.py`
- **Path:** `src/mission_state_node.py`
- **Publishes:** `/autonomy_state` (`std_msgs/String`)
- **Subscribes:** `/autonomy_command` (`std_msgs/String`)
- **Logic:** State machine — `TELEOP → READY → AUTONOMOUS → COMPLETE/FAILED → TELEOP`. On `"start_excavation"` or `"start_deposition"`, calls Nav2 BT action server. On BT result callback, transitions to `COMPLETE` or `FAILED`. Publishes state at 5 Hz for dashboard responsiveness.
- **Estimate:** 2 hrs

### `bandwidth_monitor_node.py`
- **Path:** `src/bandwidth_monitor_node.py`
- **Publishes:** `/bandwidth_mbps` (`std_msgs/Float32`)
- **Logic:** Reads `/proc/net/dev` (interface: `wlan0` or `wlp*`), computes bytes-per-second delta at 2 Hz, converts to Mbps, publishes. Uses the GL-MT3000 interface name — verify with `ip link` on lunapi.
- **Estimate:** 1 hr

### `health_monitor_node.py`
- **Path:** `src/health_monitor_node.py`
- **Publishes:** `/node_health` (`std_msgs/String`, JSON payload)
- **Logic:** Calls `subprocess.run(['ros2', 'node', 'list'])` at 1 Hz. Checks each expected node name against the output. Publishes JSON dict of `{node_name: bool}` with a timestamp. Nodes absent for >3 consecutive polls are considered dead.
- **Estimate:** 2 hrs

### `wifi_monitor_node.py`
- **Path:** `src/wifi_monitor_node.py`
- **Publishes:** `/wifi_rssi` (`std_msgs/Int32`)
- **Logic:** Runs `subprocess.run(['iwconfig', 'wlan0'])` at 1 Hz, parses `Signal level=` field, publishes dBm as Int32.
- **Estimate:** 0.5 hr

---

## Files to Create

```
lunabot_one/
├── dashboard/
│   └── dashboard.html              # Single-file browser UI (roslibjs + vanilla JS)
├── src/
│   ├── mission_state_node.py       # Autonomy state machine
│   ├── bandwidth_monitor_node.py   # WiFi throughput monitor
│   ├── health_monitor_node.py      # Node liveness checker
│   └── wifi_monitor_node.py        # RSSI monitor
└── launch/
    └── mcc_bringup.launch.py       # Launches all 4 nodes + rosbridge
```

---

## rosbridge Installation & Launch

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

Add to `mcc_bringup.launch.py`:

```python
Node(
    package='rosbridge_server',
    executable='rosbridge_websocket',
    name='rosbridge',
    parameters=[{'port': 9090}]
)
```

Launch command on lunapi:
```bash
ros2 launch lunabot_one mcc_bringup.launch.py
```

This launch file includes: `mission_state_node`, `bandwidth_monitor_node`, `health_monitor_node`, `wifi_monitor_node`, `rosbridge_websocket`. Hardware nodes are launched separately via the existing hardware bringup.

---

## dashboard.html Key Implementation Notes

```javascript
// Connection
const ros = new ROSLIB.Ros({ url: 'ws://lunapi:9090' });

// Subscribe pattern
const stateTopic = new ROSLIB.Topic({
  ros, name: '/autonomy_state', messageType: 'std_msgs/String'
});
stateTopic.subscribe(msg => updateStateBanner(msg.data));

// Publish pattern
const cmdTopic = new ROSLIB.Topic({
  ros, name: '/autonomy_command', messageType: 'std_msgs/String'
});
cmdTopic.publish(new ROSLIB.Message({ data: 'arm' }));
```

- Use roslibjs from CDN or local copy (store in `/dashboard/lib/` for offline competition use)
- State banner: CSS class swap on the `<body>` or banner `<div>` — one class per state with background-color defined in CSS
- Bandwidth bar: CSS `width` transition on a `<div>` inside a fixed-width container
- Node health: `<span>` elements with green/red background per node name
- All UI state stored in JS variables — no `localStorage` (not available in rosbridge artifact context)
- Implement auto-reconnect: `ros.on('error', ...)` and `ros.on('close', ...)` with 3-second retry
- Run timer: `setInterval` counting seconds from a `startTime` variable, reset on page load or manual button

---

## MCC Laptop Setup Checklist (dreamfyre)

1. Confirm `ROS_DOMAIN_ID=42` in `~/.bashrc`
2. Confirm lunapi is reachable: `ping lunapi`
3. `ros2 launch lunabot_one hardware_bringup.launch.py` on lunapi
4. `ros2 launch lunabot_one mcc_bringup.launch.py` on lunapi
5. Open RViz2 — verify robot pose, costmap, sensor topics
6. Open `dashboard.html` in Chromium — verify rosbridge connection indicator is green
7. Confirm all node health indicators are green
8. Confirm bandwidth reads ~0 Mbps at idle
9. Connect Switch Pro controller — verify `joy_node` receives input
10. Set Competition Run # (1 or 2) in dashboard
11. Brief MCJ on autonomy ConOps before entering MCC

---

## Competition Run Procedure

### Teleop Phase
- Drive with Switch Pro controller
- Watch Node Health panel — any red warrants investigation before attempting autonomy
- Watch Bandwidth strip — keep below 1.5 Mbps during video streaming
- Watch Bucket panel to confirm actuator positions

### Starting an Autonomy Attempt
1. Press **ARM AUTONOMY** — banner → `READY` (blue)
2. Make eye contact with MCJ: *"We are about to begin an autonomy attempt for [Excavation / Deposition]"*
3. MCJ acknowledges
4. Press **CONFIRM START** — banner → `AUTONOMOUS` (green)
5. All team members release all equipment immediately
6. Behavior tree executes on lunapi

### Autonomy Completion
- BT succeeds → banner → `COMPLETE` (teal)
- Announce to MCJ: *"Autonomy attempt complete"*
- Press **ACKNOWLEDGE COMPLETE** → resets to `TELEOP`, cycle counter increments

### Autonomy Failure
- BT fails or times out → banner → `FAILED` (red)
- Announce to MCJ: *"Autonomy attempt failed"* **before touching any equipment**
- Resume manual control

---

## Development Priority & Schedule

Target: Dashboard functional for full system test by **April 15, 2026**.  
Bandwidth proof-of-life video required by **April 30, 2026**.

| Priority | Task | Est. | What It Unblocks |
|---|---|---|---|
| **P0** | `mission_state_node.py` + autonomy state banner HTML | 3 hrs | Competition scoring communication |
| **P0** | rosbridge install + `mcc_bringup.launch.py` | 1 hr | All dashboard functionality |
| **P1** | `bandwidth_monitor_node.py` + bandwidth panel | 2 hrs | Bandwidth score optimization |
| **P1** | `health_monitor_node.py` + node health panel | 2.5 hrs | Anomaly detection |
| **P2** | Bucket telemetry panel (topics already exist) | 1.5 hrs | Excavation cycle awareness |
| **P2** | Telemetry strip (odom, IMU, CAN, run timer) | 2 hrs | Situational awareness |
| **P3** | `wifi_monitor_node.py` + RSSI display | 0.5 hr | Nice to have |
| **P3** | Polish, reconnect logic, responsive layout | 1.5 hrs | Robustness |

**Total: ~14 hours.** P0 alone (~4 hrs) provides the highest competition scoring impact.

---

## Explicit Scope Exclusions

The following are out of scope to preserve time for higher-ROI autonomy work:

- Map / costmap / occupancy grid visualization → use RViz2
- Camera video streams → use RViz2 image topics or VLC
- Teleop joystick control in browser → use Switch Pro + `joy_node`
- Full log viewer → use `ros2 topic echo` or `rqt_console`
- Simulation / Gazebo integration → competition is hardware only
- Hardware parameter control (PID tuning, motor limits, etc.)

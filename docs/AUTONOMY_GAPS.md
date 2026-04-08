# Autonomy Gaps — Lunabotics 2026

**Date:** April 2026
**Target:** Full autonomous excavation + deposition cycle (600-point tier)

---

## Gap Summary (Priority Order)

### 1. Localization / Map Frame — Biggest Blocker

Nav2 requires a `map->odom` transform. Nothing in Nav2 works without it. Three options:

| Option | Pros | Cons | Status |
|---|---|---|---|
| **SLAM Toolbox** | Easy to get running, no pre-survey | Needs a mapping session before each match, can drift | Not implemented |
| **Static map + AMCL** | Most reliable at match time | Requires pre-match survey + map file | Not implemented |
| **AprilTag absolute pose** | Corrections without pre-survey | Needs bridge from tag detections into EKF | `apriltag_detection.launch.py` exists, bridge missing |

**Recommendation:** Start with SLAM Toolbox to unblock everything else. Add AprilTag corrections later for robustness.

---

### 2. Full Nav2 Bringup Launch — Missing

`config/params/nav2_params.yaml` is configured but no launch file starts the complete Nav2 stack:
- `bt_navigator`
- `controller_server`
- `planner_server`
- `behavior_server`
- `lifecycle_manager`

`camera_only_nav2.launch.py` only tests the costmap with a stationary robot — it is not a full autonomy launch.

**To create:** `launch/autonomy_bringup.launch.py`

---

### 3. Behavior Trees — Missing Entirely

`bt_navigator` is configured in `nav2_params.yaml` but no custom BT XML files exist anywhere in the repo. The default Nav2 BT only does `NavigateToPose` — it has no bucket awareness.

**Needed:**

- **`excavation.xml`** — navigate to dig zone → lower lift → drive forward (scoop) → raise lift → navigate back
- **`deposition.xml`** — navigate to hopper → tilt bucket → wait → retract tilt → navigate back

Both BTs need to handle:
- Navigation failures (obstacle, timeout)
- Bucket hardware faults
- Returning to a safe pose on failure

**To create:** `config/behavior_trees/excavation.xml`, `config/behavior_trees/deposition.xml`

---

### 4. Bucket Not Callable from a Behavior Tree — Missing

`actuator_driver_node` only accepts a `Float64` topic command. Behavior trees call **actions or services**, not raw topics.

**Needed:** A service wrapper around the actuators so BT nodes can block until an actuator move completes.

Suggested interface:
```
/bucket/lift/extend    → std_srvs/Trigger
/bucket/lift/retract   → std_srvs/Trigger
/bucket/tilt/extend    → std_srvs/Trigger
/bucket/tilt/retract   → std_srvs/Trigger
```

Each service drives the actuator for its `max_continuous_run_s` duration (already enforced by the driver watchdog) and returns success.

**To create:** Service interface added to `actuator_driver_node.py` or a thin `bucket_controller_node.py` wrapper.

---

### 5. `mission_state_node` Not Wired to Nav2 — Stub Only

`nodes/mission_state_node.py` transitions to `AUTONOMOUS` correctly when commanded from the dashboard, but it does not actually send a goal to `bt_navigator` or any Nav2 action server. The AUTONOMOUS state is currently a no-op.

**To add:** On `start_excavation` / `start_deposition`, send a `NavigateToBehaviorTree` action goal (or equivalent) to `bt_navigator`, then transition to `COMPLETE` or `FAILED` based on the action result callback.

---

### 6. `cmd_vel` Multiplexer — Not Wired In

`cmd_vel_mux` package exists in the workspace (`src/cmd_vel_mux/`) but is not used in any launch file. When Nav2 is active it publishes to `cmd_vel` — so does `teleop_twist_joy`. Without a mux they conflict.

**To do:**
- Wire `cmd_vel_mux` into the launch: teleop → `/cmd_vel_teleop`, Nav2 → `/cmd_vel_nav`, mux output → `/cmd_vel` → `drive_node`
- Lock out teleop input while `autonomy_state == AUTONOMOUS`

---

### 7. EKF Not in Autonomy Path

The proof-of-life launch omits the EKF intentionally. For Nav2, wheel odometry alone will drift too much across a full excavation + deposition cycle (~6 m of lunar regolith simulant).

**To do:** Add EKF back to the autonomy bringup launch with `drive_node publish_odom_tf:=false`.

---

## Work Items

| # | Task | Effort | Unblocks |
|---|---|---|---|
| 1 | SLAM Toolbox bringup + mapping session | High | All of Nav2 |
| 2 | `autonomy_bringup.launch.py` (full Nav2 stack) | Low | Autonomous navigation |
| 3 | `excavation.xml` + `deposition.xml` BTs | High | Autonomy scoring |
| 4 | Bucket service interface | Medium | BT bucket control |
| 5 | Wire `mission_state_node` → Nav2 action server | Medium | Dashboard-triggered autonomy |
| 6 | `cmd_vel_mux` integration | Low | Teleop/auto coexistence |
| 7 | EKF in autonomy launch | Low | Pose accuracy over full cycle |

**Total competition scoring impact:** Items 1–3 are required for any autonomy points. Items 4–5 are required for the full 600-point tier. Items 6–7 are reliability improvements.

**Target:** Full system test by April 15, 2026. Bandwidth proof-of-life video by April 30, 2026.

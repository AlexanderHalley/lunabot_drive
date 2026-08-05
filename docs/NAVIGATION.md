# Navigation

Nav2 on a skid-steer rover with fabricated odometry and one forward-facing camera. Every one of
those three facts is a problem for a stock Nav2 configuration, and the tuning notes below are
mostly about them.

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock slam:=rtabmap nav:=true rviz:=true
```

**Status: started, not tuned.** The stack comes up, the frames and limits agree with the rest of
the workspace, and tests enforce that they keep agreeing. No rover has followed a path with it.

---

## `nav:=true` needs `slam:=`

Nav2's global costmap is rooted in `map`, and `bt_navigator`'s `global_frame` is `map`. Nothing in
this workspace publishes `map → odom` except the SLAM backend, so:

```bash
# works
robot.launch.py hw:=mock slam:=rtabmap nav:=true

# comes up, then waits forever for a transform that never arrives
robot.launch.py hw:=mock nav:=true
```

The second is deliberately **not** blocked. Running Nav2 against a bag, or against someone else's
`map → odom` publisher, is a real thing to want, and `robot.launch.py` documents its mutual
exclusions rather than policing them. But if `bt_navigator` rejects every goal with a TF timeout,
this is the first thing to check.

---

## Nav2 never touches `/cmd_vel`

This is the part worth reading even if you skip the rest.

```
  controller_server ─┐
                     ├─▶ /cmd_vel_nav_unsmoothed ─▶ velocity_smoother
  behavior_server  ──┘                                     │
                                                           ▼
                                                    /cmd_vel_nav
                                                           │
    teleop ──/cmd_vel_joy──▶ twist_mux ◀───────────────────┘
                                 │
                                 ▼
                             /cmd_vel ──▶ diff_drive_controller
```

`twist_mux` arbitrates, joystick at priority 100 against navigation at 10, so a human reaching for
the controller takes the bus back. That only works while **`/cmd_vel` has exactly one publisher**.

`nav2_bringup`'s own `navigation_launch.py` remaps `velocity_smoother`'s output
`cmd_vel_smoothed` onto **`cmd_vel`** — the mux's output topic. Including it would have put Nav2
straight onto the wire beside `twist_mux`, leaving the priority table arbitrating between teleop
and nothing while Nav2 drove the robot regardless. Remappings inside an included launch
description cannot be cleanly overridden from outside, so `navigation.launch.py` declares the five
Nav2 nodes itself, and `test_navigation_launch.py` fails if any of them ever aims at `/cmd_vel`.

Recovery behaviours go through the smoother too. A backup that stepped around the acceleration
limits would spin the wheels, and spinning wheels on regolith is how a rover buries itself.

### The message type is the other half of it

`twist_mux` runs `use_stamped: true`, because Jazzy's `diff_drive_controller` subscribes to
`TwistStamped`. Nav2 defaults to plain `Twist`, so all three velocity-publishing nodes set
`enable_stamped_cmd_vel: true`.

Get this wrong and the topics still connect, nothing errors, and the rover does not move. See
[`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md), which budgets an afternoon for it.
`test_nav2_params.py` pins the Nav2 side and the `twist_mux` side together so changing one alone
fails in CI.

> **VERIFY** `enable_stamped_cmd_vel` against the installed Nav2. It arrived in Jazzy and is
> expected to disappear again once stamped commands are the only option.

---

## What runs, and what does not

| Node | Package | Why |
|---|---|---|
| `controller_server` | `nav2_controller` | DWB local planner |
| `planner_server` | `nav2_planner` | NavFn global planner |
| `behavior_server` | `nav2_behaviors` | spin, backup, wait |
| `bt_navigator` | `nav2_bt_navigator` | the behaviour tree |
| `velocity_smoother` | `nav2_velocity_smoother` | the only publisher on `/cmd_vel_nav` |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | brings the five up, in that order |

Declaration order in `NAV2_NODES` **is** activation order, and `LIFECYCLE_NODES` is derived from
it — a node this file starts cannot be one the manager forgets. An unmanaged lifecycle node sits
in `unconfigured` forever, publishing nothing and reporting nothing.

Deliberately absent:

- **AMCL** and **`map_server`**. `rtabmap` owns `map → odom` and publishes `/map`; the global
  costmap's static layer consumes it directly. Running Nav2's localisation stack too would put two
  publishers on `map → odom` and give a robot that teleports between two beliefs about where it
  is. AMCL keeps a parameter block for documentation and is tested to never be started.
- **`smoother_server`** and **`waypoint_follower`**. Nothing needs them yet.
- **`use_composition`**. The composable variants need the same remappings stated a second time,
  and duplicating the one thing this file exists to get right is how the copies drift apart. Worth
  revisiting on the Jetson, where serialising costmaps between processes actually costs something.

---

## What will need tuning first

In roughly the order the problems will show up.

**The odometry is fabricated.** There are no encoders; `diff_drive_controller` integrates the
velocity it *commanded*. On loose regolith the wheels slip and the reported pose drifts from the
real one immediately. DWB's motion model assumes commanded velocity is achieved, so it is planning
against a robot that does not exist. **Regulated Pure Pursuit is the obvious alternative** — it
cares about path geometry rather than a velocity-space rollout. Evaluate it before spending long
on DWB's critic weights.

**The sensor horizon is 2 m.** One forward camera, depth clipped at 2 m (`i_max_range: 2000` in
`oak_d_s2.yaml`), tilted down. The costmap's `obstacle_max_range` and `raytrace_max_range` cannot
exceed that, and a test enforces it: claiming more range marks obstacles from noise and, worse,
*clears* cells that were never observed. The local costmap window is deliberately larger (4 m) so
the rover keeps a short memory of obstacles behind it — it cannot see backwards at all.

**Spin recovery is actively harmful here.** Rotating in place scrubs the wheels, throws up dust and
produces the worst odometry the estimator will ever see. It is configured because Nav2's default
tree calls it; consider removing it from `behavior_plugins` once there is field data.

**Traversability is not binary.** A slope the rover climbs empty may be impassable loaded. The
costmap has one obstacle/free distinction and no notion of cost-to-traverse. That is a real
modelling gap, not a tuning parameter.

---

## Testing

### In CI

`test_nav2_params.py` is pure YAML and one regex over `properties.xacro`. It runs both under
`colcon test` and standalone in the lint job, and it checks the things `nav2_params.yaml` restates
from elsewhere:

| Restated value | Checked against |
|---|---|
| Costmap footprint | `chassis_length` × `chassis_width` in `properties.xacro` |
| Velocity and acceleration limits | `diff_drive_controller` in `controllers.yaml` |
| Observation ranges | `i_max_range` in `oak_d_s2.yaml` |
| `enable_stamped_cmd_vel` | `use_stamped` in `twist_mux.yaml` |
| Every node's `use_sim_time` | present at all — `RewrittenYaml` replaces keys, it does not add them |

`test_navigation_launch.py` constructs the launch description and asserts the wiring: nothing
publishes to `/cmd_vel`, exactly one node publishes `/cmd_vel_nav`, the smoother's input matches
what feeds it, and the controller and planner activate before `bt_navigator`.

Neither test proves Nav2 navigates. They prove it is wired to the rest of the workspace the way
the contract says.

### On a machine with ROS 2 Jazzy

The first-run checklist, in order. Each line has caught something in an equivalent stack before.

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock slam:=rtabmap nav:=true rviz:=true

# 1. exactly ONE publisher, and it is twist_mux
ros2 topic info /cmd_vel -v

# 2. TwistStamped, published by velocity_smoother
ros2 topic info /cmd_vel_nav -v

# 3. all five active, not just configured
ros2 lifecycle get /controller_server /planner_server /behavior_server \
                   /bt_navigator /velocity_smoother

# 4. the map frame exists at all
ros2 run tf2_ros tf2_echo map base_link
```

Then send a goal from RViz's **2D Goal Pose** tool and watch `/cmd_vel_nav`. The last check is the
one people skip: **hold the teleop deadman while Nav2 is driving** and confirm the rover obeys the
stick instead. If it does not, the mux is not in the path, and everything above is decoration.

---

## Troubleshooting

| Symptom | Look at |
|---|---|
| Every goal fails with a TF timeout | `slam:=` was not set; nothing publishes `map → odom` |
| Nav2 plans, `/cmd_vel_nav` publishes, rover still | `use_stamped` / `enable_stamped_cmd_vel` mismatch |
| Rover ignores the teleop deadman | something else is publishing `/cmd_vel`; check `ros2 topic info /cmd_vel -v` |
| Costmaps never update under `hw:=sim` | a parameter block missing `use_sim_time`; `RewrittenYaml` only replaces keys that exist |
| Robot reported stuck in open space | costmap clearing past the 2 m depth clip, or the footprint padding |
| A node stays `unconfigured` | it is started but not in `LIFECYCLE_NODES` — which the tests forbid, so check the manager actually started |

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
  controller_server ─▶ /cmd_vel_nav_unlimited ─▶ curvature_limiter ─┐
                                                                    ├─▶ /cmd_vel_nav_unsmoothed ─▶ velocity_smoother
  behavior_server  ─────────────────────────────────────────────────┘                                      │
                                                                                                           ▼
                                                                                                     /cmd_vel_nav
                                                                                                           │
    teleop ──/cmd_vel_joy──▶ twist_mux ◀─────────────────────────────────────────────────────────────────────┘
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
| `controller_server` | `nav2_controller` | Regulated Pure Pursuit, reversing allowed, never rotates in place |
| `curvature_limiter` | `lunabot_navigation` | caps the commanded turn at radius ≥ 1 m; not a lifecycle node |
| `planner_server` | `nav2_planner` | Smac Hybrid-A*, Reeds-Shepp, minimum turning radius 1.5 m |
| `behavior_server` | `nav2_behaviors` | backup, wait — **no spin** |
| `bt_navigator` | `nav2_bt_navigator` | `config/navigate_car_like.xml` and `navigate_through_poses_car_like.xml` |
| `velocity_smoother` | `nav2_velocity_smoother` | the only publisher on `/cmd_vel_nav` |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | brings the five Nav2 servers up, in that order |

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

**The rover turns on the spot — because it has six wheels.** The 2026 four-wheeler could not
pivot in sand at all, only drive wide arcs, and the 2027 four-wheel design has nearly the same
proportions (Isaac's sand model reproduces it: `lunabot_sim/robot/soil.py`). Widening the track
enough (~0.9 m) breaks the 0.75 m starting width, so on 2026-10-22 the rover became a **six-wheel
drop-centre**: a middle axle at the same 0.62 m wheelbase, `wheel_drop` (1.5 cm, assumed) lower.
It rocks onto the middle axle, whose wheels sit on the pivot axis and scrub nothing. Isaac:
~99% of the commanded yaw rate, pivots and arcs alike (rigid ground flatters it — sinking sand
loads the end wheels more; the planar model's 50%-middle case gives 85–93%).

So Nav2 is the ordinary differential-drive setup:

- **Smac State Lattice** planner with Nav2's diff-drive primitives (arcs, straights, rotations in
  place, forward only), checking the real rectangular footprint at every step — it plans a pivot
  only where one fits. (Smac 2D, a point planner, routed into gaps the rover could only enter by
  pivoting where a pivot did not fit.)
- **Regulated Pure Pursuit** with `use_rotate_to_heading`: turn in place to face the path, then
  drive forward, so the forward camera looks where the rover goes. No reversing.
- **Spin** recovery is back (it is how the forward camera looks around); BackUp 15 cm, blind.
- **Initial survey** (`scripts/initial_survey.py`, on by default, `survey:=false` to skip): once
  Nav2 is active the rover turns one full circle on the spot in the start zone, through Nav2's Spin
  (0.4 rad/s, collision-checked), then latches `/navigation/survey_done` (wait for it before sending
  goals). The costmaps then start with the walls and rocks behind the rover. It does **not** make
  rtabmap close the loop: on the return every visual attempt found 45–74 matches and **zero**
  inliers (random regolith texture). It did not need to — the ~0.6 m return-leg error was the sim
  IMU (see SIM_ISAAC.md, "The IMU is mounted at oak_d_imu_frame"); fixed, the return leg ends
  ~0.07 m off. ICP proximity on the floor-stripped depth cloud was tried and gave the same 0.07 m,
  so it was not kept.
- **`curvature_limiter`** stays in the chain at radius 0 (pass-through). If field tests show the
  pivot does not hold, the four-wheel car-like setup is one commit back (`c1b4379`: Hybrid-A*
  with Reeds-Shepp at 1.5 m, no rotate-to-heading, limiter at 1 m), and
  `test_turning_in_place_is_all_or_nothing` keeps either setup self-consistent.
- The behaviour trees (`config/navigate_to_pose.xml`, `navigate_through_poses.xml`) replan every
  2 s or when the path goes invalid — the first plan is made before most rocks are visible — and
  call only behaviours `behavior_server` runs (tested; a missing server stops Nav2 coming up).
- **Footprint: the real 0.92 × 0.644 m box**, not the 0.59 m circle a pivot sweeps. The circle made
  the rover 1.18 m wide to the planner and the UCF lane's rock field impassable. Pivots are made safe
  by RPP's collision check, which projects the footprint through every rotate-to-heading.
- The global costmap is a 16 m **rolling window** with a **VoxelLayer**: sized by rtabmap's map
  it rejected goals across the arena (error 204), and a 2D obstacle layer erased low boulders.

**Costmaps and low rocks — three traps, all hit in Isaac, all tested now.**

1. *Heights are from `base_link`, the axle, 0.15 m up — not from the ground.* Both the voxel layer's
   `origin_z` and `min_obstacle_height` must sit below the axle. And `min_obstacle_height` exists
   **twice** — per observation source and per layer — with the layer's default of **0.0** silently
   overriding the source's. Every rock under ~20 cm was invisible; the rover drove into them.
2. *Clearing erases low rocks.* Rays to the ground just behind a low rock pass through the voxel
   holding its top. Once the rock drops into the camera's near blind zone (~0.65 m from the rover's
   centre) nothing re-marks it, and it vanishes — from the planner (global) and from RPP's
   collision check (local). Neither costmap clears now: arena rocks only move when pushed. On real
   hardware depth-noise speckles will persist inside the 4 m local window; watch for that.
3. *Inflation must reach past the footprint's farthest corner* (0.60 m), or a pivot gets no
   warning before a corner touches.
4. *Rocks are thin shells, and unseen space looks free.* See "Rocks behind their seen faces"
   below before working on clearance — including what NOT to rebuild.

**The odometry is fabricated.** There are no encoders; wheel speed stands in for ground speed. In
sand the wheels slip, worst at cusps and in stalls. So with `odom_source:=ekf` and rtabmap, the
EKF also fuses **visual odometry** (`/odom/visual`: forward speed and yaw rate from
`rgbd_odometry`, which does not slip) — see docs/SLAM.md. On the arena mission that took the pose
error from ~0.25–0.8 m to ~0.03 m on the first leg.

**The sensor horizon is 2 m.** One forward camera, depth clipped at 2 m (`i_max_range: 2000` in
`oak_d_s2.yaml`), tilted down. The costmap's `obstacle_max_range` and `raytrace_max_range` cannot
exceed that, and a test enforces it: claiming more range marks obstacles from noise and, worse,
*clears* cells that were never observed. The local costmap window is deliberately larger (4 m) so
the rover keeps a short memory of obstacles behind it — it cannot see backwards at all.

**Both behaviour trees must be ours.** `bt_navigator` loads the through-poses tree at configure
too, and a tree that calls a behaviour `behavior_server` does not run stops Nav2 coming up at all.

**Traversability is not binary.** A slope the rover climbs empty may be impassable loaded. The
costmap has one obstacle/free distinction and no notion of cost-to-traverse. That is a real
modelling gap, not a tuning parameter.

---

## Rocks behind their seen faces — READ before changing obstacle handling

**Symptom (Isaac seed 7, 2026-10-23):** the rover passes boulders at 0–4 cm and sometimes touches
them. Plans themselves cross the true rock, on its far side.

**Cause:** the camera only marks the faces of a rock turned towards it, so each rock is a thin
lethal *shell* in the costmaps (a 0.33 m cube: 2 of its 45 cells lethal). Behind the shell nothing
was ever seen, but nothing says so either. `track_unknown_space` is on in the global costmap,
yet clearing is off (trap 2 above), so no cell is ever marked free: unseen and seen-free look the
same, and Smac (`allow_unknown: true`) plans through both. **The planner cannot tell space it has
seen to be empty from space it has never seen.** That is the problem to fix.

**Dead end: guessing the rest of the rock. Do not rebuild this.** A node completed each
`/perception/boulders` detection into a guessed whole-rock footprint, marked lethal (kept on the
local branch `abandoned/rock-footprints`, commit `78eb3e4`, never pushed). It needed:
- rules to tell boulders from walls, the berm and bumps (size, elongation, height cut-offs);
- a shape prior (cubes and spheres, ≤ 0.40 m: "as deep as wide"), wrong for elongated rocks;
- special handling for detections cut off by the edge of the view (seen 0.14 m wide);
- a correction for the detector's axis-aligned box seen at an angle (10 cm short).

Each fix exposed the next guess, and the detector is a self-declared geometric placeholder with
no classifier, so the guesses rest on nothing. Results over 12 runs: closest pass to the worst
cube −1 to +11 cm (still touching in 2 runs), the return leg slowed from ~32 s to 110–136 s by
over-marking, and **the start survey's Spin failed** on some runs. Wall pieces completed into
"rocks" put a lethal cell inside the circle the corners sweep in a pivot. That circle is 0.60 m
from the rover's centre, plus 0.03 m of padding, and the back wall is only 0.75 m from the start
pose. Any marking scheme that puts cost in front of a seen surface will do this.

**Also tried and rejected:** `footprint_padding: 0.06`. It kept seed 7 off the rocks only by
closing its 0.9 m gaps (one leg took 158 s of replanning). A bigger margin hides the problem.

**Recommended direction (not yet tried): plan on what has been seen.**
- Mark a cell *seen-free* only where the camera actually sees floor points. That is direct
  evidence of free ground, unlike raytracing through obstacles, which erased low rocks (trap 2).
  Obstacles stay marked and are never cleared.
- Make the planner prefer seen-free cells, and avoid or penalise unknown cells near the rover,
  while still allowing unknown far away (goals lie beyond the 2 m camera range). Open question:
  how Jazzy's Smac weighs a cost-255 unknown cell with `allow_unknown: true`. The installed
  package has only headers, so read the nav2_smac_planner source before choosing.
- It is shape-agnostic, and it also covers **craters**: a forward camera can't see into a
  crater, and no boulder rule would ever mark one.

**Measuring clearance — lessons:**
- One run proves nothing. Identical runs differed by up to 12 cm of clearance, so compare ≥ 3
  per variant.
- Measure against the exact rock shape from the sim's ground truth (sphere = circle, cube =
  rotated square). Half-diagonal circles overstate a cube by up to 8 cm.
- Check the *plans* as well as the driven track. A plan crossing the true rock means the costmap
  was wrong; a clean plan with a touching track means tracking.
- Placing one depth cloud in the world with the true pose against the back wall (x = −0.75)
  put it at −0.750 m: the sim's depth and camera mount are correct, so don't chase a sensor
  offset again.

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

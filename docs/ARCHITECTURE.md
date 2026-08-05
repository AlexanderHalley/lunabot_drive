# Architecture

## Why this rebuild exists

The 2026 robot worked, but its shape blocked everything the 2027 robot needs:

- `drive_node.cpp` took `cmd_vel` and wrote **duty cycle** straight to four SparkFlex controllers.
  Duty cycle is not m/s, so `cmd_vel` had no physical units and no controller could be layered on
  top of it.
- It never read encoders, so it published **no odometry** — and both SLAM and Nav2 require an
  `odom → base_link` transform to exist before they can do anything at all.
- There was no URDF. The TF tree was faked with four `static_transform_publisher` calls rooted at
  a frame called `world`, which is not a REP-105 frame.
- There was no simulation, so every change had to be tested on the physical robot.

Those are not bugs to patch; they are a missing foundation. Hence the rebuild.

## The one idea worth remembering

**Sim and real run the same stack.** Not a similar stack — the same nodes, the same YAML, the same
kinematic constants, the same URDF bytes. The only thing that swaps is the `ros2_control` hardware
plugin at the very bottom.

```
                    ┌───────────────────────────────────────────────┐
                    │              IDENTICAL EVERYWHERE             │
                    │                                               │
   /cmd_vel ───────▶│  diff_drive_controller ──▶ /odom, odom→base   │
                    │  joint_state_broadcaster ──▶ /joint_states    │
                    │  robot_state_publisher ──▶ base_link→*        │
                    │  rtabmap / cuvslam ──▶ /map, map→odom         │
                    │  boulder_detector ──▶ /perception/boulders    │
                    └───────────────────┬───────────────────────────┘
                                        │ hardware_interface
              ┌─────────────────────────┼─────────────────────────┐
              ▼                         ▼                         ▼
     mock_components          topic_based_ros2_control     SparkFlexSystem
     GenericSystem              ↕ /isaac/joint_*            ↕ SocketCAN
     (no hardware)                Isaac Sim                 SparkFlex ×4
```

The payoff: a bug that reproduces in sim is a real bug. The cost: one extra topic hop in the sim
path, which is irrelevant at 100 Hz.

The alternative — having Isaac subscribe to `/cmd_vel` directly and run its own differential
controller — is much easier to stand up and quietly worthless. Kinematics would be computed twice
from two sets of constants, `/odom` would come from a different place in sim than on the robot, and
sim would stop being a test of anything.

## Node graph

```
  joy_node ──/joy──▶ teleop_twist_joy ──/cmd_vel_joy──┐
                                                       ├──▶ twist_mux ──/cmd_vel──┐
  nav2 ────────────────────────────/cmd_vel_nav───────┘                            │
                                                                                   ▼
                                                              ┌──────── controller_manager ────────┐
                                                              │  diff_drive_controller             │
   robot_state_publisher ◀──/joint_states── joint_state_broadcaster                                │
        │                                                     │  [hardware plugin: mock|sim|real]  │
        │                                                     └────────────────┬───────────────────┘
        ▼                                                                      │
   /tf: base_link → wheels, oak_d_*                              /odom, /tf: odom → base_link
        │                                                                      │
        └──────────────────────────┬───────────────────────────────────────────┘
                                   ▼
   OAK-D S2 ──/oak_d/rgb, /oak_d/stereo, /oak_d/imu ──▶ rtabmap ──▶ /map, /tf: map → odom
        │
        └──/oak_d/points──▶ boulder_detector ──▶ /perception/boulders
```

## Design decisions and their reasons

### `base_link` is the TF root, `base_footprint` hangs off it

The common convention makes `base_footprint` the root. We invert it so that **exactly one node
publishes `odom → base_link`** and there is never an argument about who owns `base_footprint`.
Nav2's `robot_base_frame` stays `base_link`.

### Only one node may publish each transform

| Transform | Owner |
|---|---|
| `map → odom` | the SLAM backend, whichever one is running |
| `odom → base_link` | the odom source — selected by `odom_source:=wheel\|visual\|ekf` |
| `base_link → *` | `robot_state_publisher`, from the URDF |

`odom_source` exists specifically to keep `diff_drive_controller`, cuVSLAM and `ekf_node` from
fighting over `odom → base_link`. Two publishers on one transform produces a TF tree that looks
fine in `view_frames` and behaves nondeterministically.

### Isaac Sim does not publish TF

Isaac publishes joint states, sensor data and a **ground-truth pose on its own topic**, and nothing
else. TF comes from `robot_state_publisher` and `diff_drive_controller` exactly as on the robot. If
you let Isaac's `ROS2PublishTransformTree` publish the articulation, you get two publishers on the
same transforms and a tree that only works in sim — which defeats the entire point.

Ground truth goes to `/sim/ground_truth/odom`, never to `/tf`. It is for scoring, not for
navigating.

### Boulders and craters share one message type

`vision_msgs/Detection3DArray` with `class_id` as a string. Adding crater detection later is then a
new string value, not a new message, not a downstream rebuild. This is also why the ground
segmentation stage emits **three** clouds — `ground`, `above_ground`, `below_ground` — rather than
the obvious two: boulders are the above-ground clusters, craters are the below-ground ones. That
three-way split costs nothing today and is the difference between adding craters and rewriting.

### The odometry is known-bad, on purpose, for now

The 2026 drivetrain has no encoders. `SparkFlexSystem` therefore runs with
`use_motor_feedback: false`, where `read()` echoes the commanded velocity back as measured. The
resulting `/odom` has the right topology, the right frames and the right units — and it is wrong
under any wheel slip, which on regolith is always.

This is deliberate and documented rather than hidden, and it is why `robot_localization` ships in
the skeleton disabled rather than being deferred: wheel odometry is known-bad *before* the first
test, so the fusion path needs to be wired and exercised in sim from day one. When encoders exist,
`use_motor_feedback: true` is a one-line change.

## Where things live

```
src/lunabot_description/urdf/lunabot.urdf.xacro   the robot, one file, three targets
src/lunabot_bringup/launch/robot.launch.py        the only launch file you type
src/lunabot_bringup/config/controllers.yaml       kinematic constants live here, once
docs/TOPIC_FRAME_CONTRACT.md                      the authority on names
```

The kinematic constants appearing in exactly one file is not an accident. `wheel_separation` and
`wheel_radius` are read by `diff_drive_controller` and nothing else; the URDF's geometry is for
visualisation and collision. If those two ever disagree, odometry is wrong and nothing will tell
you.

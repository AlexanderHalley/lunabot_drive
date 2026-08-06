# Isaac Sim

The scene is built **programmatically at startup** — terrain, seeded boulders, low-angle lighting,
and the ROS 2 bridge graphs. No `.usd` is committed. The robot is defined by the same
`lunabot.urdf.xacro` the real rover uses, expanded at launch.

That is a deliberate trade: a committed USD would start faster and would silently diverge from the
xacro the first time someone changed a wheel diameter. `.gitignore` blocks `*.usd*`.

---

## Running it

> **First time on this machine?** Run
> [`scripts/probe_isaac_api.sh`](../src/lunabot_sim/scripts/probe_isaac_api.sh) before anything
> else and work through [`SIM_ACCEPTANCE.md`](SIM_ACCEPTANCE.md). Every version-sensitive string
> below is unverified until that probe has passed once, and each one fails at a different point in
> startup with an error that names the symbol rather than the rename.

**Two processes, Isaac first.**

```bash
# Terminal A
src/lunabot_sim/scripts/run_isaac_sim.sh --seed 7

# Terminal B, once Isaac is up
ros2 launch lunabot_bringup sim.launch.py
```

`sim.launch.py` blocks on `/clock` before starting anything, because the failure otherwise is
silent: with `use_sim_time:=true` and no `/clock`, **every ROS node blocks at time zero** — no
error, no log line, just a stack that appears hung. If that wait times out, Isaac is not running.

Set `ISAAC_SIM_PATH` if the script cannot find `python.sh`:

```bash
export ISAAC_SIM_PATH=$HOME/isaacsim
```

---

## Why two processes

Isaac's embedded Python is not the ROS distro's Python. `lunabot_sim` **never imports `rclpy` or
`xacro`** — all ROS traffic goes through the bridge extension, which links its own DDS.

`run_isaac_sim.sh` is the seam. It runs `xacro` in the ROS environment, writes a plain URDF to
`~/.cache/lunabot_sim/`, then `exec`s Isaac's Python on `run_sim.py` with that file. A file across
the boundary works; a shared interpreter does not.

Two useful side effects: Isaac can be restarted without tearing down the ROS graph, and
`lunabot_sim`'s layout tests run in ordinary CI on a machine with no GPU.

---

## The two decisions that keep sim honest

### 1. Isaac does not publish TF

It publishes joint states, sensor data, and a ground-truth pose on its own topic. Nothing else.

`robot_state_publisher` builds `base_link → *` from the same URDF in sim and on hardware.
`diff_drive_controller` publishes `odom → base_link` in both. If Isaac's
`ROS2PublishTransformTree` also published the articulation, there would be two publishers on the
same edges and a TF tree that works **only in sim** — so every TF bug would stay invisible until
the rover was on regolith.

Ground truth goes to `/sim/ground_truth/odom` and **never** into `/tf`. It is for scoring, not
navigating.

### 2. The control stack is the real one

`topic_based_ros2_control` bridges `ros2_control` to Isaac over two `JointState` topics. The
identical `controller_manager`, `joint_state_broadcaster` and `diff_drive_controller` run, with the
identical `controllers.yaml`.

The easier alternative — Isaac subscribing to `/cmd_vel` and running its own differential
controller — would compute the kinematics twice from two sets of constants, and `/odom` would come
from a different place in sim than on the robot. Sim would stop being a test of anything.

---

## What the scene contains

**Terrain** — a flat ground plane with a regolith physics material. The friction coefficients
(0.7 static, 0.6 dynamic) are the most consequential numbers in the whole sim: **skid-steer turning
is controlled friction failure**, so these determine whether a `wheel_separation_multiplier` tuned
in sim means anything on regolith. Calibrate them against measured turning on the real rover.

**Boulders** — seeded rejection sampling, `numpy.random.default_rng(seed)`. Same seed, same scene,
every time. That determinism is what makes "did that change help?" answerable at all; evaluating a
perception change against a different random scene tells you nothing.

Rocks are irregular primitives, not meshes — a rock's contact behaviour is a friction question, not
a geometry one, and convex meshes cost the solver dearly for detail nobody measures. Large ones are
static; small ones can be shoved.

**Lighting** — this is the part that actually breaks perception, which is why it is in the skeleton
rather than being polish. A single distant light at ~8° elevation with the sun's true 0.53° angular
diameter, and near-zero ambient. On an airless body there is no atmospheric scattering, so shadows
are **genuinely black**: a boulder's shadow is a hole in the point cloud, not a dark patch in it.

A detector tuned under a default dome light will look excellent and then fail in the arena. Vary
`sun_azimuth_deg` between runs — a detector that only works with the sun behind the rover is not a
detector.

**Craters are not simulated.** Out of scope for now. `TerrainConfig.features` exists and
`terrain.build()` raises rather than silently ignoring a non-empty list, so the interface that will
carry craters is already the interface that carries nothing.

---

## Ground truth and scoring

Each run writes `ground_truth_seed<N>.json` next to the expanded URDF: every boulder's true pose
and extent.

The same data is published on `/sim/ground_truth/boulders` **in the same
`vision_msgs/Detection3DArray` type the detector emits**, so scoring is a direct comparison rather
than a format conversion. That symmetry is why the sidecar exists.

For odometry: display `/odom` and `/sim/ground_truth/odom` together in RViz (`slam.rviz` has both).
**The gap between the two trails is the odometry error** — the most direct read there is on whether
a change helped.

---

## Version sensitivity

Isaac Sim 4.5 renamed essentially every namespace:

| | ≤ 4.2 | 4.5 / 5.x |
|---|---|---|
| SimulationApp | `omni.isaac.kit` | `isaacsim` |
| World | `omni.isaac.core` | `isaacsim.core.api` |
| ROS 2 bridge | `omni.isaac.ros2_bridge` | `isaacsim.ros2.bridge` |
| URDF importer | `omni.importer.urdf` | `isaacsim.asset.importer.urdf` |
| IMU sensor | `omni.isaac.sensor` | `isaacsim.sensors.physics` |

Everything version-sensitive is resolved in **`lunabot_sim/compat.py`**, once, at startup — and
when resolution fails the error names every candidate tried. `ISAAC_VERSION` is logged on every
run, because the first question about any Isaac bug is which version produced it.

> **Treat every one of those strings as unverified.** The direction of the renames is right; the
> exact OmniGraph node types and the URDF importer's Python entry point changed more than once.
> To pin them down in a running session: read node types from the OmniGraph editor's property
> panel, and extension IDs from Window → Extensions. Fix them in `compat.py` and nowhere else.

**Also confirm before committing to a version:** that your Isaac Sim build supports Ubuntu 24.04
with a ROS 2 Jazzy bridge. The bridge ships prebuilt internal ROS libraries per distro. Landing on
a 22.04-only build means revisiting the Jazzy decision.

---

## Known rough edges

**The rover does not move at all, and nothing errors.** The first thing to check, and it is a
configured parameter rather than a mystery. `TopicBasedSystem::write()` skips publishing when the
position command and the position state are within `trigger_joint_command_threshold` of each other.
This drivetrain declares a velocity command interface and no position one, so the position command
stays at the 0.0 it was initialised with; at rest the state is 0.0 too, and the default threshold of
1e-5 makes the skip permanent. Nothing reaches Isaac, so the wheels never turn, so the position
never changes.

`lunabot.ros2_control.xacro` sets that threshold **negative**, which makes the early return
unreachable. If the rover sits still under `/cmd_vel` with a healthy graph, check that the parameter
survived, and check `ros2 topic hz /isaac/joint_commands` — silence there is this.

**The rover snaps to a wheel angle and stops instead of driving.** Anticipated, and now known not to
happen with this URDF. `ROS2SubscribeJointState` picks position or velocity targets by which arrays
in the incoming `JointState` are non-empty; `TopicBasedSystem::write()` pushes an array only for the
command interfaces a joint declares, and this one declares velocity alone, so the position array
arrives empty. `test_sim_bringup.py` asserts that, so a change that makes it untrue fails in CI.

Two defences remain in place because they cost nothing and they turn a future position command
interface into a slow rover rather than a broken one: wheel drives have **zero stiffness** (a drive
that cannot hold a position cannot obey a position target), and `positionCommand` is deliberately
left unconnected in `graphs/joints.py`.

**Point cloud density differs from hardware.** On the robot the cloud is built by
`depth_image_proc` with decimation 4 and a 2 m clip; in sim the camera helper produces it directly.
Same topic, different density. If cluster tuning transfers poorly between sim and hardware, suspect
this first.

**Depth encoding differs.** Isaac emits `32FC1` metres, the OAK-D emits `16UC1` millimetres. This
is why the boulder detector consumes the point cloud rather than the depth image, and why
`depth_normalizer_node` runs on hardware only.

**No camera prim, import fails.** `importer.py` merges fixed joints, so the URDF's optical frames
may not survive as prims. Either create a `Camera` prim explicitly at the right pose, or set
`merge_fixed_joints` to false and pay the solver cost. The `frame_id` on the image topics comes
from the graph configuration, not from the prim name, so only the pose matters.

**Real-time factor below ~0.3 makes teleop unusable.** `controller_manager` runs at 100 Hz against
the simulated clock, so everything scales together and stays correct — it is just unpleasant to
drive. Reduce the render resolution or run `--headless`.

---

## Testing

```bash
colcon test --packages-select lunabot_sim lunabot_bringup
```

Three things run with **no GPU and no Isaac install**, and between them they cover everything about
the sim path except Isaac itself:

**Scene layout** (`lunabot_sim/test/test_scene_layout.py`) — determinism under a seed, boulders
inside the arena, nothing in the start zone, minimum separation held, rocks resting on the ground
rather than floating or half-buried, ground truth round-tripping through JSON. Possible because
`scene/boulders.py` imports nothing but numpy.

**The probe's inventory** (`lunabot_sim/test/test_probe.py`) — parses `graphs/*.py` with `ast` and
fails if `probe.py`'s list of OmniGraph node types drifts from what the code actually creates, in
either direction. A probe that silently forgets a node type is worse than no probe.

**The whole ROS stack on `hw:=sim`** (`lunabot_bringup/test/test_sim_bringup.py`) — brought up
through `sim.launch.py` against `test/isaac_double.py`, which publishes Isaac's ROS surface
(`/clock`, `/isaac/joint_states` integrated from `/isaac/joint_commands`, ground-truth odom) and
nothing else. It asserts the acceptance criterion this document used to state in prose: the **same
two controllers, by the same names, as `hw:=mock`**. It also covers the two places the sim path is
expected to hurt — wrapped joint positions through `sum_wrapped_joint_states`, and whether
`TopicBasedSystem` populates the position array as well as velocity.

The double is not a simulator. There is no contact, no friction and no mass, so this is a test of
the plumbing and no test at all of whether the rover can climb a slope.

### What still needs the real thing

`compat.py`, `graphs/*.py`, `robot/importer.py`, `robot/articulation.py`, and the scene builders'
Isaac imports have never executed. That is what the acceptance run is for, and it now has an order
of operations and a pass/fail criterion per step:

- **[`SIM_ACCEPTANCE.md`](SIM_ACCEPTANCE.md)** — the day-one checklist
- `src/lunabot_sim/scripts/probe_isaac_api.sh` — resolves every `VERIFY` in `compat.py` against a
  real install, in one command, before anything else is attempted
- `ros2 run lunabot_bringup check_stack.py --profile sim` — judges the live graph against
  `TOPIC_FRAME_CONTRACT.md`, and is the same tool the sim bringup test runs against the double

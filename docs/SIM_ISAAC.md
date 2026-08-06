# Isaac Sim

The scene is built **programmatically at startup** — terrain, seeded boulders, low-angle lighting,
and the ROS 2 bridge graphs. No `.usd` is committed. The robot is defined by the same
`lunabot.urdf.xacro` the real rover uses, expanded at launch.

That is a deliberate trade: a committed USD would start faster and would silently diverge from the
xacro the first time someone changed a wheel diameter. `.gitignore` blocks `*.usd*`.

---

## Running it

**Two processes, Isaac first.**

```bash
# Terminal A
src/lunabot_sim/scripts/run_isaac_sim.sh --seed 7

# Terminal B, once Isaac is up
ros2 launch lunabot_bringup sim.launch.py
```

`sim.launch.py` blocks on `/clock` before starting anything, because the failure otherwise is
silent: with `use_sim_time:=true` and no `/clock`, **every ROS node blocks at time zero** — no
error, no log line, just a stack that appears hung. If the wait times out, the launch says so and
**shuts down rather than starting the stack**, because a stack brought up without `/clock` is that
same silent hang.

Two arguments control it. `clock_timeout` defaults to 120 seconds — raise it for a first run on a
cold machine, where Isaac compiles shaders and can take several minutes:

```bash
ros2 launch lunabot_bringup sim.launch.py clock_timeout:=600
ros2 launch lunabot_bringup sim.launch.py wait_for_clock:=false   # skip the barrier entirely
```

Set `ISAAC_SIM_PATH` if the script cannot find `python.sh`:

```bash
export ISAAC_SIM_PATH=$HOME/isaacsim
```

**Before the first run, import the source dependencies** — `topic_based_ros2_control` has no Jazzy
binary, and `hw:=sim` selects it as the hardware plugin:

```bash
vcs import src < lunabot.repos && colcon build
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

**The rover snaps to a wheel angle and stops instead of driving.** The likely bug, and it is
anticipated. `ROS2SubscribeJointState` picks position or velocity targets based on which arrays in
the incoming `JointState` are non-empty, and `topic_based_ros2_control` populates both. Two
defences are already in place: wheel drives have **zero stiffness** (a drive that cannot hold a
position cannot obey a position target), and `positionCommand` is deliberately left unconnected in
`graphs/joints.py`. If it still happens, those are the two places to look.

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
colcon test --packages-select lunabot_sim
```

Covers scene layout only, and covers it thoroughly: determinism under a seed, boulders inside the
arena, nothing in the start zone, minimum separation held, rocks resting on the ground rather than
floating or half-buried, and ground truth round-tripping through JSON. All of it runs with no GPU
and no Isaac install, because `scene/boulders.py` imports nothing but numpy.

Nothing tests the Isaac-dependent code automatically. That is a real gap and it is why the manual
acceptance run matters: bring up `hw:=sim`, confirm `ros2 control list_controllers` shows the
**same two controllers with the same names as `hw:=mock`**, then teleop forward and check `/odom`
tracks `/sim/ground_truth/odom`.

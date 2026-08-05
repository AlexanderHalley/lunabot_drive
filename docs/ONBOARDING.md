# Onboarding

Get the rover driving on your machine in about twenty minutes, with no hardware and no GPU.

---

## 1. Get it building

You need **ROS 2 Jazzy on Ubuntu 24.04**. If you have neither, use the container (§6) rather than
fighting a mismatched distro.

```bash
git clone -b develop-2027 https://github.com/AlexanderHalley/lunabot_drive
cd lunabot_drive

vcs import src < lunabot.repos
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "sparkcan isaac_ros_visual_slam"
colcon build
source install/setup.bash
```

Those two skip-keys matter and are not arbitrary: `sparkcan` builds from source via
`lunabot.repos`, and `isaac_ros_visual_slam` may have no release for this distro at all. The same
list appears in CI and in the Dockerfile — they are one list wearing three hats.

---

## 2. Drive it

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock rviz:=true
```

Second terminal:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.3}, angular: {z: 0.2}}}'

ros2 topic echo /odom --field pose.pose.position
```

**If `/odom` moves, everything works.** That one observation covers the whole chain: the xacro
expanded, the URDF parsed, `robot_state_publisher` accepted it, `controller_manager` loaded the
hardware plugin, both controllers claimed their interfaces, `mock_components` integrated the
dynamics, `diff_drive_controller`'s kinematics ran, and TF is connected.

> **First thing you may hit.** If nothing moves, `/cmd_vel` may want plain `Twist` rather than
> `TwistStamped`. Check with `ros2 topic info /cmd_vel -v`, and then **write the answer into
> [`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md)** — it is marked UNRESOLVED there and you
> will be the person who resolved it.

---

## 3. Read these three files, in this order

1. **[`ARCHITECTURE.md`](ARCHITECTURE.md)** — why the 2026 code was rebuilt, and the one idea the
   whole workspace is organised around: sim and the real robot run the *same* stack, with only the
   `ros2_control` hardware plugin swapping.
2. **[`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md)** — the authoritative names. Anything
   that disagrees with it is a bug. It exists because the 2026 README and launch files drifted
   apart and nobody noticed for months.
3. Whichever of [`HARDWARE_CAN.md`](HARDWARE_CAN.md), [`SIM_ISAAC.md`](SIM_ISAAC.md),
   [`SLAM.md`](SLAM.md) or [`OAK_D_S2_INTEGRATION.md`](OAK_D_S2_INTEGRATION.md) covers what you are
   working on.

---

## 4. Three things that will bite you

**Only one node may publish each transform.** `odom → base_link` can come from
`diff_drive_controller`, the SLAM backend, or the EKF. The `odom_source:=` argument picks. Two
publishers gives a TF tree that looks perfectly correct in `view_frames` and behaves
nondeterministically — there is a test enforcing this, and it is enforcing it for a reason.

**The odometry is known-bad.** The drivetrain has no encoders, so `/odom` is the *commanded*
velocity echoed back. Right frames, right units, wrong the instant a wheel slips — which on
regolith is continuous. Do not tune anything against its accuracy. See
[`HARDWARE_CAN.md`](HARDWARE_CAN.md).

**`use_sim_time` has to reach every node**, RViz included. RViz is the one people forget, and the
symptom is not an error — it is TF displays that freeze or lag while everything else looks fine,
which reads as a TF bug and is not one.

---

## 5. Where things are

| Want to change... | Go to |
|---|---|
| Robot dimensions | `src/lunabot_description/urdf/common/properties.xacro` |
| Wheel radius / separation | **both** that file *and* `lunabot_bringup/config/controllers.yaml` |
| What `ros2 launch` does | `src/lunabot_bringup/launch/robot.launch.py` |
| CAN wiring, motor settings | `src/lunabot_description/urdf/ros2_control/lunabot.ros2_control.xacro` |
| Camera tuning | `src/lunabot_bringup/config/oak_d_s2*.yaml` |
| SLAM tuning | `src/lunabot_slam/config/` |
| The lunar scene | `src/lunabot_sim/lunabot_sim/scene/` |

The wheel radius appearing in two files is not an oversight — xacro cannot reach into a controller
YAML. Both copies carry a comment saying so. **If they disagree, odometry is wrong and nothing
reports it.**

---

## 6. Containers

If Ubuntu 24.04 is not what you have:

```bash
docker compose -f docker/docker-compose.yml run --rm ros
```

Or open the repo in VS Code and accept the devcontainer prompt.

Isaac Sim runs from a **separate** container (`docker/Dockerfile.isaac`) and talks to this one over
DDS. Both need `--network=host` *and* `--ipc=host`: discovery uses multicast, which bridged Docker
networking breaks, and CycloneDDS uses shared memory between processes on the same host. Missing
`--ipc=host` gives containers that discover each other and then exchange nothing.

---

## 7. Before you push

```bash
colcon test && colcon test-result --verbose
pre-commit run --all-files
```

CI runs the same things plus a xacro expansion for all three hardware targets. It never runs Isaac
Sim — that needs a GPU — but it does run `test_scene_layout.py`, which is why
`lunabot_sim/scene/boulders.py` imports nothing but numpy.

---

## 8. What is not finished

Search the tree for `PLACEHOLDER`, `TODO(2027)` and `VERIFY`. Those markers are the work queue, and
they are deliberately noisy. The big ones:

- **Every robot dimension is a placeholder.** Measure the 2027 chassis.
- **The boulder detector is a geometric stub** — it finds lumps above a flat plane, cannot tell a
  boulder from a berm, and reports a constant score, not a confidence.
- **cuVSLAM is scaffolding.** Blocked on the compute decision (Jetson or Pi) and on an Isaac ROS
  release supporting Jazzy.
- **Craters are not simulated or detected.** The interfaces that will carry them exist and are
  empty, which is the whole reason the ground segmentation splits three ways instead of two.
- **Every Isaac API string in `compat.py` is an unverified reconstruction.** Isaac 4.5 renamed
  every namespace; the direction is right, the exact strings need checking against a running
  install.
- **Nothing publishes `/drive/status`** yet, though the messages exist. Blocked on motor telemetry.

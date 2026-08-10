# Lunabot — 2027 competition workspace

ROS 2 Jazzy workspace for a 4WD skid-steer lunar rover: Isaac Sim simulation, visual-inertial
SLAM from a single front-mounted OAK-D S2, and boulder detection in a lunar environment.

> **Branch layout.** `main` holds the 2026 robot: a single bare ROS 2 package at the repo root,
> open-loop CAN drive plus OAK-D bring-up. `develop-2027` (this branch) is a clean rebuild as a
> multi-package colcon workspace. The two do not share a build. Last year's code stays reachable
> at `git show main:src/drive_node.cpp`.

## The idea

One control stack, three hardware backends. The URDF, the controllers, the SLAM stack and the
perception node are **identical** whether you are running on mock hardware at your desk, in Isaac
Sim, or on the real rover. Only the `ros2_control` hardware plugin swaps:

| `hw:=` | plugin | needs |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | nothing — runs anywhere |
| `sim` | `topic_based_ros2_control/TopicBasedSystem` | Isaac Sim + GPU |
| `real` | `lunabot_hardware/SparkFlexSystem` | SocketCAN + SparkFlex motors |

If a bug reproduces in sim, it is a real bug — because sim runs the same `diff_drive_controller`
with the same YAML and the same kinematic constants as the robot.

## Packages

| Package | Purpose |
|---|---|
| `lunabot_msgs` | SparkFlex telemetry messages. Nothing that exists upstream |
| `lunabot_description` | URDF/xacro, `ros2_control` macros, RViz configs |
| `lunabot_hardware` | `SparkFlexSystem` — SocketCAN hardware interface |
| `lunabot_bringup` | All launch orchestration + controller/camera/teleop/EKF config |
| `lunabot_perception` | Boulder detector (craters later) |
| `lunabot_slam` | Swappable rtabmap / cuVSLAM backends |
| `lunabot_navigation` | Nav2 params |
| `lunabot_sim` | Isaac Sim scene builder and ROS 2 bridge graphs |

## Quick start

```bash
git clone -b develop-2027 https://github.com/AlexanderHalley/lunabot_drive
cd lunabot_drive

vcs import src < lunabot.repos
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "sparkcan isaac_ros_visual_slam"
colcon build
source install/setup.bash
```

Drive the robot with no hardware at all:

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock rviz:=true
```

```bash
# second terminal
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.3}, angular: {z: 0.2}}}'
ros2 topic echo /odom --field pose.pose.position
```

If `/odom` moves, the whole stack is wired correctly.

Add SLAM and navigation on top of the same command — no hardware still required:

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock slam:=rtabmap nav:=true rviz:=true
```

`nav:=true` needs `slam:=` for the `map` frame, and Nav2 drives `/cmd_vel_nav` rather than
`/cmd_vel` so that teleop keeps the right of way. See [`docs/NAVIGATION.md`](docs/NAVIGATION.md).

## Documentation

Start here, in this order:

- **[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)** — how the pieces fit and why
- **[`docs/TOPIC_FRAME_CONTRACT.md`](docs/TOPIC_FRAME_CONTRACT.md)** — the authoritative topic and
  frame names. Anything that disagrees with this file is a bug
- [`docs/HARDWARE_CAN.md`](docs/HARDWARE_CAN.md) — SparkFlex/SocketCAN bring-up
- [`docs/SIM_ISAAC.md`](docs/SIM_ISAAC.md) — running Isaac Sim
- [`docs/SIM_ACCEPTANCE.md`](docs/SIM_ACCEPTANCE.md) — the checklist for the first bring-up on the
  simulation machine, in the order that makes each step fail for one reason
- [`docs/SLAM.md`](docs/SLAM.md) — choosing and tuning a SLAM backend
- [`docs/NAVIGATION.md`](docs/NAVIGATION.md) — Nav2, and why it never publishes `/cmd_vel`
- [`docs/OAK_D_S2_INTEGRATION.md`](docs/OAK_D_S2_INTEGRATION.md) — camera bring-up and bandwidth
- [`docs/ONBOARDING.md`](docs/ONBOARDING.md) — new team member setup

## Status

Skeleton. Every package builds and the mock stack drives, but the robot dimensions in
`lunabot_description` are placeholders, the boulder detector is a geometric stub with no
classifier — it segments against a RANSAC-fitted ground plane, so slopes are handled, but nothing
in it distinguishes a rock from any other lump — and the cuVSLAM backend is scaffolding pending an
Isaac ROS release for Jazzy. Nav2
now starts and is wired to the contract, but nothing has been tuned and no rover has followed a
path. Search the tree for `PLACEHOLDER`, `TODO(2027)` and `VERIFY` — those markers are the work
queue.

**Both hardware-free backends are exercised in CI.** `hw:=mock` and `hw:=sim` each bring the whole
stack up and drive it — the sim one against a stand-in for Isaac's bridge graphs, so the
`TopicBasedSystem` plugin, the `/isaac/*` topics, `use_sim_time` and wrapped joint positions are
all covered on a runner with no GPU. What has never executed is `lunabot_sim`'s Isaac-facing half:
`compat.py`, the OmniGraph builders, and the URDF importer. When the simulation machine is ready,
[`docs/SIM_ACCEPTANCE.md`](docs/SIM_ACCEPTANCE.md) is the order to bring it up in, and
`src/lunabot_sim/scripts/probe_isaac_api.sh` answers every `VERIFY` in `compat.py` in one command
before anything else is attempted.

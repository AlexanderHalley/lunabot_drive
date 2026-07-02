# Lunabot 2027 — Architecture & Competition Plan

> Target: **NASA Lunabotics**. Written July 2026, for a build that ramps when the
> Isaac-Sim-capable computer arrives in **September 2026**.
>
> This document is the "why". The scaffolded packages under `src/` are the "how".
> Read this first, then the per-package `README.md` files.

---

## 1. Where we are (2026 robot)

The 2026 codebase is a single package, `lunabot_drive`, doing four unrelated jobs:

| Concern | Today | Where |
|---------|-------|-------|
| Drivetrain | Differential-drive math **hardcoded in C++** | `drive_node.cpp` |
| Motor I/O | SparkFlex over CAN (`sparkcan`) | `drive_node.cpp` |
| Teleop | `joy` + `teleop_twist_joy` → `cmd_vel` | `pc_teleop.launch.py` |
| Perception | OAK-D S2 → RGB/depth/pointcloud | `oak_d_*.launch.py` |

It works, and it is a solid hardware bring-up. But it has three structural limits
that block everything we want next year:

1. **The drivetrain is not swappable.** The forward/inverse kinematics live inside
   `cmd_vel_callback`. Switching from differential to tank/skid-steer or mecanum
   means rewriting the node, not changing a config.
2. **There is no robot model.** No URDF/xacro, no TF tree, no joints. Isaac Sim has
   nothing to import, and no autonomy component (localization, Nav2, costmaps) can
   run because they all depend on TF and a described robot.
3. **`cmd_vel` only ever comes from a human.** There is no odometry, no
   localization, no map, no planner. "Autonomy" has no foundation to stand on.

The good news: **all three problems have the same fix.** Introduce a real robot
description (URDF) driven through `ros2_control`. That single abstraction is what
makes drivetrains swappable, what Isaac Sim consumes, and what the autonomy stack
plugs into. We are not throwing away the 2026 work — the CAN/SparkFlex logic
becomes the guts of a `ros2_control` hardware plugin.

---

## 2. The core idea: one abstraction, three payoffs

```
        ┌──────────────────────── AUTONOMY ─────────────────────────┐
        │  Nav2 (planner, controller, behavior tree)                 │
        │  robot_localization EKF  ·  SLAM / map                     │
        └───────────────┬───────────────────────────────────────────┘
                        │  /cmd_vel   (from a human OR from Nav2)
                        ▼
        ┌──────── twist_mux (priority: e-stop > teleop > nav) ───────┐
        └───────────────┬───────────────────────────────────────────┘
                        │  /cmd_vel  ->  active drive controller
                        ▼
        ┌──────────────── ros2_control controller_manager ──────────┐
        │  diff_drive_controller  |  mecanum_drive_controller | ...  │  <-- SWAP HERE
        │  (kinematics live in the controller, not in our code)      │
        └───────────────┬───────────────────────────────────────────┘
                        │  joint commands / states (the hardware boundary)
          ┌─────────────┴───────────────┐
          ▼                             ▼
   REAL ROBOT hardware iface      ISAAC SIM hardware iface
   (SparkFlex/CAN plugin)         (topic/bridge or isaac ros2_control)
```

Everything **above** the `controller_manager` line is identical in simulation and
on hardware. Everything the autonomy team writes runs unchanged in Isaac Sim in
January and on the real robot in April. That is the definition of "cohesive."

### 2.1 Why `ros2_control` specifically

- **Swappable drivetrains for free.** The controllers ship with ROS 2:
  - Differential → `diff_drive_controller`
  - Tank / tracked / skid-steer → also `diff_drive_controller` (skid-steer is
    differential kinematics; tune wheel separation + slip)
  - Omni / holonomic → `mecanum_drive_controller`
  - Car-like → `ackermann_steering_controller` / `tricycle_controller`
  Switching mechanisms becomes: pick a different controller YAML + matching xacro
  drivetrain macro. **No C++ changes.**
- **Sim/real parity.** The same controller config runs against a sim hardware
  interface or the real one. You develop autonomy in Isaac Sim, then flip one
  launch arg to run on hardware.
- **It gives you odometry.** `diff_drive_controller`/`mecanum_drive_controller`
  publish `/odom` + the `odom→base_link` TF from wheel encoders automatically —
  the first ingredient autonomy needs, which we have zero of today.

---

## 3. Target repository layout

The repo becomes a colcon workspace (`src/` holds many small packages, each with
one job). The 2026 package is preserved as `src/lunabot_drive` for reference and
hardware notes while we migrate.

```
lunabot_drive/                     # repo root = colcon workspace
├── docs/
│   └── NEXT_YEAR_PLAN.md          # this file
└── src/
    ├── lunabot_drive/             # 2026 code, preserved (reference / donor)
    ├── lunabot_description/       # URDF/xacro, meshes, ros2_control tag, TF
    ├── lunabot_control/           # controller YAMLs, twist_mux, teleop, joystick
    ├── lunabot_hardware/          # SparkFlex/CAN ros2_control SystemInterface
    ├── lunabot_simulation/        # Isaac Sim assets, bridge, sim launch
    ├── lunabot_perception/        # OAK-D driver, pointcloud->costmap, AprilTags
    ├── lunabot_navigation/        # Nav2 params, EKF, SLAM, mission behavior tree
    └── lunabot_bringup/           # top-level launch + the ONE config that picks drive_type
```

Each package's responsibility is documented in its own `README.md`.

**New workflow** (was: symlink repo as a single package):
```bash
# clone into the workspace src, or treat repo root as the workspace
cd lunabot_drive
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Isaac Sim integration plan

Isaac Sim (5.x) speaks ROS 2 (Humble/Jazzy) through its **ROS 2 Bridge**
(OmniGraph action graphs). Two integration styles — we use both, staged:

1. **Topic-bridge first (fastest to stand up).** Isaac publishes `/scan`,
   `/camera/*`, `/imu`, `/odom`, TF; subscribes to joint/wheel commands. The
   `controller_manager` runs outside Isaac against a lightweight
   topic-based hardware interface (`topic_based_ros2_control`). Good enough to
   start writing Nav2 autonomy immediately.
2. **`isaacsim.ros2.control` interface (higher fidelity).** Isaac hosts the
   `ros2_control` hardware component directly against the PhysX articulation, so
   the *exact same controller config* drives sim joints. This is the parity target.

### Bring-up steps (documented fully in `src/lunabot_simulation/README.md`)
1. Model the rover in `lunabot_description` (xacro → URDF).
2. Import URDF to USD via Isaac's **URDF Importer**; save `lunabot.usd`.
3. Build a **regolith arena** USD approximating the Lunabotics sandbox
   (BP-1 / obstacle field, berm/crater geometry, lighting).
4. Add the ROS 2 Bridge action graph (clock, TF, sensors, cmd/state).
5. Launch autonomy against sim exactly as against hardware.

**Hardware note for the September machine:** Isaac Sim needs an RTX GPU
(RTX 4070/4080-class or better, ≥12 GB VRAM recommended), ≥32 GB RAM, Ubuntu
22.04/24.04. Confirm the build matches your chosen ROS 2 distro (Jazzy on 24.04).

---

## 5. Autonomy stack (the Lunabotics scoring driver)

Lunabotics is **GPS-denied** on loose regolith. Autonomy points dominate. The
mission is: **traverse to dig zone → excavate → traverse back → deposit on berm**,
ideally with zero teleop. Plan the stack bottom-up:

### 5.1 Localization (hardest part on regolith)
Wheel odometry alone is unreliable — the wheels slip in BP-1. Fuse multiple
sources with `robot_localization` (EKF):
- Wheel odometry from the drive controller (weak, high-slip).
- **IMU** (already have data flowing intent from 2026 camera/IMU trials).
- **Visual-inertial odometry** from the OAK-D (e.g. RTAB-Map / stereo VO, or the
  camera's onboard VIO) — the primary trust source when wheels slip.
- **AprilTag** fiducials at the arena start/berm for absolute pose resets
  (`apriltag_ros`) — kills accumulated drift, which is where teams lose autonomy points.
- Consider adding a **2D/3D LiDAR** for robust obstacle + SLAM. Strongly
  recommended; pointcloud-only obstacle avoidance on dust is fragile.

### 5.2 Mapping / costmaps
- `nav2` costmaps fed by OAK-D pointcloud (`pointcloud_to_laserscan` or the
  `voxel`/`obstacle` layers) and/or LiDAR.
- Craters/obstacles → obstacle layer; keep-out around the berm/collector via a
  static/keepout layer.

### 5.3 Navigation
- **Nav2**: NavFn/Smac planner + MPPI or Regulated Pure Pursuit controller
  (MPPI handles skid-steer slip well). Recovery behaviors tuned for getting
  un-stuck in regolith.

### 5.4 Mission logic
- A **behavior tree** (Nav2 BT or a mission-level BT) sequencing
  `NavigateTo(dig) → Excavate → NavigateTo(berm) → Deposit`, with retries and a
  teleop-fallback branch. Scaffolded at
  `src/lunabot_navigation/behavior_trees/lunabotics_mission.xml`.

### 5.5 Excavation subsystem (separate track)
The digging/deposition mechanism is its own `ros2_control` actuator group +
controller, commanded by the mission BT. Model it in the URDF as extra joints so
Isaac Sim can simulate the dig cycle. (Not scaffolded yet — flagged as a parallel
workstream once the drivetrain foundation lands.)

---

## 6. Phased roadmap

Anchored to the September machine and a typical spring Lunabotics competition.

### Phase 0 — Now → September (no new hardware needed)
- [x] Restructure repo into the multi-package workspace (this PR).
- [ ] Model the rover in `lunabot_description` (chassis + wheels + OAK-D). Even a
      rough URDF unblocks everything downstream.
- [ ] Decide sensor suite: keep OAK-D only, or add IMU + LiDAR (recommended).
- [ ] Write `diff_drive_controller` + `mecanum` controller configs; validate the
      xacro `drive_type` switch conceptually.
- [ ] Spec the September computer against the Isaac Sim requirements in §4.

### Phase 1 — September → October: Simulation online
- [ ] Install Isaac Sim; import URDF; build a first-pass regolith arena.
- [ ] Stand up the ROS 2 Bridge (topic style). Drive the sim rover with teleop
      through `twist_mux` → `diff_drive_controller`. **Milestone: teleop in sim.**
- [ ] Confirm `/odom`, TF, `/scan`/pointcloud all publish from sim.

### Phase 2 — October → December: Autonomy in sim
- [ ] `robot_localization` EKF fusing sim odom + IMU (+ VO/AprilTag).
- [ ] Nav2 bring-up in sim; autonomous point-to-point navigation.
- [ ] Mission behavior tree: traverse → (stub) excavate → traverse → deposit.
- [ ] **Milestone: full autonomous mission runs end-to-end in Isaac Sim.**

### Phase 3 — January → March: Hardware parity
- [ ] Implement `lunabot_hardware` SparkFlex `SystemInterface` (port 2026 CAN
      logic into `read()`/`write()`). Retire `drive_node.cpp`.
- [ ] Bring up the real drivetrain through `ros2_control` — same controller YAML
      as sim. **Milestone: sim autonomy stack drives the real robot.**
- [ ] Calibrate real odometry/slip; tune EKF and Nav2 on real regolith.
- [ ] Integrate excavation subsystem on hardware.

### Phase 4 — March → competition: Harden
- [ ] Field testing on a mock arena; AprilTag relocalization; failure recovery.
- [ ] Comms/latency, e-stop, watchdog audit (carry over the 2026 watchdog).
- [ ] Dry-run the full autonomous mission repeatedly; measure autonomy uptime.

---

## 7. What we keep from 2026

- **The SparkFlex/CAN know-how** → becomes `lunabot_hardware`. Nothing wasted.
- **The watchdog + heartbeat safety pattern** → re-implement inside the hardware
  interface / a safety node. It is genuinely good and competition-critical.
- **OAK-D integration & DDS tuning** → moves into `lunabot_perception`; the
  CycloneDDS large-image config is still relevant for the distributed setup.
- **Switch Pro teleop** → moves into `lunabot_control`, now one input among
  several through `twist_mux` (teleop must always be able to override autonomy).

---

## 8. Open questions to resolve before Phase 1

1. **ROS 2 distro**: lock to **Jazzy (Ubuntu 24.04)** to match a current Isaac Sim
   build? (2026 docs already reference Jazzy.) Confirm the Isaac Sim version's
   supported bridge distro before committing.
2. **Sensor budget**: add IMU + LiDAR, or attempt OAK-D-only autonomy? This is the
   single biggest risk to autonomy scoring — recommend at least an IMU now and a
   LiDAR if budget allows.
3. **Onboard compute**: Pi 5 ran 2026. Autonomy (Nav2 + VO/SLAM) likely wants a
   Jetson Orin class board on the rover. The September machine is for *sim*, not
   the rover — plan rover compute separately.
4. **Primary drivetrain for the arena**: skid-steer/tracked is typical for
   regolith traction. The architecture supports switching, but pick a default to
   optimize first.

---

*Scaffolding for all of the above is in `src/` as of this document. The packages
are skeletons — package manifests, configs, and launch/interface stubs with TODOs —
not yet buildable robots. They exist so Phase 0/1 work has a home to grow into.*

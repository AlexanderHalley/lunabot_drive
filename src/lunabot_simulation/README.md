# lunabot_simulation

Isaac Sim is the development environment for autonomy. The goal is **parity**: the
control + navigation stack that runs here (Phases 1–2) is byte-for-byte the same
one that runs on hardware (Phase 3). Only the `ros2_control` hardware plugin
differs (`sim_mode:=isaac` vs `none`).

## Bring-up sequence
1. **Model** the rover in `lunabot_description` (xacro -> URDF).
2. **Import** to USD with Isaac Sim's *URDF Importer*; save to `isaac/lunabot.usd`.
   Set wheel joints to velocity drive.
3. **Arena**: build `isaac/regolith_arena.usd` approximating the Lunabotics
   sandbox (BP-1 surface material, obstacle field, berm/crater, overhead lighting).
4. **Bridge**: add a ROS 2 Bridge OmniGraph publishing `/clock`, TF,
   `/camera/*`, `/imu`, `/scan`, `/odom` and subscribing to joint commands.
   See `isaac/README.md` for the exact action-graph nodes.
5. **Run** `isaac_sim.launch.py` to start the ROS-side (controllers, twist_mux,
   description) against the running Isaac stage.

## Two integration stages (plan §4)
- **Stage 1 — topic bridge** (`topic_based_ros2_control`): fastest; controllers run
  outside Isaac. Good enough to start Nav2 work immediately.
- **Stage 2 — `isaacsim.ros2.control`**: Isaac hosts the ros2_control component
  against PhysX articulation for full parity. Switch by changing the hardware
  plugin in `lunabot.ros2_control.xacro` (sim_mode=isaac branch).

## Requirements
RTX GPU (>=12 GB VRAM), >=32 GB RAM, Ubuntu 22.04/24.04, Isaac Sim 5.x with a ROS 2
bridge matching your distro (target Jazzy — confirm before install).

## Status
Launch + bridge config are stubs. The USD assets are created interactively in
Isaac (not text files) — `isaac/README.md` documents how.

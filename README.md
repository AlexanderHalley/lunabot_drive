# Lunabot

ROS 2 workspace for the Lunabot rover (NASA Lunabotics).

> **Planning the 2027 build?** Start with [`docs/NEXT_YEAR_PLAN.md`](docs/NEXT_YEAR_PLAN.md)
> — the architecture and roadmap for moving to Isaac Sim, swappable drivetrains,
> and an autonomy-first stack.

## Layout

This repo is a colcon workspace. Each package under `src/` has one job:

| Package | Role |
|---------|------|
| [`lunabot_drive`](src/lunabot_drive/) | **2026 competition code** (SparkFlex/CAN teleop + OAK-D). Preserved as reference; still the working robot until the migration lands. |
| [`lunabot_description`](src/lunabot_description/) | URDF/xacro model with a **swappable drivetrain** (`drive_type:=diff\|skid\|mecanum`) and the shared `ros2_control` tag. |
| [`lunabot_control`](src/lunabot_control/) | `ros2_control` controller configs (one per drivetrain), `twist_mux` arbitration, joystick teleop. |
| [`lunabot_hardware`](src/lunabot_hardware/) | Real-robot `ros2_control` SystemInterface — SparkFlex over CAN (the 2026 drive logic, restructured). |
| [`lunabot_simulation`](src/lunabot_simulation/) | Isaac Sim integration: URDF→USD, regolith arena, ROS 2 bridge. |
| [`lunabot_perception`](src/lunabot_perception/) | OAK-D bring-up, pointcloud→costmap, AprilTag relocalization. |
| [`lunabot_navigation`](src/lunabot_navigation/) | Autonomy: `robot_localization` EKF, Nav2, mission behavior tree. |
| [`lunabot_bringup`](src/lunabot_bringup/) | Top-level launch composing everything; one config picks the drivetrain. |

## Build

```bash
cd lunabot_drive          # repo root = workspace
colcon build --symlink-install
source install/setup.bash
```

## Run (target, once packages are implemented)

```bash
# Isaac Sim, differential drive
ros2 launch lunabot_bringup sim.launch.py drive_type:=diff

# Real robot, skid-steer
ros2 launch lunabot_bringup robot.launch.py drive_type:=skid
```

## Status

The 2026 package is complete and working. The new packages (everything except
`lunabot_drive`) are **scaffolds** — manifests, configs, and interface/launch
stubs with TODOs — created to give the 2027 build a structure to grow into. See
the roadmap for the phased plan.

# lunabot_description

The single source of truth for **what the robot is**: links, joints, sensors, and
the `ros2_control` interface. Everything else (sim, control, navigation) consumes
the TF tree and URDF this package produces.

## The drivetrain switch (the whole point)

`urdf/lunabot.urdf.xacro` takes two arguments:

| arg | values | effect |
|-----|--------|--------|
| `drive_type` | `diff` \| `skid` \| `mecanum` | which drivetrain macro + wheels to include |
| `sim_mode`   | `none` \| `isaac` \| `mock` | which `ros2_control` hardware plugin to load |

```bash
# differential drive, real hardware
xacro urdf/lunabot.urdf.xacro drive_type:=diff sim_mode:=none

# mecanum, in Isaac Sim
xacro urdf/lunabot.urdf.xacro drive_type:=mecanum sim_mode:=isaac
```

Adding a mechanism = add a macro under `urdf/drivetrains/` and a matching
controller YAML in `lunabot_control`. No node code changes.

## Layout
```
urdf/
  lunabot.urdf.xacro          # top-level: chassis + selected drivetrain + sensors + ros2_control
  lunabot.ros2_control.xacro  # ros2_control tag; hardware plugin chosen by sim_mode
  drivetrains/
    diff.xacro                # 2 driven wheels + casters (differential)
    skid.xacro                # 4/6 driven wheels (skid-steer / tracked)  [TODO]
    mecanum.xacro             # 4 mecanum wheels (holonomic)               [TODO]
  sensors/
    oak_d.xacro               # OAK-D S2 links + optical frame + gazebo/isaac sensor
meshes/                       # visual/collision meshes (add real CAD exports here)
launch/description.launch.py  # robot_state_publisher (+ optional joint_state_publisher_gui)
rviz/
```

## Status
Skeleton. `diff.xacro` and the ros2_control tag are stubbed with realistic
structure and TODOs; `skid`/`mecanum` are placeholders. Meshes are empty — the
chassis is a box primitive until CAD is exported.

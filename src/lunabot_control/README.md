# lunabot_control

The control layer between `/cmd_vel` and the wheel joints. This is where a
drivetrain is *selected* at runtime — pick the matching controller YAML.

## Drivetrain = which controller YAML
| drive_type | config file | controller |
|------------|-------------|------------|
| `diff` / `skid` | `config/controllers_diff_drive.yaml` | `diff_drive_controller` |
| `mecanum` | `config/controllers_mecanum.yaml` | `mecanum_drive_controller` |

`control.launch.py` takes `drive_type:=` and loads the right one, then spawns
`joint_state_broadcaster` + the drive controller into the `controller_manager`.
The controller publishes `/odom` and the `odom→base_link` TF — the first piece of
the autonomy stack.

## Command arbitration (twist_mux)
`config/twist_mux.yaml` merges command sources by priority so **teleop always
overrides autonomy** and an e-stop overrides everything:

```
/cmd_vel_estop (255) > /cmd_vel_joy (100) > /cmd_vel_nav (10)  ->  /cmd_vel
```

## Teleop
`launch/teleop.launch.py` + `config/joystick.yaml` — the 2026 Switch Pro mapping,
now publishing to `/cmd_vel_joy` (one input among several) instead of `/cmd_vel`.

## Status
Configs are realistic starting points with placeholder gains/limits; tune on the
real drivetrain in Phase 3.

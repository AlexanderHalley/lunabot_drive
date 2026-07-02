# lunabot_hardware

The real-robot side of the `ros2_control` boundary: a `SystemInterface` plugin
that talks to SparkFlex controllers over CAN. This **replaces** the 2026
`drive_node.cpp` — the same CAN/SparkFlex logic, restructured so the
`controller_manager` (and therefore the same controllers used in Isaac Sim)
drives it.

## Migration map (drive_node.cpp -> here)
| 2026 `drive_node.cpp` | Lands in |
|-----------------------|----------|
| `SparkFlex` construction + `configure_motors()` | `on_init()` / `on_configure()` |
| `Heartbeat()` timer | inside `write()` (called every control cycle) |
| `SetDutyCycle()` from cmd | `write()` — take velocity commands from the controller |
| Hall-sensor feedback | `read()` — populate wheel position/velocity state |
| Watchdog / stop | `on_deactivate()` + controller `cmd_vel_timeout` |

The differential-drive *math* does NOT move here — that now lives in
`diff_drive_controller`. This layer only converts per-wheel velocity commands to
motor duty/velocity and reads encoder feedback back.

## Status
Skeleton only: the lifecycle methods are stubbed with the interface contract and
TODOs. Implement in Phase 3 (hardware parity) once sim autonomy is proven.

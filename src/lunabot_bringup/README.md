# lunabot_bringup

The one place a user launches. Composes the other packages into a complete stack
and exposes the single knob that matters:

```bash
# full autonomy stack in Isaac Sim, differential drive
ros2 launch lunabot_bringup sim.launch.py drive_type:=diff

# same stack, tank/skid-steer, on the real robot
ros2 launch lunabot_bringup robot.launch.py drive_type:=skid
```

`config/robot.yaml` holds shared defaults (drive_type, ROS_DOMAIN_ID, frame names)
so sim and hardware stay consistent.

- `sim.launch.py` -> `lunabot_simulation/isaac_sim.launch.py` + navigation (sim time).
- `robot.launch.py` -> description(sim_mode=none) + real controller_manager +
  `lunabot_hardware` + perception + navigation.

The difference between sim and hardware is ONLY the hardware plugin (`sim_mode`)
and `use_sim_time`. Everything else is shared — that is the parity goal.

## Status
Launch files are structural stubs wiring the includes; flesh out as the
underlying packages come online (Phases 1–3).

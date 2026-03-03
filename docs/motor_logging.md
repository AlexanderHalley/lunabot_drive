# Motor Telemetry CSV Logging

**Status: Not yet implemented.** This describes planned additions to `src/drive_node.cpp`.

---

## Plan

Add a `MotorLogger` header-only class that writes per-motor telemetry to a timestamped CSV on each run. Chosen over rosbag/JSON because it opens directly in Excel on-site.

### CSV Schema

```
timestamp_s, motor_id, velocity_rpm, position_rot, current_a, voltage_v, temperature_c, duty_cycle, faults, sticky_faults
```

- Motor IDs: 1=FL, 2=FR, 3=RL, 4=RR
- Log rate: 10 Hz
- Log dir: `/home/lunabot/logs/motor_log_YYYYMMDD_HHMMSS.csv`

### Files to Add

| File | Change |
|---|---|
| `src/motor_logger.hpp` | New — header-only logger class |
| `src/drive_node.cpp` | Add include, logger member, `log_callback()` timer at 10 Hz |

### Copying Logs Off the Pi

```bash
scp -r lunabot@<PI_IP>:/home/lunabot/logs ./beach_logs/
```

### Quick Analysis (Python)

```python
import pandas as pd
df = pd.read_csv("motor_log.csv")

# Current per motor
df.groupby("motor_id")["current_a"].plot()

# Fault events
print(df[df["faults"] != 0][["timestamp_s", "motor_id", "faults"]])

# Temp peak
print(df.groupby("motor_id")["temperature_c"].max())
```

### Thresholds to Watch

| Signal | Warning |
|---|---|
| `temperature_c` | > 60°C |
| `current_a` | > 35A sustained |
| `voltage_v` | < 11.0V |
| `faults` | any non-zero |

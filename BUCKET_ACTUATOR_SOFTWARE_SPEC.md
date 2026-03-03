# Bucket Actuator — Hardware & Interface Reference

Implementation: `nodes/actuator_driver_node.py`

---

## Hardware

### Actuators — Firgelli Super Duty F-SD-H-220-12v

| Parameter | Lift | Tilt |
|---|---|---|
| Stroke | 8″ (203.2 mm) | 10″ (254.0 mm) |
| Speed (no load) | 15 mm/s | 15 mm/s |
| Max current | 5.5 A @ 12V | 5.5 A @ 12V |
| Drive | Worm gear — non-backdrivable, holds position without power |
| Limit switches | Built-in at both ends, non-adjustable |
| Feedback | Hall effect, 17.4 pulses/mm/wire (0.02 mm accuracy) |
| Motor wires | Brown (+12V extend), Blue (GND) |
| Sensor wires | Red (5V VCC), Black (GND), Yellow (Hall A), White (Hall B) |

### Driver — BTS7960 43A H-Bridge

| Pin | Function |
|---|---|
| RPWM | Extend direction PWM |
| LPWM | Retract direction PWM |
| R_EN + L_EN | Enable (active HIGH) |
| R_IS, L_IS | Current sense (future use) |

### GPIO Pin Assignments

See `PI5_PIN_ASSIGNMENTS.md` for the full Pi 5 pin map.

| Signal | Lift BCM | Tilt BCM |
|---|---|---|
| RPWM (extend) | 12 — PWM0 CHAN0 | 13 — PWM0 CHAN1 |
| LPWM (retract) | 18 — PWM0 CHAN2 | 19 — PWM0 CHAN3 |
| EN | 5 | 6 |
| Hall A | 22 | 24 |
| Hall B | 23 | 25 |

> Hall sensor outputs are 5V. Hardware team adds 1kΩ/2kΩ voltage dividers to 3.3V before Pi GPIO.

---

## ROS2 Interface

Each instance runs in a namespace (`/bucket/lift` or `/bucket/tilt`). All topics are relative to that namespace.

### Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `~/command` | `std_msgs/Float64` | on demand | Target position in mm from home. Range: [0, stroke_mm] |
| `~/position` | `std_msgs/Float64` | 20 Hz | Current position in mm |
| `~/status` | `diagnostic_msgs/DiagnosticStatus` | 5 Hz | State, position, moving flag, cooldown, etc. |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz | URDF joint position (merged with drive_node) |

### Services

| Service | Type | Description |
|---|---|---|
| `~/home` | `std_srvs/Trigger` | Retract to limit switch, zero position (30s timeout) |
| `~/stop` | `std_srvs/Trigger` | Immediately stop motion |

### Key Parameters

| Parameter | Lift default | Tilt default |
|---|---|---|
| `stroke_mm` | 203.2 | 254.0 |
| `pulses_per_mm` | 17.4 | 17.4 |
| `deadband_mm` | 1.0 | 1.0 |
| `approach_distance_mm` | 20.0 | 20.0 |
| `min_speed_pct` / `max_speed_pct` | 30 / 100 | 30 / 100 |
| `watchdog_timeout_s` | 2.0 | 2.0 |
| `home_on_startup` | true | true |
| `max_continuous_run_s` | 15.0 | 15.0 |
| `cooldown_time_s` | 45.0 | 45.0 |
| `joint_name` | `bucket_lift_joint` | `bucket_tilt_joint` |
| `invert_direction` | false | false |

Full defaults in `config/bucket_actuators.yaml`.

---

## Competition Notes

- **Duty cycle:** 15s on / 45s off at full load — enforced in software.
- **E-STOP:** 12V actuator supply is downstream of E-STOP. Software watchdog (2s) is a secondary layer.
- **Homing:** Limit switch detection is passive — motor stops when Hall pulses cease for 500ms.
- **Force:** 220 lb actuators exceed robot weight (~176 lbs). Limit `max_speed_pct` during autonomous excavation to reduce penetration force.
- **Autonomy scoring:** Excavation Automation (75 pts) + Dump Automation (50 pts) depend on this subsystem.

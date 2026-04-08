# Raspberry Pi 5 — Lunabot Pin Assignments

All BCM (Broadcom) GPIO numbers. Physical pin numbers shown in the header diagram below.

---

## GPIO — Bucket Actuators

Controlled by `nodes/actuator_driver_node.py` via `lgpio`.

### Lift Actuator (8″ stroke, BCM→Physical)

| Signal | BCM | Physical Pin | Direction | Notes |
|---|---|---|---|---|
| RPWM (extend) | 12 | 32 | Pi → BTS7960 | **Hardware PWM0 CHAN0** |
| LPWM (retract) | 18 | 12 | Pi → BTS7960 | **Hardware PWM0 CHAN2** — independent from GPIO 12 on Pi 5 |
| R_EN + L_EN | 5 | 29 | Pi → BTS7960 | Active HIGH — shared enable for both channels |
| Hall Channel A | 22 | 15 | Sensor → Pi | 3.3V via voltage divider (1kΩ/2kΩ from 5V sensor) |
| Hall Channel B | 23 | 16 | Sensor → Pi | 3.3V via voltage divider (1kΩ/2kΩ from 5V sensor) |

### Tilt Actuator (10″ stroke, BCM→Physical)

| Signal | BCM | Physical Pin | Direction | Notes |
|---|---|---|---|---|
| RPWM (extend) | 13 | 33 | Pi → BTS7960 | **Hardware PWM0 CHAN1** |
| LPWM (retract) | 19 | 35 | Pi → BTS7960 | **Hardware PWM0 CHAN3** — independent from GPIO 13 on Pi 5 |
| R_EN + L_EN | 6 | 31 | Pi → BTS7960 | Active HIGH — shared enable for both channels |
| Hall Channel A | 24 | 18 | Sensor → Pi | 3.3V via voltage divider (1kΩ/2kΩ from 5V sensor) |
| Hall Channel B | 25 | 22 | Sensor → Pi | 3.3V via voltage divider (1kΩ/2kΩ from 5V sensor) |

> **Hall sensor wiring:** Sensor VCC = 5V (from BTS7960 logic supply, NOT Pi 5V header).
> Sensor GND tied to common ground. Yellow = Ch A, White = Ch B.
> Voltage dividers required before Pi GPIO to protect from 5V signal.

---

## USB Interfaces

| Port | Device | Protocol | Speed | Used By |
|---|---|---|---|---|
| USB-A | CAN-USB adapter | slcand → `/dev/ttyACM0` → `can0` | 1 Mbps CAN | SparkFlex motor controllers |
| USB-A (3.0) | OAK-D S2 camera | USB 3.0 SuperSpeed | 5 Gbps | depthai_ros_driver (`/oak/*` topics) |

### CAN Bus — SparkFlex Motor Controllers

Interface brought up via `scripts/initialise_can`:
```bash
sudo slcand -o -s8 /dev/ttyACM0 can0 && sudo ip link set can0 up
```

| Motor | CAN ID | Position |
|---|---|---|
| Right Front | 1 | Drive wheel |
| Left Front | 2 | Drive wheel |
| Left Rear | 3 | Drive wheel |
| Right Rear | 4 | Drive wheel |

---

## Power Rail Summary

| Rail | Source | Powers |
|---|---|---|
| 12V | Battery → E-STOP → distribution bus | BTS7960 motor supply (B+/B−) |
| 5V (actuators) | 5V regulator off 12V bus | BTS7960 logic VCC, Hall sensor VCC |
| 5V (Pi) | Independent regulator | Raspberry Pi 5 only |
| 3.3V | Pi internal (via GPIO header) | **NOT used for actuator logic** |

> **IMPORTANT:** Do NOT draw 5V from the Pi GPIO header (pins 2/4) for the actuator drivers or Hall sensors. Use the dedicated 5V regulator off the 12V bus. Pi and actuator grounds must share a common reference.

---

## 40-Pin GPIO Header Map

```
       3V3  [ 1][ 2]  5V
     GPIO2  [ 3][ 4]  5V
     GPIO3  [ 5][ 6]  GND
     GPIO4  [ 7][ 8]  GPIO14
       GND  [ 9][10]  GPIO15
    GPIO17  [11][12]  GPIO18  ← Lift LPWM (retract, SW-PWM)
    GPIO27  [13][14]  GND
    GPIO22  [15][16]  GPIO23  ← Lift Hall A / Lift Hall B
       3V3  [17][18]  GPIO24  ← Tilt Hall A
    GPIO10  [19][20]  GND
     GPIO9  [21][22]  GPIO25  ← Tilt Hall B
    GPIO11  [23][24]  GPIO8
       GND  [25][26]  GPIO7
     GPIO0  [27][28]  GPIO1
     GPIO5  [29][30]  GND     ← Lift EN
     GPIO6  [31][32]  GPIO12  ← Tilt EN / Lift RPWM (HW-PWM0)
    GPIO13  [33][34]  GND     ← Tilt RPWM (HW-PWM1)
    GPIO19  [35][36]  GPIO16  ← Tilt LPWM (retract, SW-PWM)
    GPIO26  [37][38]  GPIO20
       GND  [39][40]  GPIO21
```

### Pins In Use (summary)

| Physical | BCM | Function | HW PWM Channel |
|---|---|---|---|
| 12 | 18 | Lift LPWM (retract) | PWM0 CHAN2 |
| 15 | 22 | Lift Hall Ch A | — |
| 16 | 23 | Lift Hall Ch B | — |
| 18 | 24 | Tilt Hall Ch A | — |
| 22 | 25 | Tilt Hall Ch B | — |
| 29 | 5 | Lift EN | — |
| 31 | 6 | Tilt EN | — |
| 32 | 12 | Lift RPWM (extend) | PWM0 CHAN0 |
| 33 | 13 | Tilt RPWM (extend) | PWM0 CHAN1 |
| 35 | 19 | Tilt LPWM (retract) | PWM0 CHAN3 |

> **Pi 5 PWM note:** All four PWM pins belong to a single `PWM0` block on the RP1 I/O chip,
> but each has a fully **independent channel** (CHAN0–CHAN3). This is a Pi 5 improvement over
> Pi 4 and earlier, where GPIO 12/18 shared one channel and GPIO 13/19 shared another —
> meaning independent hardware PWM on RPWM and LPWM was not possible. On Pi 5, all four
> actuator PWM signals can run independent hardware PWM simultaneously.
> "Slices" is RP2040/Pico terminology and does not apply to the Pi 5.

### Free GPIO (40-pin header)

Unallocated BCM pins available for future use: 0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15, 16, 17, 20, 21, 26, 27.

---

## Config Reference

- Actuator GPIO params: `config/bucket_actuators.yaml`
- CAN IDs: `launch/hardware_bringup.launch.py` (and `pi_drive.launch.py`, `motor_test_rviz.launch.py`)
- Actuator driver: `nodes/actuator_driver_node.py`
- CAN init script: `scripts/initialise_can`

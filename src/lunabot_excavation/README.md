# lunabot_excavation

The excavator behind one interface: dig and dump are **actions** the mission calls, whatever moves
the drum and hopper. `scripts/excavator.py`, started by `robot.launch.py` (`excavation:=true`, the
default).

| Interface | Type | What |
|---|---|---|
| `/excavation/excavate` | `lunabot_msgs/action/Excavate` | return the hopper flat, lower the drum, dig until `target_fill` (where measured) or `max_seconds` of digging, raise it |
| `/excavation/deposit` | `lunabot_msgs/action/Deposit` | raise the drum, tip the hopper, pour until empty (where measured) or `max_seconds` tipped, bring it back |
| `/excavation/stow` | `std_srvs/Trigger` | stop the job, stow everything (an operator after an abort) |
| `/excavation/status` | `lunabot_msgs/ExcavatorStatus` | 10 Hz: STOWED / HOLDING / EXCAVATING / DEPOSITING / STOWING / FAULT, phase, strokes, fill |
| `/excavation/joint_states`, `/excavation/fill` | `sensor_msgs/JointState`, `std_msgs/Float32` | the same in plain types, for Isaac's stand-in |
| `/e_stop` (in) | `std_msgs/Bool` | engaged: hold, end the goal as E_STOP, refuse goals until released |

The rules the mission relies on (tested in `test/`):

- **A goal that ends normally ends stowed** (FULL / EMPTY / TIME_LIMIT): the rover may drive.
- **A cancelled goal holds where it is**, it does not stow. `mission.py` cancels on pause and, on
  resume, sends the same goal with the working time that was left; a goal with none left just
  raises the drum (or flattens the hopper) without digging again.
- **A stroke that does not arrive within `move_timeout` is a FAULT**: it holds, the goal aborts,
  the mission retries once. A new goal or a stow clears it.
- **One job at a time.** A second goal is refused; the stow service preempts.
- Strokes are **normalised**: arm 0 stowed .. 1 at digging depth, hopper 0 flat .. 1 tipped. The
  2027 four-bar and hopper geometry is not final; a driver maps the strokes onto its actuators.

## Backends

`backend: model` is the only one: a kinematic stand-in that drives nothing. Strokes move at a
fixed rate, the hopper fills at `fill_rate` while the drum is down and turning and empties once
tipped past `pour_from`. Its fill is an **estimate**, so it reports `fill_measured: false` and goals
end on their time limits — as they will on a rover with no load sensing. Every number in
`config/excavator.yaml` is a PLACEHOLDER taken from the visual stand-in, not from the design.

It runs in Isaac and, until there are drivers, on the rover: the mission then stops for its dig
and dump times exactly as the old timed placeholder did, but through the interface a driver will
serve.

**Adding a driver** (the drum, lift and hopper motors): a class with

```python
name = 'spark'
def read(self) -> Reading: ...           # strokes 0..1, drum rad/s, fill (NaN if unknown), fault text
def command(self, cmd: Command) -> None: ...  # stroke targets and drum speed
def update(self, dt: float) -> None: ...      # called every tick before read()
```

registered in `make_backend()`. Nothing else changes: the sequences, limits, cancel, e-stop and
fault handling are the backend-independent `Job` / `Excavator` classes. What a driver needs from
the mechanical team: actuator types and CAN IDs, stroke ends (limit switches or encoder counts),
drum speed and current limits, and whether anything senses the load (a current threshold on the
drum, a load cell) — with that, `fill_measured` turns on and digs end when full.

## Not done

- No driver: nothing here moves hardware.
- The excavator is not in the URDF, so `/excavation/joint_states` is not on `/joint_states` and RViz
  does not draw it.
- Nothing stops the wheels while the drum is down; the mission only drives after a goal has ended
  stowed. A teleop driver can still drive with it lowered.
- Mission control (Foxglove) has no excavator panel or stow button yet.

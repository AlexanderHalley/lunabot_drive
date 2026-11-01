# Monitoring and visualisation

What to run to see what the rover is doing, and what each tool can and cannot show you.

Everything here is optional and none of it is on the control path. The one exception is
`/diagnostics`, which is on by default — see [Why diagnostics defaults to on](#why-diagnostics-defaults-to-on).

---

## The short version

| I want to | Run |
|---|---|
| See the robot, the map, the cloud and the detections | `robot.launch.py rviz:=true` |
| See what Nav2 is thinking | `robot.launch.py nav:=true rviz:=true rviz_config:=nav` |
| Know whether the stack is healthy, continuously | nothing — `/diagnostics` is on by default |
| Check the stack once, with an exit code | `ros2 run lunabot_bringup check_stack.py --profile sim` |
| A health tree with lights | `ros2 run rqt_robot_monitor rqt_robot_monitor` |
| Plot a number over time | `ros2 run plotjuggler plotjuggler` |
| All of the above from a laptop, in a browser-grade UI | `robot.launch.py foxglove:=true` |
| Run the mission from a laptop: start, pause, abort, e-stop | [Mission control](#mission-control) |
| Record a run to replay later | `robot.launch.py record:=true` |

---

## Mission control

One Foxglove layout to watch and command the autonomous run:
`lunabot_bringup/config/foxglove/mission_control.json`. Foxglove → Layouts → Import from file.

```bash
# the robot (or the sim: sim.launch.py takes the same four arguments)
ros2 launch lunabot_bringup robot.launch.py hw:=real nav:=true slam:=rtabmap \
  foxglove:=true foxglove_profile:=field wait_for_start:=true record:=true
ros2 run lunabot_navigation mission.py --ros-args -p autostart:=false \
  --params-file $(ros2 pkg prefix lunabot_navigation)/share/lunabot_navigation/config/mission_ucf.yaml
```

Then connect Foxglove to `ws://<robot>:8765` and load the layout. Nothing moves until **Start**:
the mission waits in `READY`, and `wait_for_start:=true` holds the start survey until the mission
latches `/mission/started`.

| Panel | Shows / does |
|---|---|
| Arena (3D, `arena` frame) | global costmap, the current plan, the rover model, boulder markers |
| Mission mode | `/mission/mode`: READY, RUNNING, PAUSED, E-STOP, ABORTED, FAILED, COMPLETE |
| Software e-stop | `/e_stop` as a light |
| Mission state | `/mission/state`, the phase line (`DRIVING TO EXCAVATION ZONE (1/2)`, ...) |
| Start / Pause / Resume / Abort | `/mission/<name>`, std_srvs/Trigger; the reply says why a refused one was refused |
| E-STOP / Release e-stop | `/e_stop/set` (std_srvs/SetBool), served by `operator_e_stop`, which publishes `/e_stop` |
| Onboard camera | `/operator/camera/compressed`: 424 px JPEG at 2 fps, ~15 kB/s (Isaac) |
| Speed | `/odom` measured vs `/cmd_vel` commanded |
| Stack health | `/diagnostics` |

**What the buttons do** (`mission.py`, class `Control`; unit-tested):

- **Pause** cancels the leg's Nav2 goal and holds. **Resume** sends the leg again from wherever
  the rover stopped; a dig or dump carries on with the time it had left. A pause never counts
  as a failed leg.
- **E-STOP** is twist_mux's `e_stop` lock (priority 255, above teleop): while `/e_stop` last said
  `true`, no velocity source reaches the controller, and `cmd_vel_timeout` stops the wheels in
  0.5 s (Isaac, through the bridge: wheels stopped 1.0 s after the button).
  `scripts/operator_e_stop.py` republishes `true` every second while engaged, so a twist_mux or
  mission that restarts mid-stop locks again; it never republishes `false`. The mission sees the same message and pauses. **Releasing it does not drive off:** the
  mission stays `PAUSED` until Resume, and Resume is refused while the e-stop is engaged.
- **Abort** cancels the goal and ends the mission (`ABORTED: <phase>`). Final: restart
  `mission.py` to run again. Teleop still works after an abort.
- The lock is **latching, not a heartbeat**: one `true` holds until a `false`. If the laptop
  drops off the WiFi the rover carries on autonomously, which is what the run wants; it is not a
  dead-man. It is also **not the competition e-stop**, which is the physical button that cuts
  power. It is the operator's first, gentler stop.

**`foxglove_profile:=field`** (`config/foxglove/field.yaml`) exposes only the topics the layout
draws and accepts only its buttons back: five services, and **no publishing from the laptop at
all**. That last part is forced: foxglove_bridge 3.x reads `client_topic_whitelist` and never
applies it (3.6.3, `clientAdvertise`) — measured, a client allowed only `/e_stop` got
`/cmd_vel_joy` messages through, and teleop outranks Nav2. Dropping the `clientPublish`
capability IS enforced, so the e-stop is a service. Also no parameters capability (a misclick in
a settings panel cannot retune Nav2 mid-run), no raw images, no point clouds, no local costmap.
`bench` (the default) is everything, both ways, publishing included.
`test_mission_control.py` checks that every panel's topic gets through `field` and is recorded,
and that every button is a service the mission serves.

**Lichtblick** (the open-source Foxglove fork) loads the same layout and connects too: bridge 3.x
speaks only the `foxglove.sdk.v1` WebSocket subprotocol, which both offer. Lichtblick's Call
Service panel always shows its request and response boxes (Foxglove hides them with
`editingMode: false`, which the layout sets), so the button panels are sized for that: the layout
fits a 1366×768 laptop screen with every button, the mode, the e-stop state and the camera visible.
It shows a "schema 'visualization_msgs/Marker' has changed" alert; harmless (Lichtblick's
built-in definition against Jazzy's; the markers draw).

**The operator camera** (`scripts/operator_camera.py`, started with `foxglove:=true`,
`operator_camera:=false` to skip it) subscribes to the camera only while something subscribes
to it, so it costs nothing until a panel or a recording watches it.

Verified in Isaac through the bridge, as the laptop would: `~/lunabotics/tools/mission_control/
live_check.sh` (desktop only) runs the stack as above and checks each button against `/odom`.
And in a real UI (2026-11-01): Lichtblick, headless, against the rover image and Isaac, the layout
imported from this file, a run driven entirely by its buttons (start, e-stop, release, resume,
pause, abort) and screenshotted at 1600×1000 and 1366×768 (`tools/mission_control/ui/`).

### Recording a run

`record:=true` (`launch/record.launch.py`) writes `config/record_topics.yaml` to
`~/lunabotics/data/bags/run_<date>_<time>/` (`bag_dir:=` elsewhere): mission state, costmaps,
plan, pose, every command source, the operator camera, diagnostics, `/rosout`. No raw images or
clouds — they are gigabytes, and recording them would make the camera produce them. Jazzy writes
MCAP, which Foxglove opens directly: load the mission control layout on the bag and the replay
looks like the live run.

---

## RViz

Three configs, in `lunabot_description/rviz/`, selected with `rviz_config:=`:

| Config | Fixed frame | For |
|---|---|---|
| `description` | `base_link` | The model and TF only. Checking the URDF, with nothing else running |
| `slam` | `map` | Map, cloud, detections, odometry vs ground truth. The default |
| `nav` | `map` | Costmaps, global and local plans, the padded footprint |

Two things about them are load-bearing and easy to undo by accident.

**QoS is pinned per display.** RViz's default reliability does not match every publisher in this
stack: `/oak_d/points` and the image topics are `rclcpp::SensorDataQoS` (BEST_EFFORT), and `/map`,
`/robot_description` and both costmaps are TRANSIENT_LOCAL. A mismatch is not an error — the
display simply stays empty, which reads as "the node is not running". Every `Topic` block in those
files therefore states its policies explicitly.

**RViz rewrites the file when you hit Save**, stripping every comment in it, including the ones
explaining the QoS. Save to a scratch file and port the diff by hand.

`use_sim_time` is why `rviz.launch.py` exists as a separate file at all. RViz is the node people
forget to pass it to, and the symptom is a TF display that freezes or lags rather than an error.

### What RViz cannot show

`/perception/boulders` is `vision_msgs/Detection3DArray`, and RViz has no renderer for it. That is
why `boulder_detector` also publishes `/perception/debug/markers` — a `MarkerArray` saying the same
thing, published lazily so it costs nothing until something subscribes. The debug topics are
explicitly **not** part of the contract; do not build on them.

---

## /diagnostics

`robot_health` (in `lunabot_bringup/scripts/robot_health.py`) publishes
`diagnostic_msgs/DiagnosticArray` on `/diagnostics` at 1 Hz, covering:

- **Topic rates** for the contract's core topics, plus `/clock` under `hw:=sim` and `/drive/status`
  under `hw:=real`. Each reports its rate, its minimum, its age and its publisher count, and
  distinguishes "stale, nothing for 12 s" from "running at half rate" — different problems, and a
  bare rate makes you work out which one you have.
- **Controllers**, from `controller_manager`'s `list_controllers`. `joint_state_broadcaster` and
  `diff_drive_controller` not being `active` is an ERROR; anything else loaded is reported.
- **The drivetrain**, from `/drive/status`, under `hw:=real`.
- **The EKF's own diagnostics**, when `odom_source:=ekf` — `print_diagnostics` is set in `ekf.yaml`.

`diagnostic_aggregator` also runs by default and republishes the tree on `/diagnostics_agg`,
grouped by `config/diagnostics.yaml`. That topic exists for exactly one consumer, `rqt_robot_monitor`;
Foxglove and PlotJuggler read the raw `/diagnostics`.

### What it deliberately does not watch

Nothing here subscribes to an `Image` or a `PointCloud2`, and nothing should. Subscribing to
`/oak_d/points` to check the camera is alive doubles the bandwidth of the heaviest topic on the Pi
and defeats the driver's lazy publisher — `i_enable_lazy_publisher` means the camera only produces
frames when something is subscribed, so watching it is what makes it expensive.

Heavy topics are watched by proxy instead: `camera_info` stands in for its image stream (same
publisher, same rate, a few dozen bytes), and `/perception/boulders` stands in for the point cloud,
being downstream of it.

### What it deliberately does not warn about

`motor_feedback_active: false` and `watchdog_triggered: true` are both reported as values and
neither escalates the status. Open loop is the known, documented state of this drivetrain, and the
watchdog flag is true whenever the rover is parked. Either one as a WARN would make the drivetrain
permanently amber, and a light that is always on is a light nobody reads — taking the real warnings
with it.

### Why diagnostics defaults to on

A health topic that has to be remembered is a health topic that is off during the run where it
would have mattered. It is two small nodes that subscribe to nothing expensive. `diagnostics:=false`
turns both off.

### Staleness is measured on the wall clock

Every arrival time in `robot_health.py` is `time.monotonic()`, never the ROS clock. Under
`use_sim_time`, if Isaac stops publishing `/clock` the ROS clock stops advancing — and a staleness
check written against it would subtract two frozen numbers, get a constant, and report a healthy
robot at the exact moment the entire stack has stopped.

---

## Foxglove Studio

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=sim use_sim_time:=true foxglove:=true
```

Then connect Foxglove to `ws://<robot>:8765`. Off by default because it opens a listening socket.

The bridge is Apache-2.0. Foxglove Studio itself has a permanently free tier, and students,
researchers and educators get the paid tier free on a `.edu` or `.ac` address — which is what makes
it worth wiring in rather than leaving as a suggestion.

**Why `foxglove_bridge` and not `rosbridge`.** `foxglove_bridge` reads each publisher's QoS off the
graph and matches it per topic. `rosbridge` subscribes RELIABLE by default and so silently never
matches this stack's BEST_EFFORT sensor publishers — the panel stays empty with no error anywhere.
It is the same trap the RViz configs pin their QoS to avoid.

| Panel | Works because |
|---|---|
| 3D | `/robot_description` is TRANSIENT_LOCAL, so the URDF loads. TF, clouds, images, markers all render |
| Plot | The bridge carries each message's schema from its type support, so `lunabot_msgs/DriveStatus` plots with nothing installed on the laptop |
| Diagnostics | Reads `/diagnostics` directly |
| Image | Works, and will saturate the link — see below |

Foxglove has no renderer for `vision_msgs/Detection3DArray` either. Use
`/perception/debug/markers`, same as RViz.

**Bandwidth.** `foxglove_profile:=bench` (the default) exposes every topic, which is right on a
bench and wrong over competition WiFi: the camera publishes 1080p RGB at 15 fps with no compressed
transport configured (see [`OAK_D_S2_INTEGRATION.md`](OAK_D_S2_INTEGRATION.md)). Use
`foxglove_profile:=field` on a link you do not control — see [Mission control](#mission-control).

Offline, `ros2 bag record` on Jazzy writes MCAP by default, which Foxglove opens directly.

---

## PlotJuggler

`ros2 run plotjuggler plotjuggler`, then subscribe to the ROS 2 topics you want.

Plottable today: `/odom`, `/joint_states`, `/cmd_vel`, `/oak_d/imu/data`, `/diagnostics`, and
`/drive/status` on real hardware.

`/drive/status` is the reason `lunabot_msgs` exists. `diagnostic_msgs/DiagnosticStatus` is the
usual suggestion for telemetry and it is stringly-typed key-value pairs, which cannot be plotted and
cannot be type-checked — `DriveStatus` carries the same information as numbers so that "graph bus
voltage during a dig" is a drag rather than a parsing exercise.

What it carries **today** is `applied_duty_cycle` per motor plus the drivetrain-level fields. Bus
voltage, current, temperature and fault bits are NaN, because `SparkFlexMotor` has no getter for any
of them yet — NaN and not zero, precisely so the plot shows a gap rather than a convincing flat line
at 0 V. See [`HARDWARE_CAN.md`](HARDWARE_CAN.md).

---

## rqt

`rqt_robot_monitor` reads `/diagnostics_agg` and needs the aggregator, which runs by default.
`rqt_controller_manager`, `rqt_graph`, `rqt_tf_tree` and `rqt_console` all work with no setup —
they read the standard interfaces, which this stack does not deviate from.

---

## What is not here

- **No Prometheus, Grafana or InfluxDB exporter.** Nothing in this workspace speaks to a
  time-series database. If that is ever wanted, `/diagnostics` is the input to write it against.
- **No `rosbridge` or `web_video_server`.** No browser dashboard, by choice; Foxglove covers the
  same ground with QoS that actually matches.
- **No compressed image transport configured.** `image_transport_plugins` will republish
  `/oak_d/rgb/image_raw/compressed` if installed, but no launch file sets it up and it is not a
  declared dependency. Over WiFi, use the operator camera (2 fps, small) in the mission control
  layout; full-rate images are a wired-LAN assumption, and the 10 MB CycloneDDS receive buffer in
  `OAK_D_S2_INTEGRATION.md` is part of it.
- **No teleop in the layout.** Foxglove's Teleop panel publishes unstamped `Twist`; twist_mux
  here takes `TwistStamped`. Teleop is the joystick.

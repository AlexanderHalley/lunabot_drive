# Sim acceptance: the day the GPU machine arrives

The order of operations for the first Isaac Sim bring-up, and what "working" means at each step.

Read [`SIM_ISAAC.md`](SIM_ISAAC.md) first for how the sim is built and why. This file is the
checklist.

The whole point of the order below is that **each step fails for one reason**. Starting Isaac and
the ROS stack together on the first attempt gives you a graph that does nothing, a terminal full of
warnings, and nothing to bisect.

Budget half a day. Most of it is step 1, and step 1 is the reason the rest goes quickly.

---

## Before you start

Everything in steps 0–2 is already testable and already passing in CI, on machines with no GPU:

| What | Covered by | Runs where |
|---|---|---|
| Scene layout, seeded determinism | `lunabot_sim/test/test_scene_layout.py` | CI |
| The probe's list of Isaac API names | `lunabot_sim/test/test_probe.py` | CI |
| The whole ROS stack on mock hardware | `lunabot_bringup/test/test_mock_bringup.py` | CI |
| The whole ROS stack on `hw:=sim` | `lunabot_bringup/test/test_sim_bringup.py` | CI |
| The contract checker itself | run inside the sim bringup test | CI |

So if something fails below, **the ROS side is not the first suspect**. What has never run is
`lunabot_sim`'s Isaac-facing half: `compat.py`, `graphs/*.py`, `robot/importer.py`,
`robot/articulation.py`, and the scene builders' Isaac imports.

---

## Step 0 — the machine

```bash
nvidia-smi                       # a GPU, a driver, and a version worth writing down
lsb_release -d                   # 24.04 for a Jazzy bridge
echo "$ISAAC_SIM_PATH"           # or let the scripts search; see scripts/isaac_env.sh
```

**Confirm before committing to a version:** that this Isaac Sim build ships a ROS 2 **Jazzy**
bridge for Ubuntu 24.04. The bridge links prebuilt internal ROS libraries per distro, and landing
on a 22.04-only build means reopening the Jazzy decision rather than debugging a graph.

---

## Step 1 — probe the API, before anything else

```bash
src/lunabot_sim/scripts/probe_isaac_api.sh --json /tmp/isaac_probe.json
```

Headless, builds nothing, publishes nothing, takes about a minute. It reports every
version-sensitive name in `compat.py` against what this install actually has: extension ids, the
twelve OmniGraph node types the graphs create, the IMU sensor class, the URDF importer entry point.

**This is the step that saves the day.** Every one of those names was reconstructed from
documentation for a version nobody here had run. Found one crash at a time they cost an hour each,
because the error names the symbol rather than the rename.

Expect misses. A miss is not a bug in `lunabot_sim` — it is `lunabot_sim`'s guess about your Isaac
version. Fix each one in **`lunabot_sim/compat.py`, and nowhere else**; that file exists for
exactly this, and the report names the tuple to edit. Re-run until it exits zero.

When it does, delete the `VERIFY` blocks it just answered and record the version you probed. The
next person should not repeat this.

---

## Step 2 — the stack with no Isaac at all

```bash
colcon build && source install/setup.bash
colcon test --packages-select lunabot_bringup lunabot_sim && colcon test-result --verbose
```

`test_sim_bringup` brings the whole stack up on `hw:=sim` against `test/isaac_double.py` — a
stand-in publishing `/clock`, `/isaac/joint_states` and ground-truth odom, and nothing else.

If this fails on the sim machine but passes in CI, the fault is the machine's ROS install, not
Isaac: nothing here touches a GPU.

---

## Step 3 — Isaac alone

```bash
src/lunabot_sim/scripts/run_isaac_sim.sh --seed 7
```

No ROS bringup yet. Watch for, in this order:

1. `Isaac Sim version: ...` and `ROS 2 bridge extension: ...` — `compat.py` resolved.
2. `scene "default" seed 7: 12 boulders` — the scene built.
3. `imported lunabot_sim.urdf to /World/Lunabot` — the robot imported.
4. `configured 4 wheel joints for velocity drive` — **the line to care about**. Fewer than four
   raises; the message names the joint.
5. `running. /clock is live` — the graphs evaluate.

Then, from a second terminal with the ROS environment sourced:

```bash
ros2 topic hz /clock                        # should be steady
ros2 topic list | grep -E 'isaac|oak_d|clock|ground_truth'
ros2 topic echo --once /sim/ground_truth/odom
```

**If `/clock` is not there, stop.** Nothing downstream will work and everything downstream will
fail by hanging silently rather than erroring. It is the clock graph in `graphs/clock.py`.

Known rough edges — no camera prim, depth encoding, real-time factor — are catalogued in
[`SIM_ISAAC.md`](SIM_ISAAC.md#known-rough-edges). Read that section before debugging, not after.

---

## Step 4 — the stack against Isaac

Isaac stays running. In the second terminal:

```bash
ros2 launch lunabot_bringup sim.launch.py
```

It blocks on `/clock` before starting anything. If that wait times out, Isaac is not running or is
on a different `ROS_DOMAIN_ID` — the graphs set it explicitly in `graphs/context.py`.

Then, in a third terminal:

```bash
ros2 run lunabot_bringup check_stack.py --profile sim
```

Every check must pass. It is the same tool `test_sim_bringup` runs against the double, so a failure
here is a difference between Isaac and the double — which is exactly what this run is for.

The check worth understanding rather than just running is **`only the expected nodes advertise
/tf`**. Isaac must not publish TF. If `ROS2PublishTransformTree` ever gets added to a graph, there
will be two publishers on the same edges and a tree that works only in sim, so every TF bug stays
invisible until the rover is on regolith.

---

## Step 5 — drive it

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.3}}}'
```

| Observation | Meaning |
|---|---|
| `/odom` advances in x | the whole chain works |
| nothing moves, and `/isaac/joint_commands` is silent | `trigger_joint_command_threshold` — see below |
| the rover snaps to a wheel angle and stops | the anticipated bug, which should not happen — see below |
| `/odom` jumps backwards periodically | `sum_wrapped_joint_states`; see below |
| nothing moves, and `/isaac/joint_commands` is publishing | Isaac's side: the articulation, the drives, or the joint names |

**Nothing moves and no command is published** is the failure to check first, with
`ros2 topic hz /isaac/joint_commands`. `TopicBasedSystem::write()` skips publishing when the
position command and position state are within `trigger_joint_command_threshold`, and a
velocity-only drivetrain leaves both at 0.0 forever, so at the default threshold it never publishes
anything at all. `lunabot.ros2_control.xacro` sets it negative for exactly this reason; the
comment there is the long version.

**Snapping to an angle** is the failure `graphs/joints.py` and `robot/articulation.py` are both
written against: `ROS2SubscribeJointState` chooses position or velocity targets by which arrays are
non-empty. It should not happen — `write()` pushes an array only for the command interfaces a joint
declares, and these declare velocity alone, which `test_sim_bringup` asserts. If it happens anyway,
something added a position command interface, and the two defences that keep it merely slow rather
than broken are zero drive stiffness and `positionCommand` left unwired.

**Backwards jumps in `/odom`** mean wrapped joint positions are not being summed back into a
monotonic position. Confirm it in seconds without Isaac:

```bash
python3 src/lunabot_bringup/test/isaac_double.py --no-wrap-positions
```

If the sim bringup test passes with that and fails without it, it is `sum_wrapped_joint_states` in
`lunabot.ros2_control.xacro`, and Isaac wraps exactly the same way.

---

## Step 6 — the comparison the sim exists for

```bash
ros2 launch lunabot_bringup sim.launch.py rviz:=true
```

`slam.rviz` displays `/odom` and `/sim/ground_truth/odom` together. **The gap between the two
trails is the odometry error** — the most direct read there is on whether a change helped.

Drive a square, then a spin in place. A yaw that consistently under- or over-shoots is
`wheel_separation_multiplier` in `controllers.yaml`, which is a real number to be measured and is
currently a placeholder of 1.5.

Vary `--seed` and `sun_azimuth_deg` between perception runs. A detector that only works with the
sun behind the rover is not a detector.

---

## What is still not covered after all this

Worth stating plainly, because a green checklist invites the opposite conclusion:

- **The friction numbers are unmeasured.** `static_friction: 0.7` decides whether a
  `wheel_separation_multiplier` tuned in sim means anything on regolith. Skid-steer turning is
  controlled friction failure. Calibrate against measured turning on the real rover.
- **The robot dimensions are placeholders.** Everything above passes with the wrong wheel radius;
  it just measures the wrong robot.
- **The boulder detector is a geometric stub.** It reports lumps above a plane. Scoring it against
  `/sim/ground_truth/boulders` needs that topic to exist — the seeded layout is written to JSON
  beside the expanded URDF today, and nothing publishes it yet.
- **cuVSLAM has never run.** It is scaffolding pending an Isaac ROS release for Jazzy. `slam:=rtabmap`
  is the backend to use.

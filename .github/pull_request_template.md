<!--
Delete any section that does not apply. An empty heading is worse than a
missing one.

The checklists below are not ceremony: every box is something that has
already broken this workspace once, silently. See CONTRIBUTING.md.
-->

## What and why

<!--
The diff says what changed. Say why it changed, and what happens if it is
wrong. If you disabled or worked around something, say what broke.
-->

## Contract

<!--
docs/TOPIC_FRAME_CONTRACT.md is authoritative. Delete this section if you
touched no topic, type, frame or QoS.
-->

- [ ] Topic, type, frame or QoS changes are in `docs/TOPIC_FRAME_CONTRACT.md`, **in this PR**
- [ ] Exactly one node publishes `odom → base_link` for every value of `odom_source:=`
- [ ] Nothing new in `lunabot_msgs` that already exists upstream
- [ ] `scripts/check_stack.py` and `scripts/robot_health.py` still describe the graph this produces

## Paired values

<!--
Constants that live in two files because no tool can reach across them. If
they drift, nothing errors -- the robot is just quietly wrong. Tick only the
pairs this PR touches.

The first four have a test behind them, so a mistake fails CI rather than the
robot. The last one does not: keep it by hand.
-->

- [ ] `wheel_radius` / `wheel_separation` — `properties.xacro` and `controllers.yaml`
- [ ] `command_timeout` — `lunabot.ros2_control.xacro` and `cmd_vel_timeout` in `controllers.yaml`
- [ ] Nav2 velocity limits and `diff_drive_controller`'s
- [ ] Footprint in `nav2_params.yaml` and the chassis in `properties.xacro`
- [ ] **Untested pair:** `lunabot.repos`, the rosdep skip-keys in `.github/workflows/ci.yml`, and
      the `--skip-keys` line in `README.md` — the same list wearing three hats

<!--
Enforced by lunabot_bringup/test/test_config_files.py (the first two) and
lunabot_navigation/test/test_nav2_params.py (the next two).
-->


## Sim and real run the same stack

- [ ] No node, config or branch that exists only under one `hw:=` value

<!--
If you wrote `if sim:`, stop and say here what the hardware plugin should be
doing instead. The whole workspace is built on this property: a bug that
reproduces in sim is a real bug only while it stays true.
-->

## How this was verified

<!--
Be specific and be honest. "Tests pass" is not a verification claim; which
tests, on what, is.

State what you did NOT run, and why. An unrun test named here costs a review
comment. An unrun test not named here costs a field failure.
-->

```
colcon test && colcon test-result --verbose
pre-commit run --all-files
```

- [ ] Ran on `hw:=mock`
- [ ] Ran on `hw:=sim`
- [ ] Ran on `hw:=real`
- [ ] Not run on hardware — what is unverified: <!-- … -->

## Unfinished work left behind

<!--
Mark it so it can be found, and say so here. A confident wrong number costs
far more than an obvious gap.

  PLACEHOLDER  a value that must be measured or specified
  VERIFY       an API or parameter name not checked against a real install
  TODO(2027)   deliberately deferred
-->

## New package

<!-- Delete unless this PR adds one. -->

- [ ] `package.xml` and `CMakeLists.txt`, `ament_cmake` unless it is genuinely an importable module
- [ ] `ament_lint_auto` wired into `BUILD_TESTING`, with the three duplicate linters switched off
- [ ] Added to the `package-name` list in `.github/workflows/ci.yml`
- [ ] At least one test that runs with no hardware and no GPU
- [ ] Listed in the package table in `README.md`

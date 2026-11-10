# lunabot_msgs

Interfaces with **no upstream equivalent**.

## The rule

> Nothing goes in this package that already exists upstream.

A custom message costs interoperability with every off-the-shelf node, RViz plugin, tracker and
recorder in the ecosystem, and it costs a full rebuild of everything downstream whenever it
changes. That price is worth paying roughly never.

Before adding a message, check `common_interfaces`, `vision_msgs`, `nav2_msgs`,
`control_msgs`, `diagnostic_msgs` and `geometry_msgs`. If something close exists, use it.

## What is here, and why it earned its place

**`MotorStatus` / `DriveStatus`** — SparkFlex bus voltage, output current, controller temperature
and vendor fault bits. There is no upstream home for these. `diagnostic_msgs/DiagnosticStatus` is
the usual suggestion and is stringly-typed key-value pairs, which cannot be plotted, cannot be
type-checked, and turns "graph bus voltage during a dig" into a parsing exercise.

**`Excavate` / `Deposit` (actions) and `ExcavatorStatus`** — the mission's interface to the
excavator (lunabot_excavation). The nearest upstream things, `control_msgs`'
FollowJointTrajectory and GripperCommand, move joints to positions; "dig until the hopper is full
or 20 s have passed, then stow" is a task with an outcome (full, out of time, cancelled, e-stop,
fault) that no joint command carries. Positions in the status are normalised 0..1 strokes, so the
interface survives the mechanical design changing underneath it.

## What is deliberately *not* here

**Boulder and crater detections.** They use `vision_msgs/Detection3DArray`.
`ObjectHypothesis.class_id` is a string, so `"boulder"` and `"crater"` are values of an existing
field — adding crater detection needs no message change and no downstream rebuild. A custom
`BoulderArray` would buy nothing and cost the RViz plugins and the Isaac ROS interop.

## Why this package existed early, with two messages in it

Creating an interfaces package mid-season forces a full workspace rebuild and a rename pass across
every consumer. It costs about thirty lines today. That is the whole argument.

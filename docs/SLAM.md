# SLAM

Visual-inertial SLAM from a single front-mounted OAK-D S2. No LIDAR.

```bash
ros2 launch lunabot_bringup robot.launch.py hw:=mock slam:=rtabmap rviz:=true
```

---

## Two backends, one contract

| | `rtabmap` | `cuvslam` |
|---|---|---|
| **Status** | **Default. Runs today.** | Scaffolding — see below |
| Compute | CPU. Pi 5, laptop, anything | NVIDIA GPU only |
| Input | RGB + depth aligned to RGB | Rectified stereo pair |
| Camera profile | `default` | `stereo_rect` |
| Output | 3D map + 2D occupancy grid | Pose + landmarks |
| Nav2 costmap | Yes, directly | Needs a separate grid source |

The two need **different camera configurations**, not just different topic remappings. That is why
`camera.launch.py` takes `profile:=` and why the backend selection has to reach the camera.

Switching is one argument:

```bash
ros2 launch lunabot_slam slam.launch.py backend:=rtabmap
ros2 launch lunabot_slam slam.launch.py backend:=cuvslam
```

Every topic name is declared once, in `slam.launch.py`, with defaults copied from
[`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md) and pinned by a test. Renaming a camera topic
is a one-line edit in one file — which is precisely what the 2026 layout could not do, with the
README, the launch files and the RViz config each holding their own copy.

---

## cuVSLAM is scaffolding, and honestly so

The launch file, parameters and remappings exist and are tested to construct. **The node is not
known to run.** Two things have to be true first:

1. **An Isaac ROS release supporting ROS 2 Jazzy must exist.** Isaac ROS binaries have historically
   targeted Ubuntu 22.04 / Humble. Check before promising this path to anyone.
2. **The rover must carry an NVIDIA GPU.** A Raspberry Pi 5 cannot run cuVSLAM at all. This is
   blocked on the 2027 compute decision — Jetson Orin or Pi.

`isaac_ros_visual_slam` is deliberately **not** declared in `lunabot_slam/package.xml`. Declaring a
dependency that may have no release for this distro would make the entire workspace un-installable
in order to get a backend that does not run. Instead the launch file logs a clear warning and the
node fails to start, which is a better failure than `rosdep` refusing to install anything.

**Verify before debugging:** the visual SLAM topic names changed around Isaac ROS 3.x, from
`stereo_camera/left/image` to a multi-camera `visual_slam/image_0` / `camera_info_0` scheme. The
launch file uses the newer names. Check with `ros2 node info /visual_slam_node` rather than
assuming the node is broken.

---

## Frame ownership

This is the part that goes wrong silently, so it gets a table.

| Transform | Owner | Controlled by |
|---|---|---|
| `map → odom` | the SLAM backend | `slam:=` |
| `odom → base_link` | `diff_drive_controller` \| SLAM backend \| `ekf_node` | `odom_source:=` |
| `base_link → *` | `robot_state_publisher` | always |

`odom_source` is a single switch precisely so the three candidates cannot all publish at once:

```bash
odom_source:=wheel    # diff_drive_controller (default)
odom_source:=visual   # rgbd_odometry or cuVSLAM
odom_source:=ekf      # robot_localization
```

Two publishers on one transform produces a TF tree that looks entirely correct in `view_frames` and
behaves nondeterministically — transform lookups return whichever arrived most recently. There is a
test asserting exactly one owner for every `odom_source` value.

---

## Why the wheel odometry cannot be trusted

The drivetrain has no encoders, so `diff_drive_controller`'s `/odom` is the commanded velocity
echoed back (see [`HARDWARE_CAN.md`](HARDWARE_CAN.md)). Right frames, right units, wrong the moment
a wheel slips — which on regolith is continuous.

For SLAM this means: **`/odom` is a motion prior, not a measurement.** rtabmap uses it to guess
where to look for the next frame's features, which is a genuinely useful thing for a bad odometry
source to do. It is not a source of truth, and `map → odom` is what makes the pose usable.

`robot_localization` ships configured and disabled for this reason. What it buys with no encoders is
narrow but real: the IMU's **yaw rate is a genuine measurement**, and it is the only thing that can
catch a skid-steer scrubbing through a turn. It cannot fix translation — nothing on the robot
measures distance travelled except the camera.

---

## rtabmap tuning, and what the lunar surface breaks

Regolith is **low-texture and self-similar**. That is the whole difficulty: it is a surface designed
to defeat feature-based visual SLAM. Three settings in `config/rtabmap.yaml` exist because of it:

**`Rtabmap/LoopThr: 0.15`** — a high similarity threshold before accepting a loop closure. A *false*
loop closure is far worse than none: it folds the map, and every pose after it is wrong. On a
surface where every patch looks like every other patch, false positives are the expected failure.

**`Vis/MinInliers: 20`** — more inliers required than default before accepting a transform. Same
reasoning: weak matches are common here and they are wrong.

**`Optimizer/GravitySigma: 0.3`** — fuse the IMU's gravity direction, loosely. Enough to stop the
map tilting, not so tight that a bumpy traverse fights it. This is why the URDF's `oak_d_imu_frame`
rotation must be correct: a wrong IMU orientation tilts the map with full confidence.

Plus `Reg/Force3DoF: true` and `Optimizer/Slam2D: true`, because the rover cannot roll, pitch or
change altitude in any way the estimator should believe. Left free, visual noise on those axes
propagates into the whole map.

### The one camera setting SLAM depends on

`stereo.i_align_depth: true` in `oak_d_s2.yaml`. rtabmap needs depth registered into the RGB frame.
Unaligned depth does not raise an error — it produces geometry that is subtly and consistently
wrong, which reads as a tuning problem for about a day.

---

## Testing

### Against a bag — the real functional test

No robot needed, and repeatable, which matters when tuning:

```bash
ros2 bag record /oak_d/rgb/image_rect /oak_d/rgb/camera_info \
                /oak_d/stereo/image_raw /oak_d/imu/data \
                /odom /tf /tf_static

ros2 launch lunabot_slam slam.launch.py backend:=rtabmap use_sim_time:=true
ros2 bag play <bag> --clock
```

`--clock` and `use_sim_time:=true` go together. Without both, rtabmap timestamps live data against
a clock the bag is not driving and drops nearly every frame.

### In simulation

Ground truth is published on `/sim/ground_truth/odom`, and the `slam.rviz` config has a
`GroundTruth` display alongside `Odometry`. **The gap between the two trails is the odometry
error** — that is the most direct read on whether a change helped.

### In CI

Only that every backend value constructs a valid launch description, plus config assertions on
frame names and the planar-rover settings. That is all that is possible: rtabmap needs data,
cuVSLAM needs a GPU. It still catches typos and missing substitutions, which is most of what
actually breaks in launch files.

---

## Troubleshooting

**No `map` frame.** No backend is running (`slam:=none`), or it has not initialised. rtabmap needs a
few synchronised frames before it publishes anything.

**rtabmap logs "Did not receive data since 5 seconds".** Topic names. Check
`ros2 topic info <topic>` for a publisher, then check the remappings — and remember
`i_enable_lazy_publisher: true` means the camera only produces frames when something is subscribed.

**Map geometry warped or scaled wrong.** `i_align_depth` did not take effect. Verify the parameter
name against your `depthai-ros` version rather than assuming the config applied.

**Map tilts over time.** IMU frame orientation. Check
`tf2_echo oak_d_link oak_d_imu_frame`, and confirm `Reg/Force3DoF` is actually `'true'` — rtabmap
takes these as strings and silently ignores a misspelled key. `rtabmap --params` lists the real
names.

**Map folds onto itself after driving in a loop.** A false loop closure. Raise `Rtabmap/LoopThr`.

**Robot jumps between two positions.** Two publishers on `odom → base_link`. Check `odom_source`
against the ownership table above.

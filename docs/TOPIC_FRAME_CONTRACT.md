# Topic and frame contract

**This file is the authority.** Any launch file, config, node or document that disagrees with it is
a bug — fix the code, not this table. If a name genuinely needs to change, change it here first and
in the same commit as the code.

This exists because the 2026 README documented `/camera/image_raw`, `/camera/depth/points` and
`/camera/camera_info` while the launch files actually published `/oak_d/rgb/image_raw/compressed`,
`/oak_d/stereo/image_raw` and `/oak_d/points`. Nobody noticed until someone tried to use the
documented names. One authoritative file, referenced by every launch file's default arguments, is
the fix.

---

## Frames

```
map                                          ← SLAM backend
└── odom                                     ← SLAM backend
    └── base_link                            ← odom source
        ├── base_footprint                   ← robot_state_publisher
        ├── front_left_wheel_link            ← robot_state_publisher
        ├── front_right_wheel_link           ← robot_state_publisher
        ├── rear_left_wheel_link             ← robot_state_publisher
        ├── rear_right_wheel_link            ← robot_state_publisher
        └── oak_d_link                       ← robot_state_publisher
            ├── oak_d_rgb_camera_frame
            │   └── oak_d_rgb_camera_optical_frame
            ├── oak_d_left_camera_frame
            │   └── oak_d_left_camera_optical_frame
            ├── oak_d_right_camera_frame
            │   └── oak_d_right_camera_optical_frame
            └── oak_d_imu_frame
```

| Frame | Convention | Notes |
|---|---|---|
| `map` | REP-105 | World-fixed, discontinuous (jumps on loop closure) |
| `odom` | REP-105 | Continuous, drifts without bound |
| `base_link` | REP-103 (X fwd, Y left, Z up) | **TF root.** Chassis geometric centre, at wheel-axle height |
| `base_footprint` | REP-103 | Child of `base_link` at `z = -wheel_radius`. Ground projection |
| `*_wheel_link` | REP-103 | Rotates about local **Y** |
| `oak_d_link` | REP-103 | Camera mount body frame |
| `oak_d_*_optical_frame` | REP-103 optical (Z fwd, X right, Y down) | rpy `-π/2, 0, -π/2` from its parent |
| `oak_d_imu_frame` | REP-103 | |

### Transform ownership

Exactly one publisher per transform. Two publishers on one transform gives a tree that looks
correct in `view_frames` and behaves nondeterministically.

| Transform | Owner | Selected by |
|---|---|---|
| `map → odom` | `rtabmap` (`publish_tf: true`) or cuVSLAM (`publish_map_to_odom_tf: true`) | `backend:=` |
| `odom → base_link` | `diff_drive_controller` \| cuVSLAM \| `ekf_node` | `odom_source:=` |
| `base_link → *` | `robot_state_publisher` | always |

`odom_source` settings and what each turns off:

| `odom_source:=` | publishes `odom → base_link` | must be off |
|---|---|---|
| `wheel` (default) | `diff_drive_controller` (`enable_odom_tf: true`) | cuVSLAM odom TF, EKF |
| `visual` | cuVSLAM (`publish_odom_to_base_tf: true`) | `diff_drive_controller` odom TF, EKF |
| `ekf` | `robot_localization/ekf_node` | `diff_drive_controller` odom TF, cuVSLAM odom TF |

### Why `base_link` is the root

Most robots make `base_footprint` the root and hang `base_link` off it. We invert that so exactly
one node owns `odom → base_link` and nobody has to decide who owns `base_footprint`. Nav2's
`robot_base_frame` stays `base_link`.

### Camera frame prefix

**The prefix is the driver's NODE NAME, not a parameter.** `depthai_ros_driver` builds every
published `header.frame_id` in `sensor_helpers.cpp::tfPrefix()`:

```cpp
if (camera.i_publish_tf_from_calibration)  return camera.i_tf_base_frame;
return node->get_name();
```

So exactly one setting is needed, and one name has to match:

```yaml
camera:
  i_publish_tf_from_calibration: false   # we own the TF tree, not the driver
```

with the node named `oak_d` in `camera.launch.py` (`DRIVER_NODE_NAME`). That name is why the
driver's frames are `oak_d_*` and match the URDF; renaming the node detaches every camera topic
from the TF tree with no error anywhere. `test_launch_descriptions.py` cross-checks the name
against the URDF.

`i_publish_tf_from_calibration: false` is the load-bearing one. Left true, the driver injects its
own camera transforms and fights `robot_state_publisher` — and the prefix then comes from
`i_tf_base_frame` instead.

> **RESOLVED** against `depthai-ros` 2.12.2, the version Jazzy ships. This block previously
> specified `i_tf_tf_prefix: oak_d`, which **is not a parameter depthai-ros declares** — ROS 2
> keeps undeclared YAML keys as initial values and never applies them, so it was silently ignored
> and the frames were correct only because the node happened to be named `oak_d`. A test asserted
> it too, and passed, because it read the same YAML the config wrote.

---

## Topics

### Control

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `teleop_twist_joy` |
| `/cmd_vel_joy` | `geometry_msgs/TwistStamped` ¹ | `teleop_twist_joy` | `twist_mux` |
| `/cmd_vel_nav_unsmoothed` | `geometry_msgs/TwistStamped` ¹ | `controller_server`, `behavior_server` | `velocity_smoother` |
| `/cmd_vel_nav` | `geometry_msgs/TwistStamped` ¹ | `velocity_smoother` | `twist_mux` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` ¹ | `twist_mux` | `diff_drive_controller` |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `robot_state_publisher` |
| `/odom` | `nav_msgs/Odometry` | odom source | rtabmap, Nav2, EKF |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | see ownership table | everything |
| `/drive/status` | `lunabot_msgs/DriveStatus` | `SparkFlexSystem` | `robot_health`, plots |

¹ **RESOLVED. The whole chain is `TwistStamped`, and every switch is set.**

`diff_drive_controller` in Jazzy subscribes to `geometry_msgs::msg::TwistStamped` on `~/cmd_vel`
unconditionally — there is no `use_stamped_vel` parameter to get wrong
(`ros2_controllers` jazzy, `diff_drive_controller.cpp`, `create_subscription<TwistStamped>`). It
reads only the **timestamp**, for `cmd_vel_timeout`; `header.frame_id` is never looked at, and a
zero stamp is replaced with the current time and a warning.

Every upstream node that feeds it therefore has to be told to stamp, and each was checked against
the version Jazzy ships rather than against documentation:

| Node | Parameter | Default | Ours | Verified against |
|---|---|---|---|---|
| `teleop_twist_joy` | `publish_stamped_twist` | `false` | `true` | 2.6.5, `teleop_twist_joy.cpp` |
| `twist_mux` | `use_stamped` | `true` | `true` | 4.5.0, `twist_mux.cpp` |
| `controller_server`, `behavior_server`, `velocity_smoother` | `enable_stamped_cmd_vel` | `false` | `true` | Nav2 1.3.12, `nav2_util/twist_publisher.hpp` |

Two of those three default to `false`, so they are not optional and not decoration. Nav2's is
declared by `nav2_util::TwistPublisher`/`TwistSubscriber` on whichever node constructs one, which
is why it is set per node rather than once.

Getting any of them wrong produces a robot that silently does not move, with no error anywhere:
the publisher and the subscriber simply never match.

`/cmd_vel_nav_unsmoothed` is internal to Nav2 — the hop from the controller and the recovery
behaviours into `velocity_smoother`, so that everything Nav2 commands is acceleration-limited and
`/cmd_vel_nav` has exactly one publisher. Nothing outside `lunabot_navigation` should subscribe to
it. See [`NAVIGATION.md`](NAVIGATION.md).

### Camera

Published by the depthai driver on hardware, and by Isaac Sim's `ROS2CameraHelper` graphs in sim.
Same names either way — that is the point.

| Topic | Type | Consumer |
|---|---|---|
| `/oak_d/rgb/image_raw` | `sensor_msgs/Image` | debug |
| `/oak_d/rgb/image_rect` | `sensor_msgs/Image` | rtabmap |
| `/oak_d/rgb/camera_info` | `sensor_msgs/CameraInfo` | rtabmap |
| `/oak_d/stereo/image_raw` | `sensor_msgs/Image` — **see encoding note** ² | rtabmap, `depth_image_proc` |
| `/oak_d/stereo/camera_info` | `sensor_msgs/CameraInfo` | `depth_image_proc` |
| `/oak_d/left/image_rect` | `sensor_msgs/Image` | cuVSLAM |
| `/oak_d/right/image_rect` | `sensor_msgs/Image` | cuVSLAM |
| `/oak_d/left/camera_info` | `sensor_msgs/CameraInfo` | cuVSLAM |
| `/oak_d/right/camera_info` | `sensor_msgs/CameraInfo` | cuVSLAM |
| `/oak_d/points` | `sensor_msgs/PointCloud2` | `boulder_detector` |
| `/oak_d/imu/data` | `sensor_msgs/Imu` | rtabmap, EKF, cuVSLAM |

² **Depth encoding differs between hardware and sim.**

| | encoding | units |
|---|---|---|
| OAK-D S2 via depthai | `16UC1` | millimetres |
| Isaac Sim depth annotator | `32FC1` | metres |

`rtabmap` happens to accept both, which is exactly what makes this dangerous — it works until
something we wrote reads the topic and is silently off by 1000×. Two consequences, both load-bearing:

1. `depth_normalizer_node` (in `lunabot_perception`) converts `16UC1` mm → `32FC1` m. It runs
   **on the real robot only**. Everything downstream sees one encoding.
2. `boulder_detector` consumes `/oak_d/points`, not the depth image. The point cloud is metres in
   both cases, so it is byte-comparable between sim and hardware.

`/oak_d/points` is produced by `depth_image_proc/point_cloud_xyz_node` on the robot (decimation 4,
max range 2.0 m, tuned for Pi bandwidth) and by Isaac's `depth_pcl` camera helper in sim.

### Perception

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/perception/boulders` | `vision_msgs/Detection3DArray` | `base_link` | `class_id: "boulder"` |
| `/perception/debug/ground` | `sensor_msgs/PointCloud2` | `base_link` | RViz only, not contract |
| `/perception/debug/obstacles` | `sensor_msgs/PointCloud2` | `base_link` | RViz only, not contract |
| `/perception/debug/markers` | `visualization_msgs/MarkerArray` | `base_link` | RViz only, not contract |

Craters, when they arrive, publish to the **same** `/perception/boulders` topic with
`class_id: "crater"`. `ObjectHypothesis.class_id` is a string in `vision_msgs`, so this needs no
message change and no downstream rebuild. (The topic name will read oddly at that point. Renaming
it to `/perception/detections` is a one-line change here plus the launch defaults — do it then, not
speculatively now.)

### Navigation

| Topic | Type | Publisher |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | rtabmap's grid, or `map_server` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz, autonomy |

### Monitoring

| Topic | Type | Publisher | Consumer |
|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `robot_health`, `ekf_node` ³ | Foxglove, PlotJuggler, the aggregator |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_aggregator` | `rqt_robot_monitor`, and nothing else |

Both are started by `diagnostics.launch.py`, which `robot.launch.py` includes by default. Turn them
off with `diagnostics:=false`. See [`MONITORING.md`](MONITORING.md).

³ `ekf_node` contributes its own statuses only under `odom_source:=ekf`; `print_diagnostics` is set
in `ekf.yaml`. Several publishers on `/diagnostics` is normal and is the opposite of the `/tf`
situation — a `DiagnosticStatus` carries its own name, so the aggregator merges rather than fights.

### Simulation only

| Topic | Type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | **Isaac only.** Every node needs `use_sim_time:=true` |
| `/isaac/joint_states` | `sensor_msgs/JointState` | Isaac → `TopicBasedSystem` |
| `/isaac/joint_commands` | `sensor_msgs/JointState` | `TopicBasedSystem` → Isaac |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | **Evaluation only. Never enters `/tf`** |
| `/sim/ground_truth/boulders` | `vision_msgs/Detection3DArray` | Seeded scene layout, for scoring. **NOT PUBLISHED YET** |

Ground truth is published in the **same message type the detector emits**, so scoring the detector
is a direct comparison rather than a format conversion.

> **`/sim/ground_truth/boulders` does not exist on the wire.** The data does: `run_sim.py` writes
> `ground_truth_seed<N>.json` next to the expanded URDF, with every boulder's true pose and extent,
> and `scene/boulders.py::to_ground_truth` is the function that builds it. What is missing is
> something ROS-side to read that file and publish it — `lunabot_sim` cannot, because it runs under
> Isaac's Python and imports no `rclpy` on purpose.
>
> The name is reserved here rather than deleted because the sidecar is written in the shape it will
> be published in. Do not go looking for the topic on the simulation machine; it is not a
> misconfiguration. See [`SIM_ACCEPTANCE.md`](SIM_ACCEPTANCE.md).

`use_sim_time` must reach *every* node — `controller_manager`, `robot_state_publisher`, the
spawners, rtabmap, Nav2, perception, `twist_mux`, and **RViz**. RViz is the one people forget, and
the symptom is TF displays that freeze or lag rather than an error.

---

## Networking

Carried over unchanged from the 2026 setup, on both the robot and the offboard machine:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

with `~/.ros/cyclonedds.xml` raising `SocketReceiveBufferSize` to 10 MB for image topics. See
[`OAK_D_S2_INTEGRATION.md`](OAK_D_S2_INTEGRATION.md).

Isaac Sim's ROS 2 bridge must use the same domain ID — set it on the `ROS2Context` OmniGraph node
or enable that node's `useDomainIDEnvVar` input.

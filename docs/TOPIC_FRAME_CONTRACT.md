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

The depthai driver node is named `oak_d`, and it will publish `header.frame_id` values derived from
its own prefix. Two parameters must be set or the driver's frames will not match the URDF's:

```yaml
camera:
  i_tf_tf_prefix: oak_d
  i_publish_tf_from_calibration: false   # we own the TF tree, not the driver
```

`i_publish_tf_from_calibration: false` is the important one. Left true, the driver injects its own
camera transforms and fights `robot_state_publisher`.

> **VERIFY** both parameter names against the installed `depthai-ros` version. They have moved
> between releases.

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
| `/drive/status` | `lunabot_msgs/DriveStatus` | `SparkFlexSystem` | diagnostics |

¹ **UNRESOLVED — resolve on day one and record the answer here.**
In ROS 2 Jazzy, `diff_drive_controller` subscribes to `geometry_msgs/msg/TwistStamped` on
`~/cmd_vel` and the `use_stamped_vel` parameter has been removed. Confirm with:

```bash
ros2 topic info /diff_drive_controller/cmd_vel -v
ros2 param list /diff_drive_controller
```

The answer decides whether `teleop_twist_joy` needs `publish_stamped_twist: true`, whether
`twist_mux` needs `use_stamped: true`, and whether Nav2 needs `enable_stamped_cmd_vel: true` on
every node that publishes velocity. Getting it wrong produces a robot that silently does not move,
with no error anywhere — budget an afternoon if you skip this check.

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

### Simulation only

| Topic | Type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | **Isaac only.** Every node needs `use_sim_time:=true` |
| `/isaac/joint_states` | `sensor_msgs/JointState` | Isaac → `TopicBasedSystem` |
| `/isaac/joint_commands` | `sensor_msgs/JointState` | `TopicBasedSystem` → Isaac |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | **Evaluation only. Never enters `/tf`** |
| `/sim/ground_truth/boulders` | `vision_msgs/Detection3DArray` | Seeded scene layout, for scoring |

Ground truth is published in the **same message type the detector emits**, so scoring the detector
is a direct comparison rather than a format conversion.

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

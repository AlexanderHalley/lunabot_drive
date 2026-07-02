# Isaac Sim assets

USD stages and OmniGraphs are authored inside Isaac Sim, not as text files, so
they are not committed as source. Commit the exported `.usd` files here once they
exist, and document the setup below.

## Files to produce
| File | How |
|------|-----|
| `lunabot.usd` | URDF Importer on `lunabot_description` output; wheels = velocity drive |
| `regolith_arena.usd` | Model the Lunabotics sandbox; BP-1-like ground material + physics |

## ROS 2 Bridge action graph (OmniGraph) — nodes to add
- `ROS2 Context` + `ROS2 Publish Clock` (drive `/clock`; set `use_sim_time:=true`).
- `ROS2 Publish Transform Tree` (TF from the articulation).
- `ROS2 Publish Odometry` -> `/odom` (or let diff_drive_controller do it in Stage 2).
- Camera: `ROS2 Camera Helper` -> `/camera/image_raw`, `/camera/depth/*`,
  `/camera/depth/points` (match 2026 topic names so perception is unchanged).
- IMU: `ROS2 Publish Imu` -> `/imu`.
- (If LiDAR added) `ROS2 RTX Lidar Helper` -> `/scan` or `/points`.
- Wheels: `ROS2 Subscribe JointState` (<- `/isaac/joint_commands`) driving the
  articulation, and `ROS2 Publish JointState` (-> `/isaac/joint_states`) for
  Stage-1 topic_based_ros2_control.

## Gotchas
- Set every ROS node's `nodeNamespace`/topic to match `lunabot_control` remaps.
- Enable `use_sim_time` on ALL ROS nodes when driving from Isaac's `/clock`.

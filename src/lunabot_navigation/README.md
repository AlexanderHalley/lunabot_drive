# lunabot_navigation

The autonomy stack — the part Lunabotics scores most heavily. Built and validated
in Isaac Sim (Phase 2), then run unchanged on hardware (Phase 3).

## Layers (bottom-up, plan §5)
1. **Localization** — `config/ekf.yaml`: `robot_localization` EKF fusing wheel
   odom (from diff_drive_controller) + IMU + visual odometry, with AprilTag pose
   resets to kill drift. GPS-denied, high-slip regolith is the hard part.
2. **Mapping** — `slam_toolbox` (LiDAR) or costmap-only obstacle avoidance from
   the OAK-D pointcloud if no LiDAR.
3. **Navigation** — `config/nav2_params.yaml`: Smac/NavFn planner + MPPI or
   Regulated Pure Pursuit controller (MPPI copes with skid-steer slip), recovery
   behaviors tuned for getting un-stuck in loose regolith.
4. **Mission** — `behavior_trees/lunabotics_mission.xml`: sequence
   NavigateTo(dig) -> Excavate -> NavigateTo(berm) -> Deposit, with retries and a
   teleop-fallback branch.

## Command path
Nav2 publishes to `/cmd_vel_nav`; twist_mux (in `lunabot_control`) lets teleop or
e-stop override it. Autonomy never has exclusive control of the drivetrain.

## Status
Skeletons. `nav2_params.yaml` and `ekf.yaml` are realistic starting points to tune
in sim; the behavior tree is a structural stub. The excavation subsystem
(actuators + its own controller) is a parallel workstream — see plan §5.5.

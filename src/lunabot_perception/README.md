# lunabot_perception

Sensing for autonomy. Inherits the 2026 OAK-D work (move the camera launch/config
from `lunabot_drive` here) and adds what navigation needs.

## Responsibilities
- **OAK-D S2 bring-up** — RGB/depth/pointcloud (reuse 2026 configs; keep the
  CycloneDDS large-image tuning for the distributed setup).
- **Obstacle input for Nav2** — `pointcloud_to_laserscan` (or feed the pointcloud
  straight into a Nav2 voxel/obstacle layer). Craters + rocks on regolith.
- **AprilTag detection** (`apriltag_ros`) — absolute pose fixes at the arena
  start/berm to reset odometry drift. This is the cheapest big autonomy-score win
  in a GPS-denied arena (see plan §5.1).
- **(Optional) Visual odometry / VIO** — primary localization when wheels slip.

## Sim parity
Isaac publishes the same camera/IMU/scan topic names, so these nodes run
unchanged against sim or hardware.

## Status
Skeleton — launch/config to be populated by migrating from `lunabot_drive` and
adding the pointcloud + AprilTag pipelines.

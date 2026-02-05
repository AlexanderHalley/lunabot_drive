# Camera-Only Nav2 Testing

Test the depth camera → costmap pipeline without motors or LIDAR.

## Quick Start

```bash
# Terminal 1 (Pi): Launch camera
ros2 launch lunabot_drive oak_d_camera.launch.py

# Terminal 2 (Offboard PC): Launch nav2 costmap + RViz
ros2 launch lunabot_drive camera_only_nav2.launch.py
```

## What This Tests

| Working | Not Working |
|---------|-------------|
| Depth → PointCloud2 → costmap | Odometry / localization |
| TF transforms | Global path planning |
| Obstacle detection & clearing | Autonomous navigation |
| Bandwidth profiling | |

## RViz Displays

- **TF**: Verify `map → odom → base_link → oak_d_link` chain
- **PointCloud2**: Raw depth points from camera
- **Local Costmap**: Obstacles should appear when objects are in front of camera

## Troubleshooting

**No pointcloud visible:**
```bash
ros2 topic hz /oak_d/points  # Should show ~15 Hz
```

**TF errors:**
```bash
ros2 run tf2_tools view_frames  # Check for broken links
```

**Costmap not updating:**
```bash
ros2 topic echo /local_costmap/costmap --once  # Check if publishing
```

## Files

| File | Purpose |
|------|---------|
| `config/camera_only_nav2.yaml` | Nav2 costmap parameters |
| `launch/camera_only_nav2.launch.py` | Launch with static TFs |
| `config/rviz/camera_only_nav2.rviz` | RViz visualization config |

## Frame Names

The depthai driver (node name: `oak_d`) publishes:
- `oak_d_link` - camera body frame
- `oak_d_rgb_camera_optical_frame` - RGB optical frame
- `oak_d_right_camera_optical_frame` - depth/stereo optical frame

The launch file creates static transforms:
```
map → odom → base_link → oak_d_link
```

## Next Steps

Once motors are connected, the real joint states from `drive_node` will replace the static `odom → base_link` transform, enabling odometry and full navigation testing.

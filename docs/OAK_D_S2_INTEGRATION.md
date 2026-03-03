# OAK-D S2 Camera Integration

Camera connects to Pi 5 via USB 3.0 (blue port). Driver: `depthai_ros_driver`, node name: `oak`.

---

## Quick Start

```bash
# Pi: launch camera
ros2 launch lunabot_drive oak_d_camera.launch.py

# Pi: launch camera + pointcloud (bandwidth-optimized)
ros2 launch lunabot_drive oak_d_camera.launch.py \
  config:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `/oak/rgb/image_raw` | sensor_msgs/Image | RGB stream |
| `/oak/stereo/image_raw` | sensor_msgs/Image | Depth image |
| `/oak/points` | sensor_msgs/PointCloud2 | 3D point cloud |
| `/oak/imu/data` | sensor_msgs/Imu | BNO086 IMU at 100 Hz |

## TF Frames

Published by depthai driver (node name: `oak`):
```
base_link → oak → oak_rgb_camera_optical_frame
                → oak_right_camera_optical_frame
```

## Competition: Compass Disable

The BNO086 has a magnetometer. GPS/compass is prohibited by competition rules. Verify:
```bash
ros2 param get /oak imu.i_enable_rotation
# Must be: False (default)
```

Do not fuse magnetometer axes in the EKF config (`ekf_params.yaml`).

## USB 3.0 Check

```bash
lsusb -t | grep 5000M
# Camera (03e7:2485 MyriadX) must appear under a 5000M hub
# If under 480M: wrong port or cable (use data cable, not charge-only)
```

## udev Rule (one-time setup)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## DDS Config for Large Topics

Create `~/.ros/cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
```

Add to `~/.bashrc` on both Pi and PC:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

## Bandwidth Tuning

Key knobs in `config/oak_d_pointcloud_only.yaml` (ordered by impact):

| Parameter | Default | Aggressive |
|---|---|---|
| `i_decimation_filter_decimation_factor` | 2 | 3–4 |
| `i_fps` | 10 | 5 |
| `i_threshold_filter_max_range` | 5000 mm | 3000 mm |
| `i_stereo_conf_threshold` | 200 | 230 |

Target: < 2 Mbps total for pointcloud alone (4 Mbps competition limit).

## Troubleshooting

**Camera not detected:**
```bash
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
lsusb | grep Movidius
```

**Topics not visible on PC:**
```bash
echo $ROS_DOMAIN_ID  # Must match on both machines
ros2 daemon stop && ros2 daemon start
```

**No pointcloud publisher:**
```bash
ros2 topic info /oak/points  # Publisher count must be 1
ros2 param get /oak pointcloud.i_enable  # Must be True
```

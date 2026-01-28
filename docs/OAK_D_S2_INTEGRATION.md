# OAK-D-S2 Camera Integration Roadmap

This document outlines the integration of an OAK-D-S2 stereo depth camera into the Lunabot system, with the camera connected to a Raspberry Pi 5 via USB-C and relayed to an offboard computer via ROS2 DDS.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                           │
│  ┌─────────────┐    ┌──────────────────┐                   │
│  │  OAK-D-S2   │───▶│ depthai_ros_driver│                   │
│  │  (USB-C)    │    │  /camera/*        │                   │
│  └─────────────┘    └────────┬─────────┘                   │
│                              │ DDS                          │
└──────────────────────────────┼──────────────────────────────┘
                               │ WiFi/Ethernet
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                   Offboard Computer                          │
│  ┌──────────────┐  ┌────────────┐  ┌───────────────────┐    │
│  │ ball_tracker │  │    Nav2    │  │      RViz2        │    │
│  │ /cmd_vel_*   │  │ (obstacle) │  │ Image/PointCloud  │    │
│  └──────────────┘  └────────────┘  └───────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Hardware & Driver Setup on Raspberry Pi

### 1.1 Physical Connection
- Connect OAK-D-S2 to Pi 5 via USB-C (use a USB 3.0 port for full bandwidth)
- Ensure adequate power — OAK-D-S2 draws ~2.5W; consider powered USB hub if issues arise

### 1.2 Install DepthAI SDK on Pi
```bash
# On Raspberry Pi 5
sudo apt update
sudo apt install python3-pip libusb-1.0-0-dev

# Install DepthAI
python3 -m pip install depthai

# Test camera detection
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
```

### 1.3 Install depthai-ros
```bash
# If using ROS2 Jazzy (adjust for your distro)
sudo apt install ros-jazzy-depthai-ros

# Or build from source for latest features:
cd ~/ros2_ws/src
git clone https://github.com/luxonis/depthai-ros.git
cd ~/ros2_ws && colcon build --packages-select depthai_ros_driver
```

### 1.4 udev Rules (for non-root access)
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Phase 2: Network/DDS Configuration

### 2.1 Set Common ROS_DOMAIN_ID
```bash
# Add to ~/.bashrc on BOTH Pi and offboard computer
export ROS_DOMAIN_ID=42  # Pick any 0-232
```

### 2.2 (Recommended) Use CycloneDDS for better multicast
```bash
# On both machines
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2.3 DDS Config for Large Image Topics

Create `~/.ros/cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
```

Export in `~/.bashrc`:
```bash
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

---

## Phase 3: Launch the Camera

### 3.1 On Raspberry Pi
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch camera node
ros2 launch lunabot_drive oak_d_camera.launch.py
```

### 3.2 Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | RGB image |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth image |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | 3D point cloud |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

---

## Phase 4: Testing & Visualization

### 4.1 Verify on Pi
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

### 4.2 Verify Network Relay (on offboard computer)
```bash
# Should see camera topics from Pi
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --no-arr
```

### 4.3 RViz2 Visualization
```bash
ros2 run rviz2 rviz2
```

Add these displays:
- **Image**: Topic `/camera/image_raw`
- **PointCloud2**: Topic `/camera/depth/points`
- **Camera**: Topic `/camera/camera_info` with `/camera/image_raw`

### 4.4 Quick Image Viewer
```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw from dropdown
```

---

## Phase 5: Bandwidth Optimization

If network bandwidth is insufficient, adjust camera parameters:

### Option A: Reduce Resolution/Framerate
Edit `config/oak_d_camera.yaml`:
```yaml
rgb_resolution: '720P'  # Lower from 1080P
rgb_fps: 10             # Lower from 15
```

### Option B: Use Compressed Transport
```bash
sudo apt install ros-jazzy-image-transport-plugins

# Topics appear at:
# /camera/image_raw/compressed
# /camera/depth/image_raw/compressedDepth
```

### Option C: Disable Unused Streams
```yaml
enable_depth: false      # If only using RGB
enable_pointcloud: false # If only using depth image
```

---

## Troubleshooting

### Camera Not Detected
```bash
# Check USB connection
lsusb | grep Movidius
# Should show: "03e7:2485 Intel Movidius MyriadX"

# Check depthai can see it
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
```

### Topics Not Visible on Offboard Computer
```bash
# Verify same ROS_DOMAIN_ID on both machines
echo $ROS_DOMAIN_ID

# Check network connectivity
ping <pi_ip_address>

# Verify DDS discovery
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

### Low Framerate / High Latency
- Check WiFi signal strength
- Use Ethernet if available
- Reduce resolution/framerate in config
- Enable compressed image transport

### Permission Denied on USB
```bash
# Re-apply udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Or run with sudo (not recommended for production)
sudo -E ros2 launch lunabot_drive oak_d_camera.launch.py
```

---

## URDF Integration (Optional)

If you need the camera in your robot model for TF transforms, add to your URDF:

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.091 0.028 0.017"/>
    </geometry>
  </visual>
</link>

<!-- Mount to chassis -->
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.5 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Optical frame (Z forward, X right, Y down) -->
<link name="camera_link_optical"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_link_optical"/>
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>
```

---

## Files Added

| File | Purpose |
|------|---------|
| `launch/oak_d_camera.launch.py` | Launch depthai_ros_driver on Pi |
| `config/oak_d_camera.yaml` | Camera parameters (resolution, fps, etc.) |
| `docs/OAK_D_S2_INTEGRATION.md` | This roadmap document |

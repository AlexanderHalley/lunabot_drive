# Camera Bring-Up & Test Plan (OAK-D S2)

This plan covers what is needed on each machine to get the OAK-D S2 publishing
ROS topics that show up on the offboard PC, and how to verify it tonight.

- **dreamfyre** — offboard PC (x86_64), runs RViz and dashboard.
- **lunapi** — Raspberry Pi 5. The camera, motors, and actuators are wired here.

## TL;DR

The camera plumbing in this repo gets you **camera image + pointcloud +
TF tree**, plus pointcloud → laserscan and pointcloud → Nav2 obstacle
costmap. It does **not** build a SLAM map. See
"Does this build a map?" at the bottom.

---

## 1. Pi-side prerequisites (one-time)

```bash
# ROS packages — the depthai driver is what publishes /oak/* topics
sudo apt update
sudo apt install \
  ros-jazzy-depthai-ros-driver ros-jazzy-depthai \
  ros-jazzy-image-proc ros-jazzy-depth-image-proc \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-robot-localization \
  ros-jazzy-apriltag-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-web-video-server

# udev rule so non-root can talk to MyriadX over USB
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Workspace
cd ~/ros2_ws && colcon build --packages-select lunabot_drive
source ~/ros2_ws/install/setup.bash
```

Pi `~/.bashrc` must export the same ROS env as dreamfyre:

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds_pi.xml
```

The Pi-side Cyclone XML should mirror `~/cyclonedds_pc.xml` on dreamfyre, with
its `<Peers>` list including dreamfyre's IP, and `<NetworkInterfaceAddress>`
set to whichever interface is on the test network (likely `wlan0` on the Pi).

### Plug-in sanity check

```bash
lsusb | grep 03e7              # expect: Intel Movidius MyriadX
lsusb -t | grep 5000M          # must be on a 5000M (USB 3) hub, not 480M
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
```

If `lsusb -t` shows the camera under a 480M hub, it's plugged into a USB-2
port (or a charge-only cable). Move it to a blue port with a data cable.

---

## 2. dreamfyre (PC) prerequisites

Already verified on this machine:

| Check | Status |
|---|---|
| `ROS_DOMAIN_ID=30` | ✓ |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | ✓ |
| `CYCLONEDDS_URI` → `~/cyclonedds_pc.xml` | ✓ |
| `~/cyclonedds_pc.xml` exists, interface `wlp3s0`, peers `10.105.105.211` / `192.168.8.150` | ✓ |
| `lunabot_drive` installed in `~/ros2_ws/install/` | ✓ |
| `pointcloud_to_laserscan`, `robot_state_publisher`, `image_proc`, `depth_image_proc` | ✓ |

Two things to confirm before testing:

1. The Pi's IP on tonight's network is one of the two `<Peer>` entries in
   `~/cyclonedds_pc.xml`. If not, edit the file and add it.
2. The pending diff on `launch/oak_d_rviz.launch.py` (removes the
   `joint_state_publisher_gui` from the PC launch) is the right shape for
   testing over the LAN — `drive_node` on the Pi publishes `/joint_states`.

`depthai_ros_driver` itself does **not** need to be installed on dreamfyre.
RViz subscribes and renders the camera topics directly.

---

## 3. Test sequences

Run each in two terminals — one ssh'd to lunapi, one on dreamfyre.

### A. Camera only (fastest sanity check)

```bash
# Pi
ros2 launch lunabot_drive oak_d_camera.launch.py

# PC
ros2 launch lunabot_drive oak_d_rviz.launch.py launch_camera:=false
```

In RViz: select Fixed Frame = `oak`, add Image display on `/oak/rgb/image_raw`,
add PointCloud2 display on `/oak/stereo/points`. You should see the image at
~15 Hz and a pointcloud rendering the scene.

### B. Camera + pointcloud (bandwidth-optimized, autonomy config)

```bash
# Pi
ros2 launch lunabot_drive oak_d_camera.launch.py \
  config:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml

# PC
ros2 launch lunabot_drive oak_d_rviz.launch.py launch_camera:=false
```

Topics will publish at 5 Hz RGB, 10 Hz pointcloud — the same config used by
`autonomy_bringup_pi`.

### C. Full hardware bringup (camera + motors + EKF)

```bash
# Pi
ros2 launch lunabot_drive hardware_bringup.launch.py enable_teleop:=false

# PC
ros2 launch lunabot_drive oak_d_rviz.launch.py launch_camera:=false
```

This launches `drive_node`, the camera, `robot_state_publisher`, and the EKF
on the Pi. Use this when you want the full TF tree
(`map → odom → base_link → wheel_*` and `base_link → oak → optical frames`).

### D. Camera + pointcloud → laserscan (closest to mapping flow)

```bash
# Pi
ros2 launch lunabot_drive oak_d_camera.launch.py \
  config:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml

# PC
ros2 launch lunabot_drive camera_mapping_test.launch.py launch_camera:=false use_rsp:=true
```

Adds the `pointcloud_to_laserscan` node and publishes `/scan`. RViz will show
a LaserScan in the `base_link` frame. **Still no SLAM map** — see below.

---

## 4. Verification checklist (PC side, separate terminal)

```bash
# Topics from the Pi visible?
ros2 topic list | grep oak

# Rates
ros2 topic hz /oak/rgb/image_raw      # ~5 Hz (pointcloud config) or ~15 Hz (default)
ros2 topic hz /oak/stereo/points      # ~10 Hz
ros2 topic hz /oak/imu/data           # batched, ~100 Hz effective
ros2 topic bw /oak/stereo/points      # must stay under ~2 Mbps for competition

# TF tree present?
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link oak
```

If `ros2 topic list` shows nothing from the Pi:

1. Confirm `ROS_DOMAIN_ID` matches on both machines.
2. Confirm Pi's IP is in `~/cyclonedds_pc.xml` `<Peers>`.
3. Confirm same network / same subnet (`ping <pi-ip>` from dreamfyre).
4. `ros2 daemon stop && ros2 daemon start` on both sides.

---

## 5. Known config fixes applied (May 2026)

The configs in `config/oak_d_*.yaml` previously had three errors that would
either fail silently (params ignored) or throw "unknown parameter" warnings:

- `oak_d_camera.yaml` / `oak_d_rgb_only.yaml`: pipeline params lived under
  a `pipeline_gen:` group. They actually live under `camera:` in the upstream
  driver. `i_mx_id` and `i_usb_speed` were also at the wrong indent and have
  been moved into `camera:`.
- `oak_d_pointcloud_only.yaml`: `imu.i_imu_update_rate` is not a valid key in
  `depthai_ros_driver`. Replaced with `imu.i_acc_freq: 100` and
  `imu.i_gyro_freq: 100` (plus `i_enable_rotation: false` for compass-disable
  compliance).

Reference for param layout:
- `camera:` — `i_mx_id`, `i_usb_speed`, `i_pipeline_type`, `i_nn_type`,
  `i_enable_imu`, `i_enable_ir`
- `imu:` — `i_acc_freq`, `i_gyro_freq`, `i_enable_rotation`,
  `i_batch_report_threshold`, `i_max_batch_reports`
- `rgb:`, `left:`, `right:`, `stereo:`, `pointcloud:` — sensor-specific
  settings

If a parameter is rejected by the driver, run with `--ros-args -p`
verbose logging and grep the Pi's launch terminal for
"Param ... not declared".

---

## 6. Does this build a map?

What each launch publishes today:

| Launch file | Image | Pointcloud | LaserScan | SLAM map | Costmap |
|---|---|---|---|---|---|
| `oak_d_camera.launch.py`           | ✓ | ✓ | — | — | — |
| `oak_d_rviz.launch.py`             | ✓ | ✓ | — | — | — |
| `camera_mapping_test.launch.py`    | ✓ | ✓ | ✓ | — | — |
| `hardware_bringup.launch.py`       | ✓ | ✓ | — | — | — |
| `slam_bringup_pi.launch.py`        | ✓ | ✓ | ✓ | ✓ | — |
| `autonomy_bringup_pi.launch.py`    | ✓ | ✓ | — | — | ✓ (Nav2 obstacle layer) |

In `autonomy_bringup_pi`, the `map` frame is published by
`apriltag_localizer_node` — it's the *arena coordinate frame* from tag
detections, not a SLAM-built occupancy grid. Nav2's costmap is populated *in*
that frame from the pointcloud, but it doesn't retain a long-term map.

### Running SLAM

`slam_bringup_pi.launch.py` wires up `slam_toolbox` (online async) on top of
`hardware_bringup`. It does NOT load AprilTag localization — slam_toolbox and
`apriltag_localizer_node` both want to publish `map → odom` and cannot
coexist. Use this for exploration / pre-match map capture; switch back to
`autonomy_bringup_pi` for tag-localized runs.

```bash
# Pi: install once, then launch
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
ros2 launch lunabot_drive slam_bringup_pi.launch.py

# PC: watch the map build live
ros2 launch lunabot_drive slam_view_pc.launch.py

# Save the map (from any terminal on the same domain)
ros2 run nav2_map_server map_saver_cli -f ~/maps/arena
```

Drive the robot slowly with the controller while mapping; `slam_toolbox`
needs ~10 cm of travel between scans (`minimum_travel_distance`) to add a
submap. Loop closure runs continuously.

Config knobs in `config/params/slam_toolbox_params.yaml`:

| Param | Default | Effect |
|---|---|---|
| `resolution` | 0.05 | map cell size (matches Nav2 costmap) |
| `max_laser_range` | 5.0 | match `pointcloud_to_laserscan range_max` |
| `minimum_travel_distance` | 0.1 | scan cadence — lower = more submaps, more CPU |
| `loop_search_space_dimension` | 8.0 | matches arena diagonal |

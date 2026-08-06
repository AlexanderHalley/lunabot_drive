# OAK-D S2 integration

The rover's only sensor: stereo depth, RGB, and a BNO086 IMU, over USB-C.

> **Ported from the 2026 doc, with the topic names corrected.** The old version documented
> `/camera/image_raw`, `/camera/depth/points` and `/camera/camera_info`. The launch files published
> `/oak_d/rgb/image_raw`, `/oak_d/points` and `/oak_d/stereo/camera_info`. Nobody noticed until
> someone tried the documented names. [`TOPIC_FRAME_CONTRACT.md`](TOPIC_FRAME_CONTRACT.md) is now
> the authority and this file defers to it.

---

## Profiles

The camera is not configured once. Different consumers need genuinely different device setups, so
`camera.launch.py` takes a `profile` argument:

| `profile:=` | Publishes | For |
|---|---|---|
| `default` | RGB + depth **aligned to RGB** + IMU | rtabmap, perception. The normal one |
| `stereo_rect` | Rectified left/right pair + IMU, no depth | cuVSLAM |
| `pointcloud` | Depth + IMU, no RGB | Bandwidth-starved links, perception only |
| `rgb_only` | RGB + IMU at 1080p30 | Calibration, "is the camera alive" |

```bash
ros2 launch lunabot_bringup camera.launch.py profile:=default
```

`stereo_rect` exists because cuVSLAM consumes a rectified stereo pair rather than RGB+depth. That
is a different device configuration, not a remapping — which is why `slam.launch.py` selects the
profile rather than assuming one.

---

## What changed from 2026, and why

**`i_align_depth: true`.** rtabmap needs depth registered into the RGB camera's frame. Unaligned
depth does not produce an error; it produces geometry that is subtly and consistently wrong, which
reads as a SLAM tuning problem for about a day.

**`i_publish_tf_from_calibration: false`.** `robot_state_publisher` owns the TF tree, built from
the URDF. Left at the default, the driver publishes its own camera transforms and fights it — two
publishers on the same transforms, nondeterministic result. This replaces the four
`static_transform_publisher` calls that used to live in `oak_d_rviz.launch.py`.

**The frame prefix is the node's name, `oak_d`, and not a parameter.** With the line above false,
`sensor_helpers.cpp::tfPrefix()` returns `node->get_name()`. `camera.launch.py` sets it via
`DRIVER_NODE_NAME`.

> **RESOLVED** against `depthai-ros` 2.12.2, the version Jazzy ships. This used to also list
> `i_tf_tf_prefix: oak_d`, which depthai-ros does not declare — exactly the case this note warned
> about, where a silently-ignored parameter looks like a correctly-applied one. It was ignored for
> however long it sat there, and the frames came out right for an unrelated reason.

**Point cloud generation moved into the launch file's arguments** rather than being hardcoded.
Still generated on the robot, still decimation 4 and 2 m clip.

---

## The tuning, and what each number is protecting

Every non-default value in `oak_d_s2.yaml` was set against a Raspberry Pi 5's USB bandwidth or
memory. None of it is quality tuning:

| Setting | Value | Protecting against |
|---|---|---|
| `stereo.i_resolution` | `400P` | USB bandwidth |
| `stereo.i_fps` | `10.0` | USB bandwidth |
| `i_max_range` | `2000` mm | Bandwidth — the 7.5 cm baseline makes depth beyond ~2 m mostly noise anyway |
| `i_enable_lazy_publisher` | `true` | An idle camera saturating USB |
| `i_enable_decimation_filter` | `false` | Memory warnings on the Pi |
| `i_enable_temporal_filter` | `false` | Visible lag while teleoperating |
| `i_enable_speckle_filter` | `false` | Memory constraints on the S2 itself |
| `i_enable_spatial_filter` | `true` | **Kept on** — it is what makes the cloud clusterable rather than a noise field |
| `i_nn_type` | `none` | VPU time and bandwidth; detection runs on the host |

Each disabled filter was turned off for an observed failure, not on principle. Re-enable them one
at a time and watch for the specific symptom in the right-hand column.

---

## Depth encoding: the 1000× trap

| | Encoding | Units |
|---|---|---|
| OAK-D S2 via depthai | `16UC1` | **millimetres** |
| Isaac Sim depth annotator | `32FC1` | **metres** |

`rtabmap` accepts both, which is exactly what makes this dangerous — everything works until
something we wrote reads the depth image directly and is silently off by a factor of a thousand.

Two consequences, both deliberate:

1. `lunabot_perception/depth_normalizer_node` converts `16UC1` mm → `32FC1` m and runs **on the
   real robot only**. Everything downstream sees one encoding.
2. The boulder detector consumes `/oak_d/points`, not the depth image. Point clouds are metres in
   both cases, so they are directly comparable between sim and hardware.

---

## Bring-up

### Install

```bash
sudo apt install ros-jazzy-depthai-ros

# udev rule for non-root USB access
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Use a **USB 3.0** port. The S2 draws around 2.5 W; a powered hub helps if enumeration is flaky.

### Verify the device

```bash
lsusb | grep Movidius        # 03e7:2485 Intel Movidius MyriadX
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
```

### Verify the topics

```bash
ros2 launch lunabot_bringup camera.launch.py profile:=default

ros2 topic hz /oak_d/rgb/image_raw      # ~15 Hz
ros2 topic hz /oak_d/stereo/image_raw   # ~10 Hz
ros2 topic hz /oak_d/points             # ~10 Hz
ros2 topic hz /oak_d/imu/data           # ~400 Hz
```

Then check the frames actually connect to the robot, which is the part the 2026 setup got wrong:

```bash
ros2 run tf2_ros tf2_echo base_link oak_d_rgb_camera_optical_frame
```

If that fails, `robot_state_publisher` is not running or the driver's prefix does not match the
URDF.

---

## Network

Both machines:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

`~/.ros/cyclonedds.xml`:

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

The 10 MB receive buffer is not optional for image topics — the default drops frames under load in
a way that looks like a camera problem.

If the link is still saturated: `sudo apt install ros-jazzy-image-transport-plugins` for compressed
transport, then drop to `profile:=pointcloud` if RGB is not needed.

---

## Troubleshooting

**Topics visible on the robot, not on the laptop.** Same `ROS_DOMAIN_ID` on both? Then
`ros2 daemon stop && ros2 daemon start`.

**Point cloud empty or very sparse.** `depth_image_proc` is subscribed to
`/oak_d/stereo/image_raw`, and `i_enable_lazy_publisher: true` means the device only produces
frames when something is listening — so check the depth topic has a publisher *and* a subscriber.
Also check `max_range`: everything past 2 m is discarded by design.

**Point cloud in the wrong place, or rotated 90°.** The optical frame rotation. Check
`tf2_echo oak_d_rgb_camera_frame oak_d_rgb_camera_optical_frame` gives rpy `-1.571, 0, -1.571`.

**rtabmap produces warped geometry.** `i_align_depth` is not actually applied. Verify the parameter
name against your driver version rather than assuming the config took effect.

**Frames named `oak_d_...` exist twice, or jitter.** The driver is publishing its own TF. Confirm
`i_publish_tf_from_calibration: false` is being honoured — `ros2 param get /oak_d ...`.

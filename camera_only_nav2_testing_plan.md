# Camera-Only Nav2 Testing Plan

When motor hardware isn't available, the OAK-D S2 camera can still be used to validate significant portions of the navigation stack. This plan outlines what's testable, what isn't, and how to set up a minimal camera-only configuration.

---

## Goals

1. Validate depth camera → costmap pipeline
2. Verify TF transforms are correctly configured
3. Profile bandwidth usage at competition-realistic settings
4. Prototype vision-based behaviors (beacon detection, obstacle classification)

---

## What Transfers to Full System

| Component | Transferable? | Notes |
|-----------|---------------|-------|
| Depth → PointCloud2 → local costmap | ✅ Yes | Core obstacle detection pipeline |
| Camera TF transforms | ✅ Yes | `camera_link` → `camera_link_optical` → `base_link` |
| Bandwidth profiling | ✅ Yes | Critical for 4 Mbps competition limit |
| DWB local planner obstacle response | ✅ Yes | Local avoidance behavior |
| Image processing nodes | ✅ Yes | Beacon detection, edge detection prototypes |
| AMCL localization | ❌ No | Requires LIDAR |
| Global path planning | ❌ No | Requires valid map + localization |
| Odometry / position tracking | ❌ No | Camera IMU alone can't provide translation |
| Full autonomous navigation | ❌ No | Needs wheel encoders + LIDAR |

---

## Hardware Setup

### Requirements
- Raspberry Pi 5 connected to OAK-D S2 via USB-C
- Pi and offboard computer on same network
- Same `ROS_DOMAIN_ID` on both machines

### Connections
```
┌─────────────────────────────────────────┐
│            Raspberry Pi 5               │
│  ┌─────────────┐    ┌────────────────┐  │
│  │  OAK-D S2   │───▶│ depthai_ros    │  │
│  │  (USB-C)    │    │ /camera/*      │  │
│  └─────────────┘    └───────┬────────┘  │
└─────────────────────────────┼───────────┘
                              │ WiFi
                              ▼
┌─────────────────────────────────────────┐
│          Offboard Computer              │
│  ┌──────────┐  ┌──────────┐  ┌───────┐  │
│  │ Nav2     │  │ RViz2    │  │ Tests │  │
│  │ (local)  │  │          │  │       │  │
│  └──────────┘  └──────────┘  └───────┘  │
└─────────────────────────────────────────┘
```

---

## Phase 1: Camera Validation

### 1.1 Verify Camera Detection
```bash
# On Pi
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
# Expected: [MyriadX device info]
```

### 1.2 Launch Camera Driver
```bash
# On Pi
ros2 launch lunabot_drive oak_d_camera.launch.py
```

### 1.3 Verify Topics (on offboard computer)
```bash
# Check topics are visible across network
ros2 topic list | grep camera

# Expected topics:
# /camera/image_raw
# /camera/camera_info
# /camera/depth/image_raw
# /camera/depth/points
```

### 1.4 Verify Data Flow
```bash
# Check framerate
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/points

# Quick visualization
ros2 run rqt_image_view rqt_image_view
```

---

## Phase 2: TF Transform Validation

### 2.1 Create Static Transform Publisher

Since there's no robot_state_publisher running without the full URDF, publish a minimal TF tree:

```bash
# Static transform: map → odom (identity)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

# Static transform: odom → base_link (identity, simulates stationary robot)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# Static transform: base_link → camera_link (adjust to match your mount)
ros2 run tf2_ros static_transform_publisher 0.5 0 0.1 0 0 0 base_link camera_link

# Static transform: camera_link → camera_link_optical (standard ROS convention)
# Z forward, X right, Y down for optical frame
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_link_optical
```

### 2.2 Verify TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens PDF showing transform tree

ros2 run tf2_ros tf2_echo map camera_link_optical
```

### 2.3 Visualize in RViz
1. Open RViz2
2. Set Fixed Frame to `map`
3. Add TF display — verify all frames appear
4. Add PointCloud2 display → `/camera/depth/points`
5. Verify point cloud appears in correct position relative to base_link

---

## Phase 3: Local Costmap Integration

### 3.1 Create Minimal Nav2 Config

Create `config/camera_only_nav2.yaml`:

```yaml
# Minimal Nav2 config for camera-only testing
# Only runs local costmap + controller - no AMCL, no global planner

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      
      plugins: ["obstacle_layer", "inflation_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: camera
        camera:
          topic: /camera/depth/points
          data_type: "PointCloud2"
          marking: True
          clearing: True
          min_obstacle_height: 0.1
          max_obstacle_height: 2.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.1
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    
    # DWB controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      max_vel_x: 0.26
      max_vel_theta: 1.0
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
```

### 3.2 Create Camera-Only Launch File

Create `launch/camera_only_nav2.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')
    
    # Use camera-only config
    nav2_params = os.path.join(pkg_share, 'config', 'camera_only_nav2.yaml')
    
    return LaunchDescription([
        # Static TF: map → odom → base_link (stationary robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),
        # Camera mount transform (adjust for your setup)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.5', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        ),
        # Optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
        ),
        
        # Local costmap only
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
```

### 3.3 Run and Verify

```bash
# Terminal 1 (Pi): Launch camera
ros2 launch lunabot_drive oak_d_camera.launch.py

# Terminal 2 (Offboard): Launch camera-only nav2
ros2 launch lunabot_drive camera_only_nav2.launch.py

# Terminal 3 (Offboard): RViz
rviz2
```

**In RViz, add:**
- Map display → `/local_costmap/costmap`
- PointCloud2 → `/camera/depth/points`
- TF display

**Verify:**
- [ ] Obstacles appear in costmap when objects are in front of camera
- [ ] Costmap clears when obstacles are removed
- [ ] No TF errors in terminal output

---

## Phase 4: Bandwidth Profiling

### 4.1 Measure Current Bandwidth

```bash
# Install bandwidth monitoring tool
sudo apt install ifstat

# Monitor network interface (replace wlan0 with your interface)
ifstat -i wlan0 1

# Or use ROS2 topic bandwidth
ros2 topic bw /camera/image_raw
ros2 topic bw /camera/depth/points
ros2 topic bw /camera/depth/image_raw
```

### 4.2 Test Different Configurations

| Configuration | Resolution | FPS | Expected Bandwidth |
|--------------|------------|-----|-------------------|
| High quality | 1080P | 15 | ~25 Mbps (too high) |
| Medium | 720P | 15 | ~12 Mbps (too high) |
| Competition | 720P | 10 | ~8 Mbps (borderline) |
| Conservative | 480P | 10 | ~3 Mbps (safe) |
| Compressed | 720P + JPEG | 10 | ~2 Mbps (recommended) |

### 4.3 Enable Compressed Transport

```bash
# Install compression plugins
sudo apt install ros-jazzy-image-transport-plugins

# Compressed topics auto-appear:
# /camera/image_raw/compressed
# /camera/depth/image_raw/compressedDepth
```

Update `oak_d_camera.yaml` for competition:

```yaml
rgb:
  i_resolution: '720P'
  i_fps: 10

stereo:
  i_resolution: '400P'  # Lower depth resolution
  i_fps: 10

pointcloud:
  i_enable: false  # Compute on offboard instead
```

### 4.4 Document Results

Create bandwidth test log:

```bash
# Save bandwidth measurements
ros2 topic bw /camera/image_raw/compressed > bandwidth_log.txt
```

---

## Phase 5: Vision Behavior Prototypes

### 5.1 Beacon Detection (if applicable)

Test detecting competition beacons using the RGB stream:

```python
# Simple color-based beacon detector
# beacon_detector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BeaconDetector(Node):
    def __init__(self):
        super().__init__('beacon_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/beacon/debug_image', 10)
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Example: detect bright green beacon
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (35, 100, 100), (85, 255, 255))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.get_logger().info(f'Beacon at pixel ({x + w//2}, {y + h//2})')
        
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.pub.publish(debug_msg)
```

### 5.2 Depth-Based Obstacle Classification

```python
# Classify obstacles by size using depth data
# obstacle_classifier.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ObstacleClassifier(Node):
    def __init__(self):
        super().__init__('obstacle_classifier')
        self.sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pc_callback, 10)
    
    def pc_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'))))
        
        # Filter points in front of robot (0.5m to 2m range)
        mask = (points[:, 2] > 0.5) & (points[:, 2] < 2.0)
        nearby = points[mask]
        
        if len(nearby) > 100:
            # Estimate obstacle height
            height = np.max(nearby[:, 1]) - np.min(nearby[:, 1])
            width = np.max(nearby[:, 0]) - np.min(nearby[:, 0])
            
            if height > 0.3:
                self.get_logger().info(f'Large obstacle: {width:.2f}m x {height:.2f}m')
            else:
                self.get_logger().info(f'Small obstacle: {width:.2f}m x {height:.2f}m')
```

---

## Testing Checklist

### Phase 1: Camera Basics
- [ ] Camera detected on Pi
- [ ] depthai_ros_driver launches without errors
- [ ] Topics visible from offboard computer
- [ ] Image appears in rqt_image_view
- [ ] Depth points visible in RViz

### Phase 2: Transforms
- [ ] TF tree shows map → odom → base_link → camera_link → camera_link_optical
- [ ] No TF errors in terminal
- [ ] Point cloud appears in correct position in RViz

### Phase 3: Costmap
- [ ] Local costmap publishes
- [ ] Obstacles appear when objects placed in front of camera
- [ ] Costmap clears when obstacles removed
- [ ] Inflation layer visible around obstacles

### Phase 4: Bandwidth
- [ ] Measured bandwidth at default settings
- [ ] Tested compressed transport
- [ ] Found configuration under 4 Mbps
- [ ] Documented recommended settings

### Phase 5: Vision
- [ ] Beacon detector runs (if applicable)
- [ ] Depth-based obstacle info logged
- [ ] No significant frame drops during processing

---

## Known Limitations

1. **No odometry** — Robot position stays at origin. Can't test path following.
2. **No AMCL** — No global localization. Robot doesn't know where it is on the map.
3. **No global planning** — Can't send navigation goals or test waypoint navigation.
4. **IMU not useful alone** — Camera IMU provides orientation but not position.

---

## Next Steps After Motor Integration

Once motors are reconnected:

1. Merge camera config with full `nav2_params.yaml`
2. Add depth camera as secondary obstacle source alongside LIDAR
3. Test sensor fusion: LIDAR for mapping, depth for close obstacles
4. Validate bandwidth stays under 4 Mbps with all sensors active
5. Re-run full navigation stack tests

---

## Files to Create

| File | Purpose |
|------|---------|
| `config/camera_only_nav2.yaml` | Minimal Nav2 params for testing |
| `launch/camera_only_nav2.launch.py` | Launch file with static TFs |
| `scripts/beacon_detector.py` | Vision behavior prototype |
| `scripts/obstacle_classifier.py` | Depth classification prototype |

---

## Commands Reference

```bash
# Start camera on Pi
ros2 launch lunabot_drive oak_d_camera.launch.py

# Start camera-only nav2 on offboard
ros2 launch lunabot_drive camera_only_nav2.launch.py

# Check bandwidth
ros2 topic bw /camera/image_raw

# View TF tree
ros2 run tf2_tools view_frames

# Monitor costmap
ros2 topic echo /local_costmap/costmap_raw --once
```

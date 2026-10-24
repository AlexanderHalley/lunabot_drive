#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Record a run as one video: what it looks like, and what the rover knows.

    run_isaac_sim.sh --demo-cameras          # Isaac publishes /sim/demo/*
    ros2 run lunabot_bringup demo_recorder.py --ros-args -p use_sim_time:=true \
        -p output:=$HOME/lunabot_demo.mp4

1920x1080, H.264, frames at `fps` of SIM time (so a slow sim does not make a
fast video). Ctrl-C finishes the file.

    +---------------------------------+-----------------+
    |                                 | arena overview  |
    |  chase camera (Isaac)           +-----------------+
    |                                 | the rover's map |
    +----------------+----------------+-----------------+
    | onboard camera | mission state, time, speed       |
    +----------------+----------------------------------+

The map panel draws what the ROVER knows -- the global costmap, its plan and
its own pose estimate -- not ground truth -- in the ARENA frame, through the
AprilTag start fix (TF arena -> map, fiducial_localizer.py) once there is one;
until then map is drawn as if it were arena. The zone outlines come from the
`zones` parameter (default: the UCF lane, arena frame), a viewer's guide only.

Missing inputs show as a grey "no signal" panel instead of stopping the
recording, so this runs on the robot too, with whatever is available.
"""

from __future__ import annotations

import math
import time

import numpy as np

W, H = 1920, 1080
CHASE = (0, 0, 1280, 720)  # x, y, w, h
OVERVIEW = (1280, 0, 640, 360)
MAP = (1280, 360, 640, 360)
ONBOARD = (0, 720, 576, 360)
TEXT = (576, 720, 1344, 360)

# UCF lane, map frame (lunabot_sim/scene/layout.py). name:x_min,x_max,y_min,y_max
DEFAULT_ZONES = (
    'arena:-0.75,7.39,-1.0,3.57;start:-0.75,1.25,-1.0,1.0;'
    'excavation:3.32,7.39,-1.0,3.57;construction:4.79,7.39,1.57,3.57;berm:5.34,6.84,2.12,3.02'
)
FOOTPRINT = (0.92, 0.644)  # nav2_params.yaml footprint; for drawing only


def parse_zones(text: str) -> dict[str, tuple[float, float, float, float]]:
    zones = {}
    for item in filter(None, (s.strip() for s in text.split(';'))):
        name, values = item.split(':')
        zones[name.strip()] = tuple(float(v) for v in values.split(','))
    return zones


def costmap_image(data: np.ndarray) -> np.ndarray:
    """Nav2 costmap values (0-255, int8 view or uint8) -> BGR image, row 0 = lowest y."""
    cost = data.astype(np.uint8)
    img = np.empty(cost.shape + (3,), np.uint8)
    img[:] = (38, 34, 30)  # free (seen or not): dark
    inflated = (cost > 0) & (cost < 253)
    t = cost[inflated].astype(np.float32) / 252.0
    img[inflated] = np.stack([40 + 40 * t, 60 + 60 * t, 90 + 150 * t], axis=-1).astype(np.uint8)
    img[(cost == 253) | (cost == 254)] = (40, 40, 230)  # lethal / inscribed: red
    img[cost == 255] = (70, 70, 70)  # unknown
    return img


class MapView:
    """World rectangle -> panel pixels, +x right, +y up, aspect kept."""

    def __init__(self, bounds, size, margin=0.3):
        x0, x1, y0, y1 = bounds
        self.x0, self.y1 = x0 - margin, y1 + margin
        w, h = size
        self.scale = min(w / (x1 - x0 + 2 * margin), h / (y1 - y0 + 2 * margin))
        self.ox = (w - (x1 - x0 + 2 * margin) * self.scale) / 2.0
        self.oy = (h - (y1 - y0 + 2 * margin) * self.scale) / 2.0

    def px(self, x, y):
        return (
            int(round(self.ox + (x - self.x0) * self.scale)),
            int(round(self.oy + (self.y1 - y) * self.scale)),
        )


def main() -> None:
    import cv2
    import rclpy
    from nav_msgs.msg import Path
    from rclpy.executors import ExternalShutdownException
    from nav2_msgs.srv import GetCostmap
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('demo_recorder')
    output = node.declare_parameter('output', 'lunabot_demo.mp4').value
    fps = float(node.declare_parameter('fps', 15.0).value)
    title = node.declare_parameter('title', 'Lunabot 2027 autonomy - Isaac Sim').value
    zones = parse_zones(node.declare_parameter('zones', DEFAULT_ZONES).value)
    costmap_service = node.declare_parameter('costmap_service', '/global_costmap/get_costmap').value
    log = node.get_logger()

    latest = {}
    state = {'mission': '', 'plan': [], 'trail': [], 'costmap': None, 'pose': None, 'speed': 0.0,
             'fix': (0.0, 0.0, 0.0), 'fix_label': 'no start fix yet: map = arena', 'fixed': False}

    def keep(key):
        def cb(msg):
            latest[key] = msg

        return cb

    for key, topic in (
        ('chase', '/sim/demo/chase/image_raw'),
        ('overview', '/sim/demo/overview/image_raw'),
        ('onboard', '/oak_d/rgb/image_rect'),
    ):
        node.create_subscription(Image, topic, keep(key), qos_profile_sensor_data)
    latched = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
    )
    node.create_subscription(
        String, '/mission/state', lambda m: state.__setitem__('mission', m.data), latched
    )
    node.create_subscription(
        Path,
        '/plan',
        lambda m: state.__setitem__(
            'plan', [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        ),
        10,
    )
    tf = Buffer()
    TransformListener(tf, node)
    costmap = node.create_client(GetCostmap, costmap_service)
    pending = {'future': None}

    def poll_costmap():
        if pending['future'] is not None and not pending['future'].done():
            return
        if pending['future'] is not None and pending['future'].result() is not None:
            m = pending['future'].result().map
            data = np.frombuffer(bytes(m.data), np.uint8).reshape(
                m.metadata.size_y, m.metadata.size_x
            )
            state['costmap'] = (
                data,
                m.metadata.resolution,
                m.metadata.origin.position.x,
                m.metadata.origin.position.y,
            )
        if costmap.service_is_ready():
            pending['future'] = costmap.call_async(GetCostmap.Request())

    node.create_timer(1.0, poll_costmap)

    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    writer = cv2.VideoWriter(output, fourcc, fps, (W, H))
    if not writer.isOpened():
        raise SystemExit(f'cannot open {output} for writing')
    view = MapView(zones.get('arena', (-1.0, 8.0, -1.5, 4.0)), MAP[2:])
    font = cv2.FONT_HERSHEY_SIMPLEX
    frames = {'n': 0, 't0': None, 'last_pose': None}

    def to_bgr(msg, size):
        if msg is None:
            return None
        channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        code = {
            'rgb8': cv2.COLOR_RGB2BGR,
            'rgba8': cv2.COLOR_RGBA2BGR,
            'bgra8': cv2.COLOR_BGRA2BGR,
        }.get(msg.encoding)
        img = cv2.cvtColor(img, code) if code is not None else img[:, :, :3]
        return cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    def paste(canvas, img, rect, label):
        x, y, w, h = rect
        if img is None:
            canvas[y : y + h, x : x + w] = (48, 48, 48)
            cv2.putText(canvas, 'no signal', (x + w // 2 - 70, y + h // 2), font, 0.9,
                        (150, 150, 150), 2, cv2.LINE_AA)
        else:
            canvas[y : y + h, x : x + w] = img
        cv2.rectangle(canvas, (x, y), (x + 8 + 11 * len(label), y + 26), (0, 0, 0), -1)
        cv2.putText(canvas, label, (x + 5, y + 19), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def arena_xy(x, y):
        fx, fy, fyaw = state['fix']
        c, sn = math.cos(fyaw), math.sin(fyaw)
        return fx + c * x - sn * y, fy + sn * x + c * y

    def draw_map():
        w, h = MAP[2:]
        if state['costmap'] is not None:
            # costmap cell (col i, row j; row 0 = lowest y) -> map -> arena -> panel,
            # as one affine warp, so a rotated start fix rotates the costmap too.
            data, res, ox, oy = state['costmap']
            fx, fy, fyaw = state['fix']
            c, sn = math.cos(fyaw), math.sin(fyaw)
            k = view.scale * res
            warp = np.array([
                [k * c, -k * sn,
                 view.scale * (c * ox - sn * oy + fx - view.x0) + view.ox],
                [-k * sn, -k * c,
                 view.scale * (view.y1 - (sn * ox + c * oy + fy)) + view.oy],
            ])
            panel = cv2.warpAffine(costmap_image(data), warp, (w, h), flags=cv2.INTER_NEAREST,
                                   borderMode=cv2.BORDER_CONSTANT, borderValue=(30, 30, 30))
        else:
            panel = np.full((h, w, 3), 30, np.uint8)
        colours = {'arena': (220, 220, 220), 'excavation': (0, 200, 255),
                   'construction': (255, 170, 60), 'berm': (255, 120, 255), 'start': (120, 255, 120)}
        for name, (x0, x1, y0, y1) in zones.items():
            c = colours.get(name, (200, 200, 200))
            cv2.rectangle(panel, view.px(x0, y1), view.px(x1, y0), c, 1, cv2.LINE_AA)
            if name != 'arena':
                px, py = view.px(x0, y1)
                cv2.putText(panel, name, (px + 3, py + 13), font, 0.4, c, 1, cv2.LINE_AA)
        if state['fix_label']:
            cv2.putText(panel, state['fix_label'], (8, h - 10), font, 0.45, (200, 200, 200), 1,
                        cv2.LINE_AA)
        if len(state['trail']) > 1:
            pts = np.array([view.px(*arena_xy(x, y)) for x, y in state['trail']], np.int32)
            cv2.polylines(panel, [pts], False, (180, 180, 60), 1, cv2.LINE_AA)
        if len(state['plan']) > 1:
            pts = np.array([view.px(*arena_xy(x, y)) for x, y in state['plan']], np.int32)
            cv2.polylines(panel, [pts], False, (60, 255, 60), 2, cv2.LINE_AA)
        if state['pose'] is not None:
            x, y, yaw = state['pose']
            x, y = arena_xy(x, y)
            yaw += state['fix'][2]
            c, s = math.cos(yaw), math.sin(yaw)
            hl, hw = FOOTPRINT[0] / 2, FOOTPRINT[1] / 2
            corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
            pts = np.array([view.px(x + c * a - s * b, y + s * a + c * b) for a, b in corners],
                           np.int32)
            cv2.polylines(panel, [pts], True, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.line(panel, view.px(x, y), view.px(x + hl * c, y + hl * s), (255, 255, 255), 2)
        return panel

    def update_fix():
        if state['fixed']:
            return
        try:
            t = tf.lookup_transform('arena', 'map', rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        state['fix'] = (t.transform.translation.x, t.transform.translation.y, yaw)
        state['fixed'] = True
        state['fix_label'] = (f'AprilTag start fix: x {state["fix"][0]:.2f} '
                              f'y {state["fix"][1]:.2f} yaw {math.degrees(yaw):.0f} deg')

    def update_pose():
        update_fix()
        try:
            t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = t.transform.translation.x, t.transform.translation.y
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        last = frames['last_pose']
        if last is not None and stamp > last[2]:
            speed = math.hypot(x - last[0], y - last[1]) / (stamp - last[2])
            state['speed'] = 0.7 * state['speed'] + 0.3 * speed
        frames['last_pose'] = (x, y, stamp)
        state['pose'] = (x, y, yaw)
        if not state['trail'] or math.hypot(x - state['trail'][-1][0],
                                            y - state['trail'][-1][1]) > 0.03:
            state['trail'].append((x, y))

    def frame():
        now = node.get_clock().now().nanoseconds * 1e-9
        if frames['t0'] is None:
            if 'chase' not in latest and 'onboard' not in latest:
                return  # nothing to show yet: do not record a black lead-in
            frames['t0'] = now
            log.info(f'recording to {output}')
        update_pose()
        canvas = np.zeros((H, W, 3), np.uint8)
        paste(canvas, to_bgr(latest.get('chase'), CHASE[2:]), CHASE, 'chase view (sim only)')
        paste(canvas, to_bgr(latest.get('overview'), OVERVIEW[2:]), OVERVIEW, 'arena')
        paste(canvas, draw_map(), MAP, "rover's map: costmap, plan, pose")
        paste(canvas, to_bgr(latest.get('onboard'), ONBOARD[2:]), ONBOARD, 'onboard camera')
        x, y, w, h = TEXT
        canvas[y : y + h, x : x + w] = (22, 22, 22)
        elapsed = now - frames['t0']
        lines = [
            (title, 0.8, (200, 200, 200)),
            (state['mission'] or '-', 1.3, (80, 220, 255)),
            (f'mission clock {int(elapsed // 60):02d}:{int(elapsed % 60):02d}    '
             f'speed {state["speed"]:.2f} m/s', 0.8, (230, 230, 230)),
            ('fully autonomous: no operator input', 0.7, (140, 200, 140)),
        ]
        ty = y + 60
        for text, scale, colour in lines:
            cv2.putText(canvas, text, (x + 30, ty), font, scale, colour, 2, cv2.LINE_AA)
            ty += int(70 * max(scale, 0.8))
        writer.write(canvas)
        frames['n'] += 1

    node.create_timer(1.0 / fps, frame)
    wall0 = time.monotonic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or SIGINT from ros2 run: both mean "finish the file"
    finally:
        writer.release()
        log.info(f'wrote {frames["n"]} frames ({frames["n"] / fps:.0f} s of video) to {output} '
                 f'in {time.monotonic() - wall0:.0f} s')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

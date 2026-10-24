#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Find where the rover started, from the AprilTags on the start corner's walls.

The competition draws the start pose (position in the start zone, and heading)
at random. SLAM's `map` frame starts wherever the rover is, so on its own it
knows nothing about where the zones are. Two tag36h11 tags, one on each wall
of the start corner, at known positions in the `arena` frame
(config/fiducials_ucf.yaml), fix that.

While the start survey turns the rover a full circle, every camera frame is
searched for the tags. Each detection gives the tag's pose in the camera
(solvePnP on its four corners, knowing its size), and TF at the image's stamp
gives the camera in `map`, so each detection puts the tag in `map`. When
/navigation/survey_done arrives:

  - both tags seen: fit arena -> map in 2D from the two tag CENTRES (mean of
    each tag's detections). Position from PnP is good; a small tag's
    orientation is not, and 1 m between the centres gives the heading far
    better than either tag's own normal does.
  - one tag seen: median of that tag's full-pose estimates. Worse heading.
  - none: nothing is published, and an error says so.

The result is a STATIC transform arena -> map (map's parent), and a latched
/localization/arena_fix with the map origin's pose in the arena frame, the
covariance from the spread of the detections, and in the frame id the method
used. Nothing downstream needs to know about tags: mission.py asks TF for
arena -> map.

It fixes the start only. SLAM keeps map consistent from there; if map drifts
from the arena over a run, that is SLAM's problem, and re-seeing the tags on
the way home could correct it later (not done).

Pure functions above main() are tested without ROS (test/test_fiducial_localizer.py).
"""

from __future__ import annotations

import math

import numpy as np

FIX_TOPIC = '/localization/arena_fix'
SURVEY_TOPIC = '/navigation/survey_done'

# Detection quality gates. A tag smaller than this in the image gives a poor
# PnP pose; one cut by the image edge gives a wrong one.
MIN_SIDE_PX = 20.0
MAX_REPROJECTION_PX = 2.0
MAX_RANGE_M = 4.0
EDGE_MARGIN_PX = 4.0


def tag_in_arena(x: float, y: float, z: float, facing: float) -> np.ndarray:
    """4x4 pose of a tag in the arena frame (OpenCV marker axes: +Z out, +Y up)."""
    c, s = math.cos(facing), math.sin(facing)
    t = np.eye(4)
    t[:3, 0] = (-s, c, 0.0)  # +X: the viewer's right
    t[:3, 1] = (0.0, 0.0, 1.0)  # +Y: up
    t[:3, 2] = (c, s, 0.0)  # +Z: out of the wall
    t[:3, 3] = (x, y, z)
    return t


def planar(t: np.ndarray) -> tuple[float, float, float]:
    """(x, y, yaw) of a 4x4 transform between two z-up frames."""
    return float(t[0, 3]), float(t[1, 3]), math.atan2(t[1, 0], t[0, 0])


def planar_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    t = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    t[:2, :2] = ((c, -s), (s, c))
    t[:2, 3] = (x, y)
    return t


def detect(gray, k, d, size: float, ids, cv2=None):
    """Tags in one image: [(id, T_camera_tag 4x4, reprojection px, side px)].

    `ids` limits which tags count: a wrong id is a wrong fix.
    """
    if cv2 is None:
        import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners, found, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if found is None:
        return []
    h = size / 2.0
    # OpenCV's corner order: top-left, top-right, bottom-right, bottom-left.
    model = np.array([(-h, h, 0.0), (h, h, 0.0), (h, -h, 0.0), (-h, -h, 0.0)])
    height, width = gray.shape[:2]
    out = []
    for image_corners, tag_id in zip(corners, found.ravel(), strict=True):
        if int(tag_id) not in ids:
            continue
        pts = image_corners.reshape(4, 2).astype(np.float64)
        side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
        if side < MIN_SIDE_PX:
            continue
        if (
            pts[:, 0].min() < EDGE_MARGIN_PX
            or pts[:, 1].min() < EDGE_MARGIN_PX
            or pts[:, 0].max() > width - 1 - EDGE_MARGIN_PX
            or pts[:, 1].max() > height - 1 - EDGE_MARGIN_PX
        ):
            continue
        ok, rvec, tvec = cv2.solvePnP(model, pts, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        projected, _ = cv2.projectPoints(model, rvec, tvec, k, d)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pts) ** 2, axis=1))))
        if error > MAX_REPROJECTION_PX or float(np.linalg.norm(tvec)) > MAX_RANGE_M:
            continue
        t = np.eye(4)
        t[:3, :3] = cv2.Rodrigues(rvec)[0]
        t[:3, 3] = tvec.ravel()
        out.append((int(tag_id), t, error, side))
    return out


def solve(observations, tags):
    """Arena -> map from tag sightings in map.

    observations: [(id, T_map_tag 4x4)]; tags: {id: T_arena_tag 4x4}.
    Returns (x, y, yaw, method, spread) with spread = (sd_x, sd_y, sd_yaw), or
    None with no observations.
    """
    if not observations:
        return None
    # Per sighting: the full transform it implies. Its spread is the
    # uncertainty reported, whichever method gives the answer.
    each = np.array([planar(tags[i] @ np.linalg.inv(t)) for i, t in observations])
    yaws = each[:, 2]
    mean_yaw = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())
    yaw_dev = np.array([math.remainder(a - mean_yaw, math.tau) for a in yaws])
    spread = (float(each[:, 0].std()), float(each[:, 1].std()), float(yaw_dev.std()))

    seen = sorted({i for i, _ in observations})
    if len(seen) >= 2:
        # Rigid 2D fit (Kabsch) of tag centres: map -> arena.
        src = np.array(
            [np.mean([t[:2, 3] for j, t in observations if j == i], axis=0) for i in seen]
        )
        dst = np.array([tags[i][:2, 3] for i in seen])
        src_c, dst_c = src - src.mean(axis=0), dst - dst.mean(axis=0)
        u, _, vt = np.linalg.svd(src_c.T @ dst_c)
        r = (u @ vt).T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = (u @ vt).T
        yaw = math.atan2(r[1, 0], r[0, 0])
        x, y = dst.mean(axis=0) - r @ src.mean(axis=0)
        return float(x), float(y), yaw, 'tag_centres', spread
    yaw = mean_yaw + float(np.median(yaw_dev))
    return float(np.median(each[:, 0])), float(np.median(each[:, 1])), yaw, 'single_tag', spread


def main() -> None:
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

    rclpy.init()
    node = Node('fiducial_localizer')
    log = node.get_logger()
    ids = [int(i) for i in node.declare_parameter('tag_ids', [0, 1]).value]
    flat = node.declare_parameter('tag_poses', [0.0] * 8).value
    size = float(node.declare_parameter('tag_size', 0.16).value)
    arena_frame = node.declare_parameter('arena_frame', 'arena').value
    map_frame = node.declare_parameter('map_frame', 'map').value
    max_rate = float(node.declare_parameter('max_rate', 10.0).value)
    if len(flat) != 4 * len(ids):
        raise SystemExit('tag_poses needs x, y, z, facing per entry of tag_ids')
    tags = {i: tag_in_arena(*flat[4 * n : 4 * n + 4]) for n, i in enumerate(ids)}

    tf = Buffer()
    TransformListener(tf, node)
    static = StaticTransformBroadcaster(node)
    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    fix_pub = node.create_publisher(PoseWithCovarianceStamped, FIX_TOPIC, latched)
    state = {
        'info': None,
        'last': -math.inf,
        'observations': [],
        'done': False,
        'frames': 0,
        'pending': [],
        'dropped': 0,
    }

    def on_info(msg):
        state['info'] = (np.array(msg.k, dtype=float).reshape(3, 3), np.array(msg.d, dtype=float))

    def on_image(msg):
        if state['done'] or state['info'] is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        now = stamp.nanoseconds * 1e-9
        if now - state['last'] < 1.0 / max_rate:
            return
        state['last'] = now
        channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
        if msg.encoding in ('mono8', '8UC1'):
            gray = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
            gray = gray[:, : msg.width]
        else:
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
            img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
            gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2GRAY)
        state['frames'] += 1
        found = detect(gray, *state['info'], size, ids, cv2=cv2)
        if found:
            # The image usually arrives before TF has the camera's pose at its
            # stamp, and blocking here would stall this executor. Resolve later.
            state['pending'].append((stamp, msg.header.frame_id, found))

    def resolve(final=False):
        """Put queued sightings into map, once TF covers their stamps."""
        keep = []
        for stamp, frame, found in state['pending']:
            try:
                cam = tf.lookup_transform(map_frame, frame, stamp)
            except Exception as exc:  # noqa: BLE001
                age = (node.get_clock().now() - stamp).nanoseconds * 1e-9
                if not final and age < 2.0:
                    keep.append((stamp, frame, found))
                else:
                    state['dropped'] += 1
                    log.warning(
                        f'dropped a sighting: no {map_frame} -> camera at its stamp ({exc})',
                        throttle_duration_sec=5.0,
                    )
                continue
            q, p = cam.transform.rotation, cam.transform.translation
            t_map_cam = np.eye(4)
            t_map_cam[:3, :3] = _quat_matrix(q.x, q.y, q.z, q.w)
            t_map_cam[:3, 3] = (p.x, p.y, p.z)
            for tag_id, t_cam_tag, error, side in found:
                state['observations'].append((tag_id, t_map_cam @ t_cam_tag))
                log.info(
                    f'tag {tag_id}: {side:.0f} px, reprojection {error:.2f} px, '
                    f'{np.linalg.norm(t_cam_tag[:3, 3]):.2f} m away',
                    throttle_duration_sec=1.0,
                )
        state['pending'] = keep

    def on_survey(msg):
        # Stop looking, then give TF a second to cover the last sightings.
        if not state['done']:
            state['done'] = True
            state['timer'] = node.create_timer(1.0, finish)

    def finish():
        node.destroy_timer(state['timer'])
        resolve(final=True)
        obs = state['observations']
        counts = {i: sum(1 for j, _ in obs if j == i) for i in ids}
        result = solve(obs, tags)
        if result is None:
            log.error(
                f'NO START FIX: no tag seen in {state["frames"]} frames of the survey '
                f'(looking for ids {ids}). arena -> map is not published.'
            )
            return
        x, y, yaw, method, (sx, sy, syaw) = result
        log.info(
            f'start fix ({method}, sightings {counts}, {state["frames"]} frames, '
            f'{state["dropped"]} dropped): map origin at arena '
            f'x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg; spread '
            f'{sx:.3f} / {sy:.3f} m, {math.degrees(syaw):.1f} deg'
        )
        now = node.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = arena_frame
        t.child_frame_id = map_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        static.sendTransform(t)
        fix = PoseWithCovarianceStamped()
        fix.header.stamp = now
        fix.header.frame_id = arena_frame
        fix.pose.pose.position.x, fix.pose.pose.position.y = x, y
        fix.pose.pose.orientation.z = math.sin(yaw / 2)
        fix.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = max(sx, 0.01) ** 2, max(sy, 0.01) ** 2, max(syaw, 0.005) ** 2
        fix.pose.covariance = cov
        fix_pub.publish(fix)

    node.create_subscription(
        CameraInfo,
        node.declare_parameter('camera_info_topic', '/oak_d/rgb/camera_info').value,
        on_info,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Image,
        node.declare_parameter('image_topic', '/oak_d/rgb/image_rect').value,
        on_image,
        qos_profile_sensor_data,
    )
    node.create_subscription(Bool, SURVEY_TOPIC, on_survey, latched)
    node.create_timer(0.25, resolve)
    try:
        rclpy.spin(node)  # stays up: the static transform's publisher
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _quat_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


if __name__ == '__main__':
    main()

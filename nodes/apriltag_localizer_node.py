#!/usr/bin/env python3
"""
AprilTag Localizer Node — Provides map->odom transform from AprilTag detections.

Uses apriltag_ros (which publishes tag poses as TF frames) to compute the
robot's absolute pose in the map frame.  The resulting map->odom transform
is broadcast continuously so Nav2 can plan in the global map frame.

Approach
--------
1. apriltag_ros detects tags and publishes TF: camera_frame -> tag36h11:N
2. We know the tag's ground-truth pose in the map frame (from parameters).
3. From those two pieces we derive T_map_base_link.
4. We also look up T_odom_base_link (from wheel-odom / EKF).
5. T_map_odom = T_map_base_link * T_base_link_odom

Publishes:
  /tf                 — map->odom transform (TransformBroadcaster, 10 Hz)
  /localization_status (std_msgs/String) — UNLOCALIZED / LOCALIZED / STALE

Subscribes:
  /detections (apriltag_msgs/AprilTagDetectionArray) — detection events (for timing)
  /tf, /tf_static — tag pose frames from apriltag_ros

Parameters
----------
  tag_ids:            [int]   — list of tag IDs to use (default [0, 1])
  tag_family:         str     — apriltag family string (default 'tag36h11')
  tag_poses:          [float] — flat list [x,y,z,qx,qy,qz,qw] per tag in tag_ids order
                                Default places tag 0 at (0.1,0,0.3) and tag 1 at (0,0.1,0.3),
                                both with identity orientation (tags face +z in map frame).
  camera_frame:       str     — optical frame apriltag_ros uses (default 'oak_rgb_camera_optical_frame')
  base_frame:         str     — robot base (default 'base_link')
  odom_frame:         str     — odometry frame (default 'odom')
  map_frame:          str     — global map frame (default 'map')
  publish_rate:       float   — Hz to re-broadcast map->odom (default 10.0)
  stale_timeout:      float   — seconds without update before status → STALE (default 3.0)
  tf_lookup_timeout:  float   — seconds to wait for a single TF lookup (default 0.15)
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray

import tf2_ros
from tf_transformations import (
    quaternion_multiply,
    quaternion_conjugate,
    quaternion_matrix,
)


class AprilTagLocalizerNode(Node):

    def __init__(self):
        super().__init__('apriltag_localizer')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('tag_ids', [0, 1])
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('stale_timeout', 3.0)
        self.declare_parameter('tf_lookup_timeout', 0.15)

        # Tag poses: flat list [x0,y0,z0,qx0,qy0,qz0,qw0,  x1,y1,z1,qx1,qy1,qz1,qw1, ...]
        # Default: tag 0 at (0.1, 0.0, 0.3), tag 1 at (0.0, 0.1, 0.3), both identity orientation.
        self.declare_parameter('tag_poses', [
            0.1, 0.0, 0.3,  0.0, 0.0, 0.0, 1.0,   # tag 0
            0.0, 0.1, 0.3,  0.0, 0.0, 0.0, 1.0,   # tag 1
        ])

        self._tag_ids   = list(self.get_parameter('tag_ids').value)
        self._family    = self.get_parameter('tag_family').value
        self._cam_frame = self.get_parameter('camera_frame').value
        self._base      = self.get_parameter('base_frame').value
        self._odom      = self.get_parameter('odom_frame').value
        self._map       = self.get_parameter('map_frame').value
        self._stale_t   = self.get_parameter('stale_timeout').value
        self._tf_timeout = self.get_parameter('tf_lookup_timeout').value

        # Parse tag ground-truth poses from flat list.
        # Expected: exactly 7 floats per tag in tag_ids order (x,y,z,qx,qy,qz,qw).
        raw = list(self.get_parameter('tag_poses').value)
        expected_len = 7 * len(self._tag_ids)
        if len(raw) != expected_len:
            raise ValueError(
                f'apriltag_localizer: tag_poses has {len(raw)} floats but '
                f'tag_ids={self._tag_ids} requires exactly {expected_len} '
                f'(7 per tag: x y z qx qy qz qw). Check apriltag_localizer_params.yaml.'
            )

        self._tag_map_poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        for i, tid in enumerate(self._tag_ids):
            b = i * 7
            t = np.array(raw[b:b + 3], dtype=float)
            q = np.array(raw[b + 3:b + 7], dtype=float)  # xyzw

            # Sanity-check the quaternion: zero-norm quats cause silent NaN
            # explosions downstream in the TF math.
            qn = float(np.linalg.norm(q))
            if qn < 1e-6:
                raise ValueError(
                    f'apriltag_localizer: tag {tid} has zero-norm quaternion '
                    f'{q.tolist()}. Use (0,0,0,1) for identity.'
                )
            if abs(qn - 1.0) > 1e-3:
                self.get_logger().warn(
                    f'Tag {tid} quaternion norm={qn:.4f} (not unit) — normalizing.'
                )
                q = q / qn

            self._tag_map_poses[tid] = (t, q)
            self.get_logger().info(
                f'Tag {tid} ({self._family}:{tid}) '
                f'map pose: pos={t.tolist()}  quat(xyzw)={q.tolist()}'
            )

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buf  = tf2_ros.Buffer()
        self._tf_listener  = tf2_ros.TransformListener(self._tf_buf, self)
        self._tf_bcast = tf2_ros.TransformBroadcaster(self)

        # ── State ─────────────────────────────────────────────────────────────
        self._status: str = 'UNLOCALIZED'
        self._last_update: float | None = None        # monotonic time
        self._map_to_odom: tuple[np.ndarray, np.ndarray] | None = None  # (trans, quat)

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.create_subscription(
            AprilTagDetectionArray, '/detections',
            self._on_detections, 10
        )
        self._status_pub = self.create_publisher(String, '/localization_status', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_timer)

        self.get_logger().info(
            f'AprilTag localizer ready — watching for tags {self._tag_ids} '
            f'(family {self._family})'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Detection callback
    # ──────────────────────────────────────────────────────────────────────────

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        """Triggered by every apriltag_ros detection batch.

        apriltag_ros publishes the 6-DOF tag pose directly into the TF tree
        (parent = camera_frame, child = '<family>:<id>').  We resolve the
        map->odom transform from each visible tag and keep the most recent.
        """
        if not msg.detections:
            return

        for det in msg.detections:
            tid = det.id
            if tid not in self._tag_map_poses:
                continue

            tag_frame = f'{self._family}:{tid}'

            # ── Step 1: get tag pose in base_link via TF ──────────────────────
            try:
                tf_base_tag: TransformStamped = self._tf_buf.lookup_transform(
                    self._base,
                    tag_frame,
                    msg.header.stamp,
                    timeout=Duration(seconds=self._tf_timeout),
                )
            except Exception as exc:
                self.get_logger().debug(
                    f'TF lookup {self._base}<-{tag_frame} failed: {exc}',
                    throttle_duration_sec=1.0,
                )
                continue

            # ── Step 2: get odom->base_link (latest available) ────────────────
            try:
                tf_odom_base: TransformStamped = self._tf_buf.lookup_transform(
                    self._odom,
                    self._base,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self._tf_timeout),
                )
            except Exception as exc:
                self.get_logger().debug(
                    f'TF lookup {self._odom}<-{self._base} failed: {exc}',
                    throttle_duration_sec=1.0,
                )
                continue

            # ── Step 3: compose map->odom ──────────────────────────────────────
            result = self._compute_map_to_odom(tf_base_tag, tf_odom_base, tid)
            if result is not None:
                self._map_to_odom = result
                self._last_update = time.monotonic()
                if self._status != 'LOCALIZED':
                    self._status = 'LOCALIZED'
                    self.get_logger().info(
                        f'Initial localization achieved via tag {tid} ({tag_frame})'
                    )
                else:
                    self.get_logger().debug(f'Map->odom updated via tag {tid}')
                break   # one tag per cycle is enough

    # ──────────────────────────────────────────────────────────────────────────
    # Math
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_map_to_odom(
        self,
        tf_base_tag: TransformStamped,
        tf_odom_base: TransformStamped,
        tag_id: int,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Derive T_map_odom from:
          T_base_tag  — tag in base_link frame (from TF lookup)
          T_odom_base — base_link in odom frame (from TF lookup)
          T_map_tag   — tag in map frame (ground truth from params)

        Derivation
        ----------
        T_map_base = T_map_tag  * inv(T_base_tag)
        T_map_odom = T_map_base * inv(T_odom_base)

        where inv(T) means the inverse rigid transform:
          inv(T).rot   = conjugate(T.rot)
          inv(T).trans = -R(conjugate(T.rot)) @ T.trans
        """
        try:
            t_map_tag, q_map_tag = self._tag_map_poses[tag_id]

            # Unpack T_base_tag
            tr = tf_base_tag.transform.translation
            ro = tf_base_tag.transform.rotation
            t_bt = np.array([tr.x, tr.y, tr.z])
            q_bt = np.array([ro.x, ro.y, ro.z, ro.w])

            # inv(T_base_tag) = T_tag_base
            q_tb = quaternion_conjugate(q_bt)
            t_tb = -(quaternion_matrix(q_tb)[:3, :3] @ t_bt)

            # T_map_base = T_map_tag * T_tag_base
            q_mb = quaternion_multiply(q_map_tag, q_tb)
            t_mb = t_map_tag + quaternion_matrix(q_map_tag)[:3, :3] @ t_tb

            # Unpack T_odom_base
            tr2 = tf_odom_base.transform.translation
            ro2 = tf_odom_base.transform.rotation
            t_ob = np.array([tr2.x, tr2.y, tr2.z])
            q_ob = np.array([ro2.x, ro2.y, ro2.z, ro2.w])

            # inv(T_odom_base) = T_base_odom
            q_bo = quaternion_conjugate(q_ob)
            t_bo = -(quaternion_matrix(q_bo)[:3, :3] @ t_ob)

            # T_map_odom = T_map_base * T_base_odom
            q_mo = quaternion_multiply(q_mb, q_bo)
            t_mo = t_mb + quaternion_matrix(q_mb)[:3, :3] @ t_bo

            return (t_mo, q_mo)

        except Exception as exc:
            self.get_logger().warn(f'map->odom computation failed: {exc}')
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # Publish timer
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_timer(self) -> None:
        now = self.get_clock().now()

        # Update staleness state
        if self._status == 'LOCALIZED' and self._last_update is not None:
            if time.monotonic() - self._last_update > self._stale_t:
                self._status = 'STALE'
                self.get_logger().warn(
                    f'No tag detected for >{self._stale_t:.1f} s — '
                    'broadcasting last known map->odom (STALE)'
                )
        elif self._status == 'STALE' and self._last_update is not None:
            if time.monotonic() - self._last_update <= self._stale_t:
                self._status = 'LOCALIZED'

        # Broadcast map->odom even when stale (Nav2 must not lose the frame).
        #
        # TODO(team): we currently keep re-broadcasting the last known
        # map->odom *forever* once we enter STALE — i.e. we trust wheel odom
        # as global for the rest of the mission. On a smooth floor that's
        # fine for a 2-3 cycle autonomy run, but on the lunar simulant we
        # may drift > 50 cm per trip. Options to discuss:
        #   (a) degrade Nav2 goal tolerance (or bail out) when STALE for > N s,
        #   (b) inflate the map->odom TF with synthetic covariance for Nav2
        #       to ignore it, or
        #   (c) swap localizer into a "dead-reckoning only" mode that stops
        #       publishing and lets Nav2 plan in odom_frame directly.
        if self._map_to_odom is not None:
            t_mo, q_mo = self._map_to_odom
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = now.to_msg()
            tf_msg.header.frame_id = self._map
            tf_msg.child_frame_id  = self._odom
            tf_msg.transform.translation.x = float(t_mo[0])
            tf_msg.transform.translation.y = float(t_mo[1])
            tf_msg.transform.translation.z = float(t_mo[2])
            tf_msg.transform.rotation.x = float(q_mo[0])
            tf_msg.transform.rotation.y = float(q_mo[1])
            tf_msg.transform.rotation.z = float(q_mo[2])
            tf_msg.transform.rotation.w = float(q_mo[3])
            self._tf_bcast.sendTransform(tf_msg)

        # Publish human-readable status
        self._status_pub.publish(String(data=self._status))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Publish a fake point cloud so the detector can be exercised with no camera.

    ros2 run lunabot_perception publish_synthetic_cloud.py --boulders 3

A flat ground plane with cubes on it, published on /oak_d/points in the
camera's optical frame, plus the static transform the detector needs to get
back to base_link. That means this alone is enough to see /perception/boulders
populate -- no robot, no sim, no bag.

Deliberately noisy by default: a perfect cloud makes the clustering look far
better than it is.
"""

import argparse
import math
import struct
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

OPTICAL_FRAME = 'oak_d_rgb_camera_optical_frame'


def make_cloud_msg(points, frame_id, stamp):
    """Pack XYZ float32 points into a PointCloud2."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


def build_scene(rng, boulders, noise):
    """Ground plane plus cubes, in BASE_LINK coordinates (X fwd, Y left, Z up)."""
    ground_z = -0.10
    points = []

    x = 0.3
    while x <= 2.5:
        y = -1.2
        while y <= 1.2:
            points.append((x, y, ground_z + rng.gauss(0.0, noise)))
            y += 0.04
        x += 0.04

    placed = []
    for _ in range(boulders):
        # Keep rocks apart, or the clustering merges them and the script
        # looks broken when it is the tolerance doing its job.
        for _attempt in range(50):
            cx = rng.uniform(0.6, 2.2)
            cy = rng.uniform(-1.0, 1.0)
            if all(math.hypot(cx - px, cy - py) > 0.5 for px, py in placed):
                placed.append((cx, cy))
                break
        else:
            continue

        size = rng.uniform(0.12, 0.35)
        half = size / 2
        steps = max(3, int(size / 0.02))
        for i in range(steps):
            for j in range(steps):
                for k in range(steps):
                    points.append(
                        (
                            cx - half + i * size / steps + rng.gauss(0.0, noise),
                            cy - half + j * size / steps + rng.gauss(0.0, noise),
                            ground_z + k * size / steps + rng.gauss(0.0, noise),
                        )
                    )

    return points, placed


def base_link_to_optical(points):
    """base_link (X fwd, Y left, Z up) -> optical (Z fwd, X right, Y down).

    The inverse of the URDF's -pi/2, 0, -pi/2 rotation. Publishing in the
    optical frame rather than base_link is what makes this exercise the
    detector's transform path instead of bypassing it.
    """
    return [(-y, -z, x) for x, y, z in points]


class SyntheticCloudPublisher(Node):
    def __init__(self, args):
        super().__init__('synthetic_cloud_publisher')
        import random

        self.rng = random.Random(args.seed)
        self.args = args

        self.publisher = self.create_publisher(
            PointCloud2,
            '/oak_d/points',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )

        # The detector transforms from the optical frame to base_link. Without
        # this it would sit there logging "no transform yet" forever, and the
        # script would look like it was not publishing.
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self._optical_transform())

        points, placed = build_scene(self.rng, args.boulders, args.noise)
        self.points = base_link_to_optical(points)

        self.get_logger().info(
            f'publishing {len(self.points)} points at {args.rate} Hz, '
            f'{len(placed)} boulders at {[(round(x, 2), round(y, 2)) for x, y in placed]}'
        )

        self.create_timer(1.0 / args.rate, self.publish)

    def _optical_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = OPTICAL_FRAME
        # Quaternion for rpy = (-pi/2, 0, -pi/2).
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        return transform

    def publish(self):
        self.publisher.publish(
            make_cloud_msg(self.points, OPTICAL_FRAME, self.get_clock().now().to_msg())
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--boulders', type=int, default=3, help='how many rocks to scatter')
    parser.add_argument('--seed', type=int, default=0, help='same seed, same scene')
    parser.add_argument('--rate', type=float, default=5.0, help='Hz')
    parser.add_argument(
        '--noise',
        type=float,
        default=0.005,
        help='per-point gaussian sigma, metres. Zero makes the detector look better than it is.',
    )
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticCloudPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

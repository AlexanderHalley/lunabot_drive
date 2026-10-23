# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Ground truth for the boulders, live: /sim/ground_truth/boulders.

The scene's seeded layout is written to JSON at start-up, but small boulders
are dynamic and move when the rover pushes them -- which the rules forbid in
the obstacle zone, and which the JSON cannot show. This publishes every
boulder's CURRENT pose, in the same vision_msgs/Detection3DArray type the
boulder detector emits (docs/SIM_ISAAC.md, "Ground truth and scoring"), in the
sim_ground_truth frame of /sim/ground_truth/odom.

Like the rover's ground truth: for scoring and diagnosis, never for driving.

current_boulders() is plain Python so it is tested in CI; the publisher needs
Isaac and rclpy.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace

from lunabot_sim.scene.boulders import Boulder

logger = logging.getLogger(__name__)

TOPIC = '/sim/ground_truth/boulders'
FRAME = 'sim_ground_truth'
PRIM_PATH = '/World/Boulders/Boulder_{index:03d}'  # arena._spawn's naming


def current_boulders(placed: list[Boulder], dynamic_poses: dict[int, tuple]) -> list[Boulder]:
    """The placed boulders, with live poses for the dynamic ones.

    dynamic_poses maps a boulder's index to (x, y, z, qx, qy, qz, qw). Static
    boulders cannot move, so they keep their placed pose.
    """
    out = []
    for index, boulder in enumerate(placed):
        pose = dynamic_poses.get(index)
        if pose is None:
            out.append(boulder)
            continue
        x, y, z, qx, qy, qz, qw = pose
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        out.append(replace(boulder, position=(x, y, z), yaw=yaw % (2.0 * math.pi)))
    return out


class BoulderTruthPublisher:
    """Publishes current_boulders() at `rate_hz` of sim time. Call step() each loop."""

    def __init__(self, placed: list[Boulder], domain_id: int, rate_hz: float = 5.0):
        import rclpy
        from rclpy.node import Node
        from vision_msgs.msg import Detection3DArray

        self._placed = placed
        self._dynamic = [i for i, b in enumerate(placed) if not b.static]
        self._period = 1.0 / rate_hz
        self._last = -math.inf
        self._view = None
        if not rclpy.ok():
            rclpy.init(domain_id=domain_id)
        self._node = Node('sim_boulder_truth')
        self._pub = self._node.create_publisher(Detection3DArray, TOPIC, 10)
        logger.info('publishing %d boulders (%d movable) on %s', len(placed), len(self._dynamic), TOPIC)

    def _attach(self) -> bool:
        from isaacsim.core.simulation_manager import SimulationManager

        view = SimulationManager.get_physics_sim_view()
        if view is None:
            return False
        if self._dynamic:
            self._view = view.create_rigid_body_view([PRIM_PATH.format(index=i) for i in self._dynamic])
        else:
            self._view = False
        return True

    def step(self, sim_time: float) -> None:
        if sim_time - self._last < self._period:
            return
        if self._view is None and not self._attach():
            return
        self._last = sim_time

        poses = {}
        if self._view:
            raw = self._view.get_transforms()
            raw = raw.cpu().numpy() if hasattr(raw, 'cpu') else raw
            for index, row in zip(self._dynamic, raw.reshape(-1, 7)):
                poses[index] = tuple(float(v) for v in row)
        self._pub.publish(self._message(current_boulders(self._placed, poses), sim_time))

    def _message(self, boulders, sim_time):
        from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

        msg = Detection3DArray()
        msg.header.frame_id = FRAME
        msg.header.stamp.sec = int(sim_time)
        msg.header.stamp.nanosec = int((sim_time % 1.0) * 1e9)
        for index, boulder in enumerate(boulders):
            det = Detection3D()
            det.header = msg.header
            det.id = str(index)
            det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = (
                boulder.position
            )
            det.bbox.center.orientation.z = math.sin(boulder.yaw / 2.0)
            det.bbox.center.orientation.w = math.cos(boulder.yaw / 2.0)
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = boulder.dimensions
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'boulder_static' if boulder.static else 'boulder'
            hypothesis.hypothesis.score = 1.0
            det.results.append(hypothesis)
            msg.detections.append(det)
        return msg

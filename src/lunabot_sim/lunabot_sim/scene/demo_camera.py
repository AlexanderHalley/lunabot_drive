# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Cameras for showing the rover to people: a chase view and an arena overview.

    run_isaac_sim.sh --demo-cameras

Not sensors. Nothing on the rover can see from here, and nothing in the stack
may subscribe to these for driving. They exist so a run can be recorded and
shown (lunabot_bringup's demo_recorder.py) without anyone screen-capturing
the Isaac window, and they work headless.

  /sim/demo/chase/image_raw     behind and above the rover, following it
  /sim/demo/overview/image_raw  fixed, high over the start end of the arena

The chase camera follows a SMOOTHED heading, so a K-turn or a reversing leg
does not whip the view round; chase_pose() is plain Python and tested.
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)

CHASE_PRIM = '/World/DemoCameras/Chase'
OVERVIEW_PRIM = '/World/DemoCameras/Overview'
CHASE_TOPIC = 'sim/demo/chase/image_raw'
OVERVIEW_TOPIC = 'sim/demo/overview/image_raw'

CHASE_BACK = 2.2  # metres behind the rover
CHASE_UP = 1.5  # metres above the ground
CHASE_AHEAD = 1.0  # look at a point this far ahead of the rover
HEADING_TAU = 1.5  # seconds; time constant of the smoothed heading


def smooth_heading(previous: float, current: float, dt: float, tau: float = HEADING_TAU) -> float:
    """First-order lag on an angle, taking the short way round."""
    alpha = 1.0 - math.exp(-max(dt, 0.0) / tau)
    return previous + alpha * math.remainder(current - previous, math.tau)


def chase_pose(x: float, y: float, heading: float):
    """(eye, target) for a camera behind the rover along `heading`."""
    c, s = math.cos(heading), math.sin(heading)
    eye = (x - CHASE_BACK * c, y - CHASE_BACK * s, CHASE_UP)
    target = (x + CHASE_AHEAD * c, y + CHASE_AHEAD * s, 0.0)
    return eye, target


def overview_pose(floor):
    """(eye, target) high over the start end of the arena, looking down its length."""
    if floor is None:
        return (-2.5, -3.5, 4.5), (3.0, 0.0, 0.0)
    (cx, cy), (length, width) = floor.centre, floor.size
    eye = (floor.x_min - 0.15 * length, cy - 0.55 * width, 0.75 * length)
    target = (cx + 0.1 * length, cy, 0.0)
    return eye, target


def look_at_rows(eye, target):
    """USD camera transform rows (row-vector convention) looking from eye at target.

    A USD camera looks down its -Z axis with +Y up; world up is +Z.
    """
    f = [t - e for t, e in zip(target, eye, strict=True)]
    n = math.sqrt(sum(v * v for v in f))
    f = [v / n for v in f]
    r = [f[1], -f[0], 0.0]  # f x (0, 0, 1)
    n = math.hypot(r[0], r[1]) or 1.0
    r = [v / n for v in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    return (
        (r[0], r[1], r[2], 0.0),
        (u[0], u[1], u[2], 0.0),
        (-f[0], -f[1], -f[2], 0.0),
        (eye[0], eye[1], eye[2], 1.0),
    )


class DemoCameras:
    """Builds both cameras and their publishers; call step() every loop."""

    def __init__(
        self,
        articulation_path: str,
        floor,
        domain_id: int,
        width: int = 1280,
        height: int = 720,
        frame_skip: int = 3,
    ):
        self._articulation_path = articulation_path
        self._view = None
        self._heading = None
        self._last = None
        self._ops = {}
        for path, focal in ((CHASE_PRIM, 18.0), (OVERVIEW_PRIM, 16.0)):
            self._ops[path] = _make_camera(path, focal)
        self._set(OVERVIEW_PRIM, *overview_pose(floor))
        for path, topic, name in (
            (CHASE_PRIM, CHASE_TOPIC, 'Chase'),
            (OVERVIEW_PRIM, OVERVIEW_TOPIC, 'Overview'),
        ):
            _build_graph(
                path, topic, f'/World/Graphs/Demo{name}', width, height, frame_skip, domain_id
            )
        logger.info('demo cameras on /%s and /%s', CHASE_TOPIC, OVERVIEW_TOPIC)

    def _set(self, path, eye, target):
        from pxr import Gf

        self._ops[path].Set(Gf.Matrix4d(*look_at_rows(eye, target)))

    def step(self, sim_time: float) -> None:
        if self._view is None:
            from isaacsim.core.simulation_manager import SimulationManager

            sim_view = SimulationManager.get_physics_sim_view()
            if sim_view is None:
                return
            self._view = sim_view.create_articulation_view(self._articulation_path)
        pose = self._view.get_root_transforms()
        pose = pose.cpu().numpy() if hasattr(pose, 'cpu') else pose
        x, y, _, qx, qy, qz, qw = (float(v) for v in pose.reshape(-1)[:7])
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        if self._heading is None:
            self._heading = yaw
        else:
            self._heading = smooth_heading(self._heading, yaw, sim_time - self._last)
        self._last = sim_time
        self._set(CHASE_PRIM, *chase_pose(x, y, self._heading))


def _make_camera(path: str, focal_length: float):
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    camera = UsdGeom.Camera.Define(stage, path)
    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetClippingRangeAttr().Set((0.05, 200.0))
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    return xform.AddTransformOp()


def _build_graph(camera_prim, topic, graph_path, width, height, frame_skip, domain_id):
    import omni.graph.core as og

    from lunabot_sim import compat
    from lunabot_sim.graphs import context

    ctx_node, ctx_values = context.ros2_context_node(domain_id)
    nodes = [
        context.playback_tick_node(),
        ctx_node,
        ('RenderProduct', compat.core_node_type('IsaacCreateRenderProduct')),
        ('Publish', compat.og_node_type('ROS2CameraHelper')),
    ]
    values = ctx_values + [
        ('RenderProduct.inputs:cameraPrim', [camera_prim]),
        ('RenderProduct.inputs:width', width),
        ('RenderProduct.inputs:height', height),
        ('RenderProduct.inputs:enabled', True),
        ('Publish.inputs:frameId', 'sim_demo_camera'),
        ('Publish.inputs:topicName', topic),
        ('Publish.inputs:type', 'rgb'),
        # 60 Hz render / (1 + 3) = 15 Hz: plenty for video, and a 1280x720
        # image at render rate is 165 MB/s on the wire.
        ('Publish.inputs:frameSkipCount', frame_skip),
    ]
    connections = [
        ('OnTick.outputs:tick', 'RenderProduct.inputs:execIn'),
        ('RenderProduct.outputs:execOut', 'Publish.inputs:execIn'),
        ('RenderProduct.outputs:renderProductPath', 'Publish.inputs:renderProductPath'),
        ('Context.outputs:context', 'Publish.inputs:context'),
    ]
    return context.create_graph(graph_path, og.Controller.Keys, nodes, values, connections)

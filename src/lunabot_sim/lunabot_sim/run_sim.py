# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Isaac Sim entry point.

Launched by scripts/run_isaac_sim.sh, which expands the xacro inside the ROS
environment first and passes the resulting plain URDF in.

    src/lunabot_sim/scripts/run_isaac_sim.sh --scene default --seed 7

Then, separately:

    ros2 launch lunabot_bringup robot.launch.py hw:=sim use_sim_time:=true

Two processes on purpose. Isaac's embedded Python is not the ROS distro's
Python, and this module must never import rclpy or xacro -- all ROS traffic
goes through the bridge extension, which links its own DDS. That is also what
lets lunabot_sim be an ordinary ament_python package whose layout tests run in
normal CI.

ORDER MATTERS: start Isaac first. /clock must be publishing before the ROS
graph comes up, or every node with use_sim_time blocks at time zero -- no
error, no log line, just a stack that appears hung.
"""

from __future__ import annotations

import argparse
import json
import logging
import random
import signal
import sys
import time
from pathlib import Path

import numpy as np

logging.basicConfig(
    level=logging.INFO, format='[%(levelname)s] [lunabot_sim.%(module)s] %(message)s'
)
logger = logging.getLogger(__name__)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--urdf', required=True, help='Expanded URDF. Written by run_isaac_sim.sh.'
    )
    parser.add_argument(
        '--scene',
        default='default',
        help="Arena layout: 'ucf' (default), 'ksc' (approximate) or 'open' (no walls).",
    )
    parser.add_argument(
        '--lighting',
        default='arena',
        help="'arena' (default: indoor, soft, readable) or 'lunar' (low sun, black "
        'shadows; for stress-testing perception).',
    )
    parser.add_argument(
        '--no-markings',
        dest='markings',
        action='store_false',
        help='No painted zone outlines. The real arena is unmarked sand; use this for '
        'perception and SLAM evaluation.',
    )
    parser.add_argument(
        '--seed', type=int, default=0, help='Boulder layout seed. Same seed, same scene.'
    )
    parser.add_argument(
        '--start-pose',
        default='0,0,0',
        help="Where the rover starts: 'x,y,yaw' (metres, degrees, arena frame) or "
        "'random' -- anywhere in the start zone that leaves room for the start spin, "
        'any heading, drawn from --seed. The competition draws the start pose at random; '
        'the wall AprilTags are how the rover finds out which (fiducial_localizer.py).',
    )
    parser.add_argument('--headless', action='store_true', help='No viewport.')
    parser.add_argument(
        '--no-realtime',
        dest='realtime',
        action='store_false',
        help='Run as fast as the GPU allows instead of pacing to real time.',
    )
    parser.add_argument(
        '--no-sand',
        dest='sand',
        action='store_false',
        help='Plain PhysX friction under the wheels instead of the sand model '
        '(robot/soil.py). For comparison only: it pivots, and barely arcs.',
    )
    parser.add_argument(
        '--demo-cameras',
        action='store_true',
        help='Publish a chase view and an arena overview on /sim/demo/*, for recording '
        'runs to show people (scene/demo_camera.py). Not sensors.',
    )
    parser.add_argument('--domain-id', type=int, default=42, help='ROS_DOMAIN_ID for the bridge.')
    parser.add_argument(
        '--ground-truth',
        default=None,
        help='Where to write the boulder ground-truth JSON. Defaults next to the URDF.',
    )
    parser.add_argument(
        '--physics-dt', type=float, default=1.0 / 240.0, help='Physics step, seconds.'
    )
    parser.add_argument(
        '--render-dt', type=float, default=1.0 / 60.0, help='Render step, seconds.'
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    # ==================== SimulationApp FIRST ====================
    # Before any other omni import, without exception. Importing omni.* before
    # the app exists fails in ways that do not mention the ordering. This is
    # why the imports below are inside the function rather than at module
    # scope, and why ruff's import rules are relaxed for this file.
    # =============================================================
    from lunabot_sim import compat

    simulation_app_class = compat.get_simulation_app_class()
    simulation_app = simulation_app_class(
        {
            'headless': args.headless,
            'width': 1280,
            'height': 720,
        }
    )

    # 6.x close() ends the process (fast shutdown), so the traceback has to be
    # logged before it and the exit status passed into it. Otherwise a failed
    # startup prints nothing and exits 0.
    exit_code = 1
    try:
        exit_code = _run(args, simulation_app)
    except Exception:
        logger.exception('lunabot_sim failed')
        raise
    finally:
        # Always close, including on exception. A leaked SimulationApp holds
        # the GPU and the next run fails to start for unrelated-looking
        # reasons.
        try:
            simulation_app.close(exit_code=exit_code)
        except TypeError:  # 4.5/5.x: no exit_code, and close() returns
            simulation_app.close()
    return exit_code


def _run(args, simulation_app) -> int:
    from lunabot_sim import compat
    from lunabot_sim.graphs import camera, clock, ground_truth, imu, joints
    from lunabot_sim.robot import articulation, importer, traction
    from lunabot_sim.scene import arena

    logger.info('Isaac Sim version: %s', compat.isaac_version())

    bridge = compat.enable_ros2_bridge()
    logger.info('ROS 2 bridge extension: %s', bridge)

    # ---- Robot, part 1 ----
    # 6.x converts the URDF to USD by opening the result as the stage, so it
    # runs before anything is built, and a fresh stage replaces it after.
    robot_usd = importer.convert_urdf(Path(args.urdf))
    if robot_usd is not None:
        import omni.usd

        omni.usd.get_context().new_stage()

    world_class = compat.get_world_class()
    world = world_class(
        stage_units_in_meters=1.0,
        physics_dt=args.physics_dt,
        rendering_dt=args.render_dt,
    )

    # ---- Scene ----
    config = arena.scene_config(
        args.scene, seed=args.seed, markings=args.markings, light=args.lighting
    )
    ground_truth_path = Path(
        args.ground_truth or (Path(args.urdf).parent / f'ground_truth_seed{args.seed}.json')
    )
    placed = arena.build(world, config, ground_truth_path)
    logger.info('scene "%s" seed %d: %d boulders', args.scene, args.seed, len(placed))

    # ---- Robot, part 2 ----
    robot_path = '/World/Lunabot'
    if robot_usd is not None:
        articulation_path = importer.add_to_stage(robot_usd, robot_path)
    else:
        articulation_path = importer.import_urdf(Path(args.urdf), robot_path)
    start = _start_pose(args.start_pose, config.layout, args.seed)
    _place_on_ground(robot_path, Path(args.urdf), start)
    (Path(args.urdf).parent / 'start_pose.json').write_text(
        json.dumps({'x': start[0], 'y': start[1], 'yaw_deg': start[2]})
    )
    articulation.configure(articulation_path, robot_path=robot_path)
    articulation.set_solver_iterations(articulation_path)
    wheel_paths = traction.make_wheels_frictionless(robot_path) if args.sand else None

    camera_prim = _find_camera_prim(articulation_path, Path(args.urdf))
    imu_prim = _create_imu_sensor(articulation_path, Path(args.urdf))

    # ---- ROS 2 graphs ----
    # One graph per concern. A graph that fails to evaluate takes down
    # everything in it, and losing the camera must not stop /clock.
    clock.build(domain_id=args.domain_id)
    joints.build(articulation_path, domain_id=args.domain_id)
    # No cloud from Isaac: robot.launch.py builds /oak_d/points from depth,
    # decimated and clipped exactly as on the robot (point_cloud.launch.py).
    camera.build(camera_prim, domain_id=args.domain_id, publish_pointcloud=False)
    imu.build(imu_prim, domain_id=args.domain_id)
    ground_truth.build(articulation_path, domain_id=args.domain_id)
    demo = None
    if args.demo_cameras:
        from lunabot_sim.scene.demo_camera import DemoCameras

        demo = DemoCameras(articulation_path, config.layout.floor, domain_id=args.domain_id)

    world.reset()
    if not args.headless:
        _frame_viewport(config.layout)
    if wheel_paths is not None:
        # After reset: the tensor views need a running physics scene.
        traction.SandTraction(
            world, articulation_path, wheel_paths, traction.wheel_radius(Path(args.urdf))
        )
    logger.info('running. /clock is live -- start the ROS bringup now.')

    running = True

    def stop(_signum, _frame):
        nonlocal running
        logger.info('shutting down')
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    # render=True even headless. It is what runs Kit's app update, and the
    # update is what evaluates the OnPlaybackTick graphs: with render=False
    # physics steps, the timeline plays, and not one ROS message is published.
    # Headless only drops the window.
    # Live boulder poses, for scoring and for diagnosing pushed rocks. Not
    # essential to the sim: if rclpy or vision_msgs is missing, say so and run.
    try:
        from lunabot_sim.scene.boulder_truth import BoulderTruthPublisher

        boulder_truth = BoulderTruthPublisher(placed, domain_id=args.domain_id)
    except Exception as exc:  # noqa: BLE001
        logger.warning('no live boulder ground truth: %s', exc)
        boulder_truth = None

    pacer = RealTimePacer(world) if args.realtime else None
    while running and simulation_app.is_running():
        world.step(render=True)
        if boulder_truth is not None:
            boulder_truth.step(world.current_time)
        if demo is not None:
            demo.step(world.current_time)
        if pacer is not None:
            pacer.wait()

    return 0


def _frame_viewport(layout) -> None:
    """Point the GUI camera at the arena: behind and above the start, looking down it.

    Isaac otherwise opens wherever its default camera happens to be, which
    for this scene is a corner view of a wall.
    """
    import numpy as np
    from isaacsim.core.utils.viewports import set_camera_view

    floor = layout.floor
    if floor is None:  # open scene: frame the rock field ahead of the spawn
        eye, target = (-3.0, -3.0, 3.0), (3.0, 0.0, 0.0)
    else:
        (cx, cy), (length, _) = floor.centre, floor.size
        eye = (floor.x_min - 0.35 * length, cy - 0.45 * length, 0.55 * length)
        target = (cx, cy, 0.0)
    set_camera_view(eye=np.array(eye), target=np.array(target))


class RealTimePacer:
    """Hold the simulation to a real-time factor of 1.0.

    Isaac does not pace itself: on the desktop it runs 1.2-1.6x real time.
    The ROS stack follows /clock and does not care, but a person teleoperating,
    a script timing /cmd_vel by the wall clock, or a bag recorded alongside
    all do. After each step this sleeps off however far simulated time has
    run ahead of wall time.

    It only slows the sim down. When a step takes longer than it simulates,
    the sim falls behind and nothing here can help -- see "real-time factor"
    in docs/SIM_ISAAC.md. Once more than MAX_LAG behind, the reference is
    reset, so a slow stretch (shader compile, a heavy scene) is not followed by
    a sprint to catch up, which is the one thing worse than running slow.
    """

    MAX_LAG = 0.25  # seconds

    def __init__(self, world):
        self._world = world
        self._reset()

    def _reset(self):
        self._wall_start = time.monotonic()
        self._sim_start = self._world.current_time

    def wait(self):
        ahead = (self._world.current_time - self._sim_start) - (
            time.monotonic() - self._wall_start
        )
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -self.MAX_LAG:
            self._reset()


def _find_camera_prim(articulation_path: str, urdf_path: Path) -> str:
    """The camera prim for the OAK-D's RGB optical frame, created if absent.

    The URDF has no camera sensor tag, and importer.py merges fixed joints, so
    the optical frames do not survive as prims. A Camera is created under the
    articulation root at the optical frame's pose, read from the URDF's own
    joint chain so the mount position stays defined in one place. The
    frame_id on the image topics comes from the graph configuration, not from
    this prim's name, so only the pose matters.
    """
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(articulation_path)

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Camera):
            logger.info('using camera prim %s', prim.GetPath())
            return str(prim.GetPath())

    transform = _urdf_frame_transform(urdf_path, 'base_link', CAMERA_OPTICAL_FRAME)
    # ROS optical frame: +Z forward, +Y down. USD camera: -Z forward, +Y up.
    # The same frame, turned 180 degrees about X.
    transform = transform @ np.diag([1.0, -1.0, -1.0, 1.0])

    path = f'{articulation_path}/oak_d_camera'
    camera = UsdGeom.Camera.Define(stage, path)
    # Gf matrices are row-vector: the transpose of the column-vector form.
    camera.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))

    # The lens. A bare UsdGeom.Camera is a 50 mm lens (24 degrees across) with
    # a 1 m near plane: the rover saw a sliver of the arena and nothing within
    # a metre of the camera -- the ground in front of it least of all.
    camera.CreateHorizontalApertureAttr().Set(CAMERA_APERTURE_MM)
    camera.CreateVerticalApertureAttr().Set(CAMERA_APERTURE_MM * 400.0 / 640.0)
    camera.CreateFocalLengthAttr().Set(
        CAMERA_APERTURE_MM / 2.0 / np.tan(np.radians(CAMERA_HFOV_DEG) / 2.0)
    )
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(*CAMERA_CLIPPING_M))
    logger.info('created camera prim %s at the %s pose', path, CAMERA_OPTICAL_FRAME)
    return path


CAMERA_OPTICAL_FRAME = 'oak_d_rgb_camera_optical_frame'

# OAK-D S2 colour camera (IMX378), standard-FOV variant: 69 degrees across,
# per the Luxonis datasheet -- VERIFY against your unit. Depth is aligned to
# RGB on the robot (i_align_depth), so this is the depth FOV too. The aperture
# only sets the units; the angle is what matters. The near plane is roughly
# where the OAK-D's stereo depth starts; the far plane is generous because the
# 2 m clip lives in the consumers, as on hardware.
CAMERA_HFOV_DEG = 69.0
CAMERA_APERTURE_MM = 20.955
CAMERA_CLIPPING_M = (0.2, 20.0)

# Dropped from this high onto the ground, so the wheels start just clear of it.
SPAWN_CLEARANCE_M = 0.005


def _start_pose(spec: str, layout, seed: int) -> tuple[float, float, float]:
    """(x, y, yaw degrees) from --start-pose."""
    if spec != 'random':
        x, y, yaw = (float(v) for v in spec.split(','))
        return x, y, yaw
    area = layout.start_area()
    if area is None:
        raise SystemExit(f'--start-pose random needs an arena with a start zone, not {layout.name}')
    rng = random.Random(seed * 7919 + 1)
    return (
        round(rng.uniform(area.x_min, area.x_max), 3),
        round(rng.uniform(area.y_min, area.y_max), 3),
        round(rng.uniform(-180.0, 180.0), 1),
    )


def _place_on_ground(robot_path: str, urdf_path: Path, start=(0.0, 0.0, 0.0)) -> None:
    """Raise the robot so its wheels start on the ground, not in it.

    Imported as-is, base_link sits at z = 0 with the wheels a full radius
    below the ground plane. PhysX only partly pushes them back out, and the
    rover then drives around about 2.5 cm sunk into the terrain. The height
    comes from the URDF: base_footprint is at z = -wheel_radius under
    base_link by construction (see lunabot.urdf.xacro), and the drop-centre
    middle axle sits wheel_drop lower still -- the LOWEST wheel sets it.
    """
    import omni.usd
    from pxr import Gf, UsdGeom

    from lunabot_sim.robot.traction import WHEEL_LINKS

    footprint = _urdf_frame_transform(urdf_path, 'base_link', 'base_footprint')
    lowest_axle = min(
        _urdf_frame_transform(urdf_path, 'base_link', link)[2, 3] for link in WHEEL_LINKS
    )
    height = -footprint[2, 3] - min(lowest_axle, 0.0) + SPAWN_CLEARANCE_M

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(robot_path)
    x, y, yaw = start
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(x, y, height))
    UsdGeom.XformCommonAPI(prim).SetRotate(Gf.Vec3f(0.0, 0.0, yaw))
    logger.info('start pose: x %.3f y %.3f yaw %.1f deg, z %.3f m', x, y, yaw, height)


def _urdf_frame_transform(urdf_path: Path, parent: str, child: str):
    """4x4 transform of `child` in `parent`, composed along the URDF's joints."""
    import xml.etree.ElementTree as ET

    joints = {}
    # findall, not iter: <ros2_control> also has <joint> elements, without links.
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        origin = joint.find('origin')
        attrib = origin.attrib if origin is not None else {}
        xyz = [float(v) for v in attrib.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in attrib.get('rpy', '0 0 0').split()]
        joints[joint.find('child').get('link')] = (joint.find('parent').get('link'), xyz, rpy)

    transform = np.eye(4)
    link = child
    while link != parent:
        if link not in joints:
            raise RuntimeError(f'{child} is not below {parent} in {urdf_path}')
        link, xyz, rpy = joints[link]
        transform = _origin_matrix(xyz, rpy) @ transform
    return transform


def _origin_matrix(xyz, rpy):
    """URDF <origin>: translate, then fixed-axis roll-pitch-yaw (Rz @ Ry @ Rx)."""
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    )
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix[:3, 3] = xyz
    return matrix


def _create_imu_sensor(articulation_path: str, urdf_path: Path) -> str:
    """Attach an IMU sensor at the camera's IMU frame.

    The API differs across versions; compat.create_imu_sensor resolves it.
    6.x samples every physics step and has no frequency setting, so the real
    BNO086's 400 Hz is not reproduced -- the IMU runs at the physics rate.

    The prim gets the URDF's oak_d_imu_frame pose, like the camera. The
    messages say oak_d_imu_frame, which is pitched down with the camera; a
    sensor left level at base_link measured the yaw rate the EKF then rotated
    by that pitch -- every turn read cos(15 deg) short, 6 deg per half turn,
    and the return leg ended 0.6 m off.
    """
    import omni.usd
    from lunabot_sim import compat
    from pxr import Gf, UsdGeom

    path = compat.create_imu_sensor(f'{articulation_path}/oak_d_imu')
    transform = _urdf_frame_transform(urdf_path, 'base_link', IMU_FRAME)
    xform = UsdGeom.Xformable(omni.usd.get_context().get_stage().GetPrimAtPath(path))
    xform.ClearXformOpOrder()
    # Gf matrices are row-vector: the transpose of the column-vector form.
    xform.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
    logger.info('created IMU sensor at %s, at the %s pose', path, IMU_FRAME)
    return path


IMU_FRAME = 'oak_d_imu_frame'


if __name__ == '__main__':
    sys.exit(main())

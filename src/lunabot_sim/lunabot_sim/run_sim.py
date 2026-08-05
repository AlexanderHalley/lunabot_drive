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
import logging
import signal
import sys
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format='[%(levelname)s] [lunabot_sim.%(module)s] %(message)s'
)
logger = logging.getLogger(__name__)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--urdf', required=True, help='Expanded URDF. Written by run_isaac_sim.sh.'
    )
    parser.add_argument('--scene', default='default', help='Scene config name.')
    parser.add_argument(
        '--seed', type=int, default=0, help='Boulder layout seed. Same seed, same scene.'
    )
    parser.add_argument('--headless', action='store_true', help='No viewport.')
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

    try:
        return _run(args, simulation_app)
    finally:
        # Always close, including on exception. A leaked SimulationApp holds
        # the GPU and the next run fails to start for unrelated-looking
        # reasons.
        simulation_app.close()


def _run(args, simulation_app) -> int:
    from lunabot_sim import compat
    from lunabot_sim.graphs import camera, clock, ground_truth, imu, joints
    from lunabot_sim.robot import articulation, importer
    from lunabot_sim.scene import arena

    logger.info('Isaac Sim version: %s', compat.isaac_version())

    bridge = compat.enable_ros2_bridge()
    logger.info('ROS 2 bridge extension: %s', bridge)

    world_class = compat.get_world_class()
    world = world_class(
        stage_units_in_meters=1.0,
        physics_dt=args.physics_dt,
        rendering_dt=args.render_dt,
    )

    # ---- Scene ----
    config = arena.SceneConfig(seed=args.seed)
    ground_truth_path = Path(
        args.ground_truth or (Path(args.urdf).parent / f'ground_truth_seed{args.seed}.json')
    )
    placed = arena.build(world, config, ground_truth_path)
    logger.info('scene "%s" seed %d: %d boulders', args.scene, args.seed, len(placed))

    # ---- Robot ----
    articulation_path = importer.import_urdf(Path(args.urdf))
    articulation.configure(articulation_path)
    articulation.set_solver_iterations(articulation_path)

    camera_prim = _find_camera_prim(articulation_path)
    imu_prim = _create_imu_sensor(articulation_path)

    # ---- ROS 2 graphs ----
    # One graph per concern. A graph that fails to evaluate takes down
    # everything in it, and losing the camera must not stop /clock.
    clock.build(domain_id=args.domain_id)
    joints.build(articulation_path, domain_id=args.domain_id)
    camera.build(camera_prim, domain_id=args.domain_id)
    imu.build(imu_prim, domain_id=args.domain_id)
    ground_truth.build(articulation_path, domain_id=args.domain_id)

    world.reset()
    logger.info('running. /clock is live -- start the ROS bringup now.')

    running = True

    def stop(_signum, _frame):
        nonlocal running
        logger.info('shutting down')
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    while running and simulation_app.is_running():
        world.step(render=not args.headless)

    return 0


def _find_camera_prim(articulation_path: str) -> str:
    """Locate the camera prim the URDF's optical frame corresponds to.

    VERIFY: importer.py merges fixed joints, so the optical frames may not
    survive as prims. If this raises, either create a Camera prim explicitly
    at the right pose, or set merge_fixed_joints False and pay the solver
    cost. The frame_id published on the image topics comes from the graph
    configuration, not from this prim's name, so the pose is what matters.
    """
    from pxr import Usd, UsdGeom
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(articulation_path)

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Camera):
            logger.info('using camera prim %s', prim.GetPath())
            return str(prim.GetPath())

    raise RuntimeError(
        f'no camera prim under {articulation_path}. URDF has no camera sensor tag, so one '
        'must be created explicitly -- see _find_camera_prim in run_sim.py.'
    )


def _create_imu_sensor(articulation_path: str) -> str:
    """Attach an IMU sensor at the camera's IMU frame.

    VERIFY: the IMU sensor API moved from omni.isaac.sensor to
    isaacsim.sensors.physics in 4.5.
    """
    from lunabot_sim import compat

    imu_sensor_class = compat.import_first(
        ('isaacsim.sensors.physics', 'omni.isaac.sensor'), 'IMUSensor'
    )

    path = f'{articulation_path}/oak_d_imu'
    imu_sensor_class(
        prim_path=path,
        name='oak_d_imu',
        frequency=400,  # matches the real BNO086 configuration
        translation=(0.0, 0.0, 0.0),
    )
    logger.info('created IMU sensor at %s', path)
    return path


if __name__ == '__main__':
    sys.exit(main())

"""Import the rover's URDF into the stage.

The URDF handed to this function is ALREADY EXPANDED. scripts/run_isaac_sim.sh
runs xacro inside the ROS environment and passes the result in, because Isaac's
embedded Python is not the ROS distro's Python and forcing xacro (or rclpy)
into it is a losing fight. That separation is why this whole package can be
ament_python with no ROS runtime dependency, and why its layout tests run in
ordinary CI.
"""

from __future__ import annotations

import logging
from pathlib import Path

from lunabot_sim import compat

logger = logging.getLogger(__name__)


def import_urdf(urdf_path: Path, prim_path: str = '/World/Lunabot') -> str:
    """Import a plain URDF and return the prim path of the articulation root.

    ==================== VERIFY ====================
    The URDF importer's Python entry point has changed more than once across
    Isaac versions -- acquire_urdf_interface() with an ImportConfig, a
    URDFParseAndImportFile kit command, and helper wrappers have all been the
    documented approach at some point.

    Both known shapes are attempted below. If neither works, the error lists
    what was tried; find the current API in the URDF Importer extension's docs
    and add a branch here.
    ================================================
    """
    urdf_path = Path(urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f'{urdf_path} does not exist. It should have been written by '
            'run_isaac_sim.sh expanding lunabot.urdf.xacro -- check that step ran.'
        )

    compat.enable_extension(compat.URDF_IMPORTER_EXTENSIONS)

    config = _import_config()
    errors = []

    # Newer API: a kit command taking the config.
    try:
        import omni.kit.commands

        result, prim = omni.kit.commands.execute(
            'URDFParseAndImportFile',
            urdf_path=str(urdf_path),
            import_config=config,
            get_articulation_root=True,
        )
        if result and prim:
            logger.info('imported %s to %s', urdf_path.name, prim)
            return prim
        errors.append(f'URDFParseAndImportFile returned result={result} prim={prim}')
    except Exception as exc:  # noqa: BLE001
        errors.append(f'URDFParseAndImportFile: {exc}')

    # Older API: acquire the interface, parse, then import.
    try:
        urdf = compat.import_first(('isaacsim.asset.importer.urdf', 'omni.importer.urdf'), '_urdf')
        interface = urdf.acquire_urdf_interface()
        robot_model = interface.parse_urdf(str(urdf_path.parent), urdf_path.name, config)
        prim = interface.import_robot(
            str(urdf_path.parent), urdf_path.name, robot_model, config, ''
        )
        logger.info('imported %s to %s (legacy API)', urdf_path.name, prim)
        return prim
    except Exception as exc:  # noqa: BLE001
        errors.append(f'acquire_urdf_interface: {exc}')

    raise compat.IsaacCompatError('could not import the URDF. Tried:\n  ' + '\n  '.join(errors))


def _import_config():
    """Build the importer configuration.

    The settings that matter for this robot, and why:
    """
    import omni.kit.commands

    _, config = omni.kit.commands.execute('URDFCreateImportConfig')

    # False. The rover must be free to drive; a fixed base is for arms.
    config.set_fix_base(False)

    # True. The URDF's inertia tensors are computed from primitive solids in
    # inertials.xacro and are order-of-magnitude estimates. Letting Isaac
    # recompute from the collision geometry is more consistent -- and a wrong
    # inertia tensor is the usual cause of an articulation that jitters,
    # sinks, or launches itself.
    config.set_import_inertia_tensor(False)

    # Merge fixed joints. base_footprint and the six camera frames are pure
    # coordinate frames with no mass; keeping them as articulation links adds
    # solver work for nothing. TF still carries them, because
    # robot_state_publisher builds the tree from the URDF, not from Isaac.
    config.set_merge_fixed_joints(True)

    # Convex decomposition for collision. The wheels are cylinders and the
    # chassis is a box, so this is cheap; it matters if anyone adds a mesh.
    config.set_convex_decomp(False)

    # Do NOT let the importer create drives. articulation.py configures them
    # explicitly for velocity control, and importer defaults are position
    # drives with stiffness -- which fight every velocity command.
    config.set_default_drive_type(0)
    config.set_default_drive_strength(0.0)
    config.set_default_position_drive_damping(0.0)

    config.set_self_collision(False)
    config.set_make_default_prim(False)
    config.set_distance_scale(1.0)

    return config

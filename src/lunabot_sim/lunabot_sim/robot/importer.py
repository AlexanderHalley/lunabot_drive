# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Import the rover's URDF into the stage.

The URDF handed to this function is ALREADY EXPANDED. scripts/run_isaac_sim.sh
runs xacro inside the ROS environment and passes the result in, because Isaac's
embedded Python is not the ROS distro's Python and forcing xacro (or rclpy)
into it is a losing fight. That separation is why this whole package can be
ament_python with no ROS runtime dependency, and why its layout tests run in
ordinary CI.

Two importer APIs, by Isaac version:

  6.x       convert_urdf() writes a USD file, then add_to_stage() references
            it. The conversion OPENS its output as the current stage, so it
            must run before the World and the scene exist -- see run_sim.py.
  4.5/5.x   import_urdf() imports straight into the open stage through kit
            commands. Never run against those versions since the 6.x port.
"""

from __future__ import annotations

import logging
import shutil
from pathlib import Path

from lunabot_sim import compat

logger = logging.getLogger(__name__)


def convert_urdf(urdf_path: Path) -> Path | None:
    """Convert the URDF to USD with the 6.x importer. None if this Isaac has none.

    Writes to usd/ beside the URDF (the cache directory run_isaac_sim.sh
    uses), replacing the previous run's output, and leaves the converted asset
    open as the stage. The caller opens a fresh
    stage afterwards.
    """
    urdf_path = _check_urdf(urdf_path)
    compat.enable_extension(compat.URDF_IMPORTER_EXTENSIONS)
    try:
        importer_class = compat.import_first(compat.URDF_IMPORTER_EXTENSIONS, 'URDFImporter')
        config_class = compat.import_first(compat.URDF_IMPORTER_EXTENSIONS, 'URDFImporterConfig')
    except compat.IsaacCompatError:
        return None

    # One copy of the output. The importer numbers its output directory when
    # the name is taken (lunabot_sim_1, _2, ...), so the cache would otherwise
    # grow by a robot per run.
    out_dir = urdf_path.parent / 'usd'
    shutil.rmtree(out_dir, ignore_errors=True)
    out_dir.mkdir(parents=True)

    config = config_class(
        urdf_path=str(urdf_path),
        usd_path=str(out_dir),
        # False. The rover must be free to drive; a fixed base is for arms.
        fix_base=False,
        # base_footprint and the camera frames are massless coordinate frames;
        # as articulation links they add solver work for nothing. TF still
        # carries them, because robot_state_publisher builds the tree from the
        # URDF, not from Isaac.
        merge_fixed_joints=True,
        allow_self_collision=False,
        # Velocity targets with zero stiffness, matching what articulation.py
        # authors. Importer defaults are position drives with stiffness, which
        # fight every velocity command.
        joint_target_type='velocity',
        override_joint_stiffness=0.0,
    )
    # The 4.5/5.x path asked the importer to recompute inertia from the
    # collision geometry. 6.x has no such option and keeps the URDF's
    # inertials.xacro estimates -- the first suspect if the rover jitters.
    usd_path = Path(importer_class(config).import_urdf())
    logger.info('converted %s to %s', urdf_path.name, usd_path)
    return usd_path


def add_to_stage(usd_path: Path, prim_path: str = '/World/Lunabot') -> str:
    """Reference the converted robot into the open stage. Returns the articulation root."""
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, 'Xform')
    prim.GetReferences().AddReference(str(usd_path))

    # The 6.x asset carries physics backends as a variant; select PhysX, as
    # the importer's own tests do.
    variants = prim.GetVariantSets()
    if variants.HasVariantSet('Physics'):
        physics = variants.GetVariantSet('Physics')
        if 'physx' in physics.GetVariantNames():
            physics.SetVariantSelection('physx')

    for candidate in Usd.PrimRange(prim):
        if candidate.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = str(candidate.GetPath())
            logger.info('imported %s to %s, articulation root %s', usd_path.name, prim_path, root)
            return root

    raise compat.IsaacCompatError(
        f'{usd_path} referenced at {prim_path}, but nothing under it has '
        'ArticulationRootAPI. Inspect the asset in the Stage window.'
    )


def _check_urdf(urdf_path: Path) -> Path:
    urdf_path = Path(urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f'{urdf_path} does not exist. It should have been written by '
            'run_isaac_sim.sh expanding lunabot.urdf.xacro -- check that step ran.'
        )
    return urdf_path


def import_urdf(urdf_path: Path, prim_path: str = '/World/Lunabot') -> str:
    """4.5/5.x: import a plain URDF and return the articulation root's prim path.

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
    urdf_path = _check_urdf(urdf_path)
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

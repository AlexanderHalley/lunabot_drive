# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Isaac Sim version shims.

Isaac Sim 4.5 renamed essentially every namespace and every OmniGraph node
type. Rather than pin a version and break silently on upgrade, everything
version-sensitive is resolved here, once, at startup -- and when resolution
fails the error names every candidate that was tried, so the fix is obvious
instead of archaeological.

    old (<= 4.2)                     new (4.5, 5.x)
    omni.isaac.kit.SimulationApp      isaacsim.SimulationApp
    omni.isaac.core.World             isaacsim.core.api.World
    omni.isaac.ros2_bridge            isaacsim.ros2.bridge
    omni.importer.urdf                isaacsim.asset.importer.urdf
    omni.isaac.sensor                 isaacsim.sensors.physics

6.0 deprecated isaacsim.core.api (World, scene objects) and
isaacsim.sensors.physics in favour of isaacsim.core.experimental.* and
isaacsim.sensors.experimental.physics. The deprecated extensions still ship in
extsDeprecated/, on the search path but not enabled, so World is enabled here
before it is imported. Moving off them is its own job.

============================ VERIFIED ============================
Probed against Isaac Sim 6.2.0 (6.2.0-rc.14+release.52805), 2026-10-21,
Ubuntu 24.04, ROS 2 Jazzy. Every candidate listed FIRST below resolved there.
The older candidates are kept for 4.5/5.x and were never probed.

The URDF importer was rewritten in 6.x (URDFImporter converts to a USD file);
robot/importer.py handles that, and the old kit-command API is gone.

On a new version, re-run scripts/probe_isaac_api.sh before anything else.
To pin a miss down by hand, in a running Isaac session:
  - OmniGraph node types: open the OmniGraph editor, select a node, read the
    type from the property panel.
  - Extension IDs: Window > Extensions, or
    omni.kit.app.get_app().get_extension_manager().get_extensions()

Fix them here and nowhere else.
================================================================
"""

from __future__ import annotations

import importlib
import logging

logger = logging.getLogger(__name__)

# Extension IDs, newest naming first.
ROS2_BRIDGE_EXTENSIONS = ('isaacsim.ros2.bridge', 'omni.isaac.ros2_bridge')
URDF_IMPORTER_EXTENSIONS = ('isaacsim.asset.importer.urdf', 'omni.importer.urdf')

# Deprecated in 6.0, still shipped and enabled on demand. See the docstring.
WORLD_EXTENSIONS = ('isaacsim.core.api', 'omni.isaac.core')

# The IMU sensor authoring class. 6.x: IMU.create(path) in the experimental
# module. 4.5/5.x: IMUSensor(prim_path=...) in the module that 6.0 deprecated.
IMU_SENSOR_MODULES = ('isaacsim.sensors.experimental.physics',)
LEGACY_IMU_SENSOR_MODULES = ('isaacsim.sensors.physics', 'omni.isaac.sensor')

# OmniGraph node-type prefixes. Must match whichever bridge extension loaded.
OG_PREFIXES = ('isaacsim.ros2.bridge', 'omni.isaac.ros2_bridge')
OG_CORE_PREFIXES = ('isaacsim.core.nodes', 'omni.isaac.core_nodes')
# IsaacReadIMU is registered by the sensor extension, not core nodes.
OG_SENSOR_PREFIXES = ('isaacsim.sensors.physics', 'omni.isaac.sensor')


class IsaacCompatError(RuntimeError):
    """Raised when nothing in a candidate list resolves."""


def import_first(candidates: tuple[str, ...], attribute: str | None = None):
    """Import the first module that exists, optionally returning an attribute.

    Returns the module (or attribute). Raises IsaacCompatError naming every
    candidate tried, because "ModuleNotFoundError: isaacsim" alone does not
    tell you that four other names were also attempted.
    """
    errors = []
    for name in candidates:
        try:
            module = importlib.import_module(name)
        except ImportError as exc:
            errors.append(f'{name}: {exc}')
            continue

        if attribute is None:
            logger.debug('resolved %s', name)
            return module
        if hasattr(module, attribute):
            logger.debug('resolved %s.%s', name, attribute)
            return getattr(module, attribute)
        errors.append(f'{name}: imported but has no attribute {attribute!r}')

    raise IsaacCompatError(
        'none of the candidates resolved'
        + (f' (looking for {attribute!r})' if attribute else '')
        + ':\n  '
        + '\n  '.join(errors)
        + '\n\nThis is almost certainly an Isaac Sim version difference. '
        'Update the candidate lists in lunabot_sim/compat.py -- see the module '
        'docstring for how to find the real names.'
    )


def get_simulation_app_class():
    """SimulationApp. Must be constructed BEFORE any other omni import."""
    return import_first(('isaacsim', 'isaacsim.simulation_app', 'omni.isaac.kit'), 'SimulationApp')


def get_world_class():
    """World / simulation context. Safe only after SimulationApp exists.

    On 6.x the extension is deprecated and off until enabled; on 4.5/5.x it is
    already on, and enabling it again is a no-op. A failure to enable is left
    for import_first to report, since its error names every candidate.
    """
    try:
        enable_extension(WORLD_EXTENSIONS)
    except IsaacCompatError as exc:
        logger.warning('%s', exc)
    return import_first(WORLD_EXTENSIONS, 'World')


def create_imu_sensor(path: str) -> str:
    """Create an IMU sensor prim at `path`. Returns the prim path actually used.

    6.x auto-numbers the path if it is taken, so the returned path is the one
    the IMU graph must read.
    """
    try:
        imu_class = import_first(IMU_SENSOR_MODULES, 'IMU')
    except IsaacCompatError:
        imu_sensor_class = import_first(LEGACY_IMU_SENSOR_MODULES, 'IMUSensor')
        imu_sensor_class(prim_path=path, name=path.rsplit('/', 1)[-1], translation=(0.0, 0.0, 0.0))
        return path
    return str(imu_class.create(path).paths[0])


def enable_extension(name_candidates: tuple[str, ...]) -> str:
    """Enable the first extension that exists. Returns the id that worked."""
    from omni.kit.app import get_app

    manager = get_app().get_extension_manager()
    tried = []

    for name in name_candidates:
        try:
            manager.set_extension_enabled_immediate(name, True)
        except Exception as exc:  # noqa: BLE001 - the API raises bare Exception
            tried.append(f'{name}: {exc}')
            continue
        if manager.is_extension_enabled(name):
            logger.info('enabled extension %s', name)
            return name
        tried.append(f'{name}: enable returned but extension is not active')

    raise IsaacCompatError('could not enable any of these extensions:\n  ' + '\n  '.join(tried))


def og_node_type(name: str, prefixes: tuple[str, ...] = OG_PREFIXES) -> str:
    """Qualify an OmniGraph node type against the naming in use.

    Which prefix is correct depends on which bridge extension loaded, so this
    resolves lazily against `active_og_prefix`, set by enable_ros2_bridge().
    """
    return f'{_active_og_prefix or prefixes[0]}.{name}'


_active_og_prefix: str | None = None
_active_core_prefix: str | None = None


def enable_ros2_bridge() -> str:
    """Enable the ROS 2 bridge and remember which naming scheme it uses."""
    global _active_og_prefix, _active_core_prefix

    extension = enable_extension(ROS2_BRIDGE_EXTENSIONS)
    # The OmniGraph node prefix tracks the extension id.
    _active_og_prefix = extension
    _active_core_prefix = (
        'isaacsim.core.nodes' if extension.startswith('isaacsim') else 'omni.isaac.core_nodes'
    )
    return extension


def core_node_type(name: str) -> str:
    """Qualify a non-ROS Isaac OmniGraph node type (clock reads, odometry)."""
    return f'{_active_core_prefix or OG_CORE_PREFIXES[0]}.{name}'


def sensor_node_type(name: str) -> str:
    """Qualify a physics-sensor OmniGraph node type (IsaacReadIMU)."""
    legacy = _active_core_prefix is not None and not _active_core_prefix.startswith('isaacsim')
    return f'{OG_SENSOR_PREFIXES[1] if legacy else OG_SENSOR_PREFIXES[0]}.{name}'


def isaac_version() -> str:
    """Best-effort version string, logged at startup.

    Worth the effort: the first question about any Isaac bug is which version
    produced it, and the answer is otherwise buried.
    """
    for module_name in ('isaacsim.core.version', 'omni.isaac.version'):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        getter = getattr(module, 'get_version', None)
        if getter is None:
            continue
        try:
            return str(getter())
        except Exception:  # noqa: BLE001
            continue
    return 'unknown'

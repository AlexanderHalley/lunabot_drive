# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Put the sand model (soil.py) under the wheels in Isaac.

Two halves:

  1. make_wheels_frictionless(), before the sim starts: the wheel colliders
     get a zero-friction material that wins every friction combine, so PhysX
     holds the rover UP and nothing else. Everything else on the stage --
     chassis, boulders, ground -- keeps its friction, so a boulder pushed by
     the chassis still drags on the ground.
  2. SandTraction, after world.reset(): every physics step, read the wheel
     bodies' poses and velocities and each wheel's normal force on the
     ground, ask soil.wheel_forces() what sand would do, and apply that at
     each contact point. The normal forces are what make the drop-centre
     six-wheeler pivot: its middle axle carries more than an even share.

The wheels also lose their grip on boulders: climbing a rock is not something
this model knows how to do. Flat ground only, like terrain.py.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from lunabot_sim.robot.articulation import WHEEL_JOINTS
from lunabot_sim.robot.soil import SoilConfig, wheel_forces

logger = logging.getLogger(__name__)

WHEEL_LINKS = tuple(j.replace('_joint', '_link') for j in WHEEL_JOINTS)
CALLBACK = 'lunabot_sand_traction'
LOAD_SMOOTHING = 0.5
"""Weight of the newest contact reading. Contact forces jitter step to step;
the sand model's stability caps scale with load, so jitter becomes buzz."""


def wheel_radius(urdf_path: Path) -> float:
    """The wheel collider's radius, from the expanded URDF."""
    root = ET.parse(urdf_path).getroot()
    for link in root.iter('link'):
        if link.get('name') == WHEEL_LINKS[0]:
            cylinder = link.find('collision/geometry/cylinder')
            if cylinder is not None:
                return float(cylinder.get('radius'))
    raise RuntimeError(f'no collision cylinder on {WHEEL_LINKS[0]} in {urdf_path}')


def make_wheels_frictionless(robot_path: str, config: SoilConfig | None = None) -> list[str]:
    """Bind a zero-friction, compliant physics material over every wheel link.

    Compliant: the contact is a spring and damper (SoilConfig sinkage), so
    the wheels sink into the ground a little as they would into sand.
    """
    config = config or SoilConfig()
    import omni.usd
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    material_path = f'{robot_path}/SandTractionWheelMaterial'
    material = UsdShade.Material.Define(stage, material_path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr().Set(0.0)
    physics.CreateDynamicFrictionAttr().Set(0.0)
    physics.CreateRestitutionAttr().Set(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    # 'min' outranks the default 'average', so the wheel's zero wins against
    # the ground's 0.7. (PhysX priority: average < min < multiply < max.)
    physx.CreateFrictionCombineModeAttr().Set('min')
    physx.CreateCompliantContactStiffnessAttr().Set(config.sinkage_stiffness)
    physx.CreateCompliantContactDampingAttr().Set(config.sinkage_damping)

    wheels = _find_wheel_links(stage, robot_path)
    for prim in wheels:
        # On the link, stronger than descendants: the colliders under it may
        # be instance proxies, which cannot carry a binding of their own.
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, 'physics'
        )
        # So SandTraction can read each wheel's load on the ground. Must be
        # on the stage before the simulation starts.
        report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        report.CreateThresholdAttr().Set(0.0)
    paths = [str(p.GetPath()) for p in wheels]
    logger.info('wheel colliders frictionless (%s); sand model drives them', material_path)
    return paths


class SandTraction:
    """Applies soil.wheel_forces() to the wheels every physics step."""

    def __init__(self, world, articulation_path: str, wheel_paths: list[str], radius: float,
                 config: SoilConfig | None = None):
        self._config = config or SoilConfig()
        self._radius = radius
        self._articulation_path = articulation_path
        self._wheel_paths = list(wheel_paths)
        self._wheels = None
        self._contacts = None
        self._loads = None
        world.add_physics_callback(CALLBACK, self.step)

    def _attach(self) -> bool:
        """Build the views on Isaac's own simulation view, once it exists.

        A view of our own fails ("no valid attached USD stage id"); the
        SimulationManager's is created at the first play and shared.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        view = SimulationManager.get_physics_sim_view()
        if view is None:
            return False
        self._wheels = view.create_rigid_body_view(self._wheel_paths)
        if self._wheels.count != len(WHEEL_LINKS):
            raise RuntimeError(
                f'sand traction found {self._wheels.count} of {len(WHEEL_LINKS)} wheel bodies '
                f'at {self._wheel_paths}'
            )
        self._torch = SimulationManager.get_backend() == 'torch'
        self._device = SimulationManager.get_physics_sim_device()
        self._mass, self._yaw_inertia = _rover_inertia(view, self._articulation_path)
        try:
            # Net contact force per wheel, no filter: the ground is a static
            # plane collider, not a rigid body, and a ground-only filter on it
            # reads zero on every wheel.
            self._contacts = view.create_rigid_contact_view(self._wheel_paths)
        except Exception as exc:  # noqa: BLE001
            # Still drives, on an even load split -- which a drop-centre
            # six-wheeler is NOT. Loud, because pivots will look wrong.
            logger.warning('no wheel contact forces (%s): splitting load evenly', exc)
        self._indices = self._out(np.arange(self._wheels.count, dtype=np.int32))
        logger.info(
            'sand traction on %d wheels (%s): rover %.1f kg, yaw inertia %.2f kg m^2, %s',
            self._wheels.count, self._device, self._mass, self._yaw_inertia, self._config,
        )
        return True

    def step(self, dt: float) -> None:
        if self._wheels is None and not self._attach():
            return
        pose = _np(self._wheels.get_transforms()).reshape(-1, 7)
        vel = _np(self._wheels.get_velocities()).reshape(-1, 6)
        axis = _rotate(pose[:, 3:7], np.array([0.0, 1.0, 0.0]))
        loads = self._read_loads(dt)
        force, point = wheel_forces(
            self._config,
            pose[:, :3],
            axis,
            vel[:, :3],
            vel[:, 3:],
            self._radius,
            self._mass,
            self._yaw_inertia,
            dt,
            loads=loads,
        )
        self._wheels.apply_forces_and_torques_at_position(
            self._out(force), None, self._out(point), self._indices, True
        )

    def _read_loads(self, dt: float):
        """Each wheel's normal force on the ground, N, smoothed; None if unknown."""
        if self._contacts is None:
            return None
        # The VERTICAL part of each wheel's net contact force is its load: a
        # rock pushing on a wheel sideways adds none, a wheel resting on a
        # rock still carries its share. Flat-ground scope, like soil.py.
        forces = _np(self._contacts.get_net_normal_contact_forces(dt)).reshape(-1, 3)
        normal = np.maximum(forces[:, 2], 0.0)
        if self._loads is None:
            self._loads = normal
            self._steps = 0
        else:
            self._loads = LOAD_SMOOTHING * normal + (1.0 - LOAD_SMOOTHING) * self._loads
        self._steps += 1
        if self._steps == 480:  # once, after the rover has settled (~2 s)
            share = self._loads / max(self._loads.sum(), 1e-9)
            logger.info(
                'wheel load share at rest: %s',
                ', '.join(f'{p.rsplit("/", 1)[-1].replace("_wheel_link", "")} {100 * f:.0f}%'
                          for p, f in zip(self._wheel_paths, share)),
            )
        return self._loads

    def _out(self, array):
        if self._torch:
            import torch

            return torch.as_tensor(array, device=self._device)
        return array.astype(np.float32) if array.dtype.kind == 'f' else array


def _np(data):
    """A numpy float64 copy of whatever tensor type the view hands back."""
    if hasattr(data, 'cpu'):
        data = data.cpu().numpy()
    elif hasattr(data, 'numpy'):
        data = data.numpy()
    return np.asarray(data, dtype=float)


def _find_wheel_links(stage, robot_path: str):
    from pxr import Usd, UsdPhysics

    found = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path)):
        if prim.GetName() in WHEEL_LINKS and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            found[prim.GetName()] = prim
    missing = set(WHEEL_LINKS) - set(found)
    if missing:
        raise RuntimeError(f'wheel bodies not found under {robot_path}: {sorted(missing)}')
    return [found[name] for name in WHEEL_LINKS]


def _rover_inertia(view, articulation_path: str) -> tuple[float, float]:
    """Total mass, and yaw inertia about the rover's centre of mass."""
    rover = view.create_articulation_view(articulation_path)
    masses = _np(rover.get_masses()).reshape(-1)
    inertias = _np(rover.get_inertias()).reshape(-1, 3, 3)  # body frame, at the COM
    links = _np(rover.get_link_transforms()).reshape(-1, 7)
    coms = _np(rover.get_coms()).reshape(-1, 7)  # COM in the link frame

    com_world = links[:, :3] + _rotate(links[:, 3:7], coms[:, :3])
    centre = np.average(com_world[:, :2], axis=0, weights=masses)
    yaw = 0.0
    for q, inertia, m, p in zip(links[:, 3:7], inertias, masses, com_world):
        rot = _matrix(q)
        yaw += (rot @ inertia @ rot.T)[2, 2] + m * np.sum((p[:2] - centre) ** 2)
    return float(masses.sum()), float(yaw)


def _matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rotate(quats, vectors):
    """Rotate vectors by (x, y, z, w) quaternions, row by row."""
    quats = np.asarray(quats, dtype=float)
    vectors = np.broadcast_to(vectors, (len(quats), 3))
    return np.einsum('nij,nj->ni', np.array([_matrix(q) for q in quats]), vectors)

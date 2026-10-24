# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A stand-in excavator you can see: a drum on arms and a hopper, for demo videos.

    run_isaac_sim.sh --demo-excavator

There is no excavation model or driver yet (drum on a four-bar, lift and
hopper actuators in the 2027 design), so the mission's dig and dump are timed
stops. This makes them visible: the drum lowers behind the rover and spins
while /mission/state says EXCAVATING, the hopper fills, and it tips back and
empties while DEPOSITING -- onto a regolith pile that grows on the ground
behind the rover and stays there, so a second dump in the same place makes
it bigger.

VISUAL ONLY. No colliders and no mass, so the physics, the footprint and every
sensor are exactly as without it: it all sits behind the forward camera, and
stowed it stays inside the chassis outline. The PILE is different: it is in
the world, and the camera sees it like any other obstacle -- which a berm is.
The geometry is a GUESS at the design's layout, not its dimensions.
Animation() is plain Python and tested.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

logger = logging.getLogger(__name__)

STATE_TOPIC = '/mission/state'

# base_link frame, metres. base_link is at axle height, 0.15 m above ground.
ARM_PIVOT = (-0.40, 0.0, 0.10)  # rear top of the chassis
ARM_LENGTH = 0.30
DRUM_RADIUS = 0.12
DRUM_WIDTH = 0.50
HOPPER_PIVOT = (-0.30, 0.0, 0.10)  # the hopper's rear bottom edge
HOPPER_SIZE = (0.55, 0.40, 0.20)  # x forward from the pivot, y, z

# Angles in degrees about base_link's +Y. Arm: the drum sits at
# pivot + L * (-cos a, 0, sin a); 80 is folded up over the rear wheels,
# -26 puts the drum on the ground behind the rover. Hopper: negative tips
# its front up, so it empties backwards.
ARM_STOWED, ARM_DIGGING = 80.0, -26.0
HOPPER_FLAT, HOPPER_DUMPING = 0.0, -50.0
ARM_RATE = 45.0  # deg/s
HOPPER_RATE = 35.0
DRUM_SPIN = 240.0  # deg/s while digging
FILL_RATE = 0.12  # hopper fraction per second while the drum is down
EMPTY_RATE = 0.5  # per second once the hopper is tipped past EMPTY_FROM
EMPTY_FROM = -35.0
# Regolith per full hopper (m^3): 80% of the hopper's inside.
LOAD_VOLUME = 0.8 * HOPPER_SIZE[0] * HOPPER_SIZE[1] * HOPPER_SIZE[2]
PILE_BEHIND = 0.80  # m behind base_link, where the tipped hopper pours
PILE_ASPECT = 0.4  # height / radius: a low mound
PILE_MERGE = 0.6  # m: a dump this close to an existing pile adds to it


def _towards(value: float, target: float, step: float) -> float:
    if abs(target - value) <= step:
        return target
    return value + step if target > value else value - step


@dataclass
class Animation:
    """What the stand-in is doing, stepped by the mission state."""

    arm: float = ARM_STOWED
    drum: float = 0.0
    hopper: float = HOPPER_FLAT
    load: float = 0.0  # 0 empty .. 1 full

    def step(self, mission_state: str, dt: float) -> Animation:
        digging = mission_state.startswith('EXCAVATING')
        dumping = mission_state.startswith('DEPOSITING')
        self.arm = _towards(self.arm, ARM_DIGGING if digging else ARM_STOWED, ARM_RATE * dt)
        self.hopper = _towards(
            self.hopper, HOPPER_DUMPING if dumping else HOPPER_FLAT, HOPPER_RATE * dt
        )
        if digging and self.arm == ARM_DIGGING:
            self.drum = (self.drum + DRUM_SPIN * dt) % 360.0
            self.load = min(1.0, self.load + FILL_RATE * dt)
        if self.hopper <= EMPTY_FROM:
            self.load = max(0.0, self.load - EMPTY_RATE * dt)
        return self


def pile_size(volume: float) -> tuple[float, float]:
    """Return (radius, height) of a half-ellipsoid mound holding `volume` m^3."""
    radius = (max(volume, 0.0) / ((2.0 / 3.0) * math.pi * PILE_ASPECT)) ** (1.0 / 3.0)
    return radius, PILE_ASPECT * radius


class DemoExcavator:
    """Builds the stand-in under the rover's base_link; call step() every loop."""

    def __init__(self, base_link_path: str, domain_id: int):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._anim = Animation()
        self._state = ''
        self._last = None
        self._ops = _build(f'{base_link_path}/DemoExcavator')
        self._articulation_path = base_link_path
        self._view = None
        self._piles = []  # [x, y, volume, scale op]
        self._pile = None  # the pile the current dump is pouring onto
        if not rclpy.ok():
            rclpy.init(domain_id=domain_id)
        self._node = Node('sim_demo_excavator')
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._node.create_subscription(
            String, STATE_TOPIC, lambda m: setattr(self, '_state', m.data), latched
        )
        logger.info('demo excavator stand-in under %s, following %s', base_link_path, STATE_TOPIC)

    def step(self, sim_time: float) -> None:
        import rclpy

        if rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.0)
        dt = 0.0 if self._last is None else max(0.0, sim_time - self._last)
        self._last = sim_time
        before = self._anim.load
        a = self._anim.step(self._state, dt)
        if a.load < before:
            self._pour(before - a.load)
        elif self._pile is not None and not self._state.startswith('DEPOSITING'):
            self._pile = None
        arm, drum, hopper, load = self._ops
        arm.Set(a.arm)
        drum.Set(a.drum)
        hopper.Set(a.hopper)
        load.Set((1.0, 1.0, max(a.load, 1e-3)))

    def _pour(self, fraction: float) -> None:
        """Add `fraction` of a hopper to the pile behind the rover."""
        if self._pile is None:
            x, y, yaw = self._rover_pose()
            x, y = x - PILE_BEHIND * math.cos(yaw), y - PILE_BEHIND * math.sin(yaw)
            near = [p for p in self._piles if math.hypot(p[0] - x, p[1] - y) < PILE_MERGE]
            self._pile = near[0] if near else self._new_pile(x, y)
        self._pile[2] += fraction * LOAD_VOLUME
        r, h = pile_size(self._pile[2])
        self._pile[3].Set((r, r, h))

    def _rover_pose(self):
        if self._view is None:
            from isaacsim.core.simulation_manager import SimulationManager

            self._view = SimulationManager.get_physics_sim_view().create_articulation_view(
                self._articulation_path
            )
        pose = self._view.get_root_transforms()
        pose = pose.cpu().numpy() if hasattr(pose, 'cpu') else pose
        x, y, _, qx, qy, qz, qw = (float(v) for v in pose.reshape(-1)[:7])
        return x, y, math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _new_pile(self, x: float, y: float):
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        stage = omni.usd.get_context().get_stage()
        path = f'/World/DemoPiles/Pile_{len(self._piles):02d}'
        mat = UsdShade.Material.Define(stage, f'{path}/Look')
        shader = UsdShade.Shader.Define(stage, f'{path}/Look/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.45, 0.40, 0.34)
        )
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.9)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        # A unit sphere, half below the floor, scaled into a mound.
        mound = UsdGeom.Sphere.Define(stage, f'{path}/Mound')
        mound.GetRadiusAttr().Set(1.0)
        mound.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        scale = mound.AddScaleOp()
        scale.Set(Gf.Vec3f(1e-3, 1e-3, 1e-3))
        UsdShade.MaterialBindingAPI.Apply(mound.GetPrim()).Bind(mat)
        pile = [x, y, 0.0, scale]
        self._piles.append(pile)
        logger.info('regolith pile at (%.2f, %.2f)', x, y)
        return pile


def _build(root: str):
    """Create the prims; return the (arm, drum, hopper, load-scale) ops to animate."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    stage = omni.usd.get_context().get_stage()
    materials = {}

    def material(name, rgb):
        if name not in materials:
            path = f'{root}/Looks/{name}'
            mat = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, f'{path}/Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.6)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            materials[name] = mat
        return materials[name]

    def xform(path, translate=(0.0, 0.0, 0.0)):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*translate))
        return x

    def box(path, centre, size, colour, rotate_y=0.0):
        # Op order translate, rotate, scale: USD applies the LAST op first,
        # so the box is sized, then turned, then placed.
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddRotateYOp().Set(rotate_y)
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material(*colour))
        return cube

    yellow = ('Arm', (0.85, 0.65, 0.10))
    steel = ('Drum', (0.30, 0.30, 0.32))
    orange = ('Hopper', (0.80, 0.40, 0.10))
    regolith = ('Regolith', (0.55, 0.52, 0.48))

    xform(root)
    # ---- Drum on two arms ----
    pivot = xform(f'{root}/ArmPivot', ARM_PIVOT)
    arm_op = pivot.AddRotateYOp()
    for side, y in (('L', 0.27), ('R', -0.27)):
        box(
            f'{root}/ArmPivot/Arm{side}',
            (-ARM_LENGTH / 2, y, 0.0),
            (ARM_LENGTH, 0.03, 0.04),
            yellow,
        )
    hub = xform(f'{root}/ArmPivot/Drum', (-ARM_LENGTH, 0.0, 0.0))
    drum_op = hub.AddRotateYOp()
    drum = UsdGeom.Cylinder.Define(stage, f'{root}/ArmPivot/Drum/Body')
    drum.GetAxisAttr().Set('Y')
    drum.GetRadiusAttr().Set(DRUM_RADIUS * 0.85)
    drum.GetHeightAttr().Set(DRUM_WIDTH)
    UsdShade.MaterialBindingAPI.Apply(drum.GetPrim()).Bind(material(*steel))
    # Buckets round the drum, so the spin shows.
    for i in range(6):
        a = i * math.pi / 3
        r = DRUM_RADIUS * 0.9
        box(
            f'{root}/ArmPivot/Drum/Bucket{i}',
            (r * math.cos(a), 0.0, r * math.sin(a)),
            (0.05, DRUM_WIDTH * 0.95, 0.05),
            yellow,
            rotate_y=-math.degrees(a),
        )
    # ---- Hopper: open box on the chassis ----
    hop = xform(f'{root}/HopperPivot', HOPPER_PIVOT)
    hopper_op = hop.AddRotateYOp()
    lx, ly, lz = HOPPER_SIZE
    t = 0.015
    walls = {
        'Floor': ((lx / 2, 0.0, t / 2), (lx, ly, t)),
        'Front': ((lx - t / 2, 0.0, lz / 2), (t, ly, lz)),
        'Back': ((t / 2, 0.0, lz / 2), (t, ly, lz)),
        'Left': ((lx / 2, ly / 2 - t / 2, lz / 2), (lx, t, lz)),
        'Right': ((lx / 2, -ly / 2 + t / 2, lz / 2), (lx, t, lz)),
    }
    for name, (centre, size) in walls.items():
        box(f'{root}/HopperPivot/{name}', centre, size, orange)
    # The load grows upwards from the floor: scaled in z about the floor.
    fill = xform(f'{root}/HopperPivot/Fill', (lx / 2, 0.0, t))
    load_op = fill.AddScaleOp()
    box(
        f'{root}/HopperPivot/Fill/Regolith',
        (0.0, 0.0, (lz - 2 * t) / 2),
        (lx - 2 * t, ly - 2 * t, lz - 2 * t),
        regolith,
    )
    return arm_op, drum_op, hopper_op, load_op

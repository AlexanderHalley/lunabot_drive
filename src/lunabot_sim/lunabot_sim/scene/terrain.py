"""Lunar terrain.

Phase 1 is a flat ground plane with a regolith physics material. That is
deliberate scope, not laziness: the friction coefficient is what determines
whether a skid-steer rover behaves like a skid-steer rover, and a displaced
noise mesh adds cost without changing that.

Craters are OUT OF SCOPE for now, per the 2027 plan -- but `build()` takes a
`features` list from the start so a crater depression drops in later without
restructuring this module or its caller. That empty list is the entire cost of
keeping the door open, and it mirrors the three-way ground split in
lunabot_perception for the same reason.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class TerrainFeature:
    """A local deformation of the ground plane.

    Nothing produces these yet. The type exists so the interface that will
    carry craters is already the interface that carries nothing.
    """

    kind: str
    """'crater' or 'mound'."""

    centre: tuple[float, float]
    diameter: float
    depth: float
    """Positive into the ground for a crater, negative for a mound."""


@dataclass
class TerrainConfig:
    size: float = 20.0
    """Ground plane extent, metres. Comfortably larger than the arena so the
    rover cannot drive off the edge of the world during a debugging session."""

    # ==================== Regolith friction ====================
    # The most consequential numbers in the whole sim.
    #
    # Skid-steer turning IS controlled friction failure -- the wheels scrub
    # sideways, and how much they scrub is what makes a commanded yaw rate
    # differ from the achieved one. Get these wrong and sim will happily
    # validate a wheel_separation_multiplier that is nothing like the real
    # one.
    #
    # PLACEHOLDERS. Lunar regolith simulant is usually quoted around 0.6-0.8
    # static. Calibrate against measured turning on the real rover: command a
    # known yaw rate on both, compare.
    static_friction: float = 0.7
    dynamic_friction: float = 0.6
    restitution: float = 0.01
    """Near zero. Regolith does not bounce."""

    # ==================== Appearance ====================
    # Mid-grey and very rough. Combined with the low sun angle in lighting.py
    # this is what produces the harsh, high-contrast look that actually
    # matters -- not for realism, but because it is what defeats naive vision.
    albedo: tuple[float, float, float] = (0.35, 0.34, 0.33)
    roughness: float = 0.95
    metallic: float = 0.0

    features: list[TerrainFeature] = field(default_factory=list)


def build(world, config: TerrainConfig, prim_path: str = '/World/Terrain'):
    """Create the ground plane. Imports Isaac lazily so this module stays
    importable in CI."""
    import numpy as np
    from isaacsim.core.api.materials import PhysicsMaterial, PreviewSurface
    from isaacsim.core.api.objects import GroundPlane

    physics_material = PhysicsMaterial(
        prim_path=f'{prim_path}/PhysicsMaterial',
        static_friction=config.static_friction,
        dynamic_friction=config.dynamic_friction,
        restitution=config.restitution,
    )

    visual_material = PreviewSurface(
        prim_path=f'{prim_path}/VisualMaterial',
        color=np.array(config.albedo),
        roughness=config.roughness,
        metallic=config.metallic,
    )

    ground = GroundPlane(
        prim_path=prim_path,
        name='terrain',
        size=config.size,
        physics_material=physics_material,
        visual_material=visual_material,
    )
    world.scene.add(ground)

    if config.features:
        # Reached only once craters exist. Raising rather than silently
        # ignoring them: a scene that quietly lacks the features it was
        # configured with is worse than one that refuses to start.
        raise NotImplementedError(
            f'{len(config.features)} terrain features requested, but displacement is not '
            'implemented yet. Craters are out of scope for the 2027 skeleton -- see '
            'docs/SIM_ISAAC.md.'
        )

    return ground

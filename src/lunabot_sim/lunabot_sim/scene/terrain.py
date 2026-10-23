# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

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

import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


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

    # ==================== Surface texture ====================
    # Regolith is granular: grains, clumps, lighter and darker patches. An
    # untextured floor gives a camera nothing to match, so visual loop
    # closure and visual odometry fail and the wheel odometry's slip drift is
    # never corrected -- the sim was harder than the real arena in the one way
    # that matters most to SLAM. See regolith_texture().
    texture: bool = True
    texture_tile_m: float = 8.192
    """One texture covers this many metres. Large on purpose: a small tile
    REPEATS across the arena, and repeated patches are exactly what makes
    loop closure match the wrong place. 8.192 m covers a UCF lane uniquely."""

    texture_px: int = 4096
    """Pixels across the tile: 2 mm per pixel, finer than the camera sees."""

    texture_contrast: float = 0.45
    """Albedo varies by about +-this fraction around `albedo`."""

    features: list[TerrainFeature] = field(default_factory=list)


def regolith_texture(seed: int, size: int = 4096, contrast: float = 0.45, albedo: float = 0.35):
    """Greyscale regolith, uint8 (size x size), seamless, deterministic in `seed`.

    Band-limited noise synthesised in the frequency domain (so it tiles
    seamlessly) at three scales -- grains (a few mm), clumps (cm), patches
    (tens of cm) -- plus sparse light and dark pebbles. Values are sRGB-ish
    grey levels centred on `albedo`.
    """
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(size)
    radius = np.hypot(f[:, None], f[None, :]) * size  # cycles per tile
    spectrum = np.fft.fft2(rng.standard_normal((size, size)))

    def band(cycles, width):
        return np.exp(-0.5 * ((np.log(np.maximum(radius, 1e-9)) - np.log(cycles)) / width) ** 2)

    # cycles per tile: grains ~4 mm, clumps ~3 cm, patches ~40 cm (8.2 m tile)
    # Weighted toward the coarse bands: at camera distance (2-6 mm per
    # camera pixel) grains average out, and features come from clumps and
    # patches. Each band's weight is per unit of its own energy.
    weights = (
        band(size / 2.0, 0.5) * 0.15
        + band(size / 15.0, 0.6) * 1.0 * 4.0
        + band(size / 200.0, 0.7) * 1.0 * 30.0
    )
    noise = np.real(np.fft.ifft2(spectrum * weights))
    noise /= max(noise.std(), 1e-9)

    # Sparse pebbles: a few per square decimetre, light or dark.
    pebbles = rng.random((size, size)) < 0.002
    noise[pebbles] += rng.choice([-2.5, 2.5], size=int(pebbles.sum()))

    level = albedo * (1.0 + contrast * np.tanh(noise / 2.0))
    return np.clip(level * 255.0, 0, 255).astype(np.uint8)


def write_png(path: Path, grey: np.ndarray) -> Path:
    """Write an 8-bit greyscale PNG with nothing but zlib (no Pillow in Isaac)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    height, width = grey.shape
    raw = b''.join(b'\x00' + grey[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack('>I', len(data)) + kind + data + struct.pack(
            '>I', zlib.crc32(kind + data) & 0xFFFFFFFF
        )

    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    path.write_bytes(
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', header)
        + chunk(b'IDAT', zlib.compress(raw, 6))
        + chunk(b'IEND', b'')
    )
    return path


def build(
    world,
    config: TerrainConfig,
    prim_path: str = '/World/Terrain',
    texture_dir: Path | None = None,
    seed: int = 0,
    origin: tuple[float, float] = (0.0, 0.0),
):
    """Create the ground plane.

    Imports Isaac lazily so this module stays importable in CI.
    """
    import numpy as np
    from isaacsim.core.api.materials import PhysicsMaterial, PreviewSurface
    from isaacsim.core.api.objects import GroundPlane

    # Materials go BESIDE the ground prim, not under it. Creating a child first
    # makes prim_path exist, and GroundPlane then takes it for an existing
    # ground, looks for its Plane child, finds none, and fails on a None path.
    physics_material = PhysicsMaterial(
        prim_path=f'{prim_path}Materials/PhysicsMaterial',
        static_friction=config.static_friction,
        dynamic_friction=config.dynamic_friction,
        restitution=config.restitution,
    )

    visual_material = PreviewSurface(
        prim_path=f'{prim_path}Materials/VisualMaterial',
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

    if config.texture:
        _texture_floor(prim_path, config, texture_dir, seed, origin)

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


def _texture_floor(prim_path, config, texture_dir, seed, origin):
    """Lay a textured visual floor over the ground plane; physics unchanged.

    The GroundPlane's own mesh is hidden rather than re-materialed: it has no
    texture coordinates. The new quad has them, with one texture tile per
    `texture_tile_m`, its first tile starting at `origin` (the arena corner)
    so the arena itself never sees a repeat.
    """
    import logging

    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    logger = logging.getLogger(__name__)
    texture_dir = Path(texture_dir or Path.home() / '.cache' / 'lunabot_sim')
    png = texture_dir / f'regolith_seed{seed}_{config.texture_px}.png'
    if not png.exists():
        write_png(
            png,
            regolith_texture(seed, config.texture_px, config.texture_contrast, config.albedo[0]),
        )
        logger.info('wrote regolith texture %s', png)

    stage = omni.usd.get_context().get_stage()
    geom = stage.GetPrimAtPath(f'{prim_path}/geom')
    if geom and geom.IsValid():
        UsdGeom.Imageable(geom).MakeInvisible()

    half = config.size / 2.0
    mesh = UsdGeom.Mesh.Define(stage, f'{prim_path}Visual/Floor')
    mesh.CreatePointsAttr([(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0.0, 0.0, 1.0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        'st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    tile = config.texture_tile_m
    ox, oy = origin
    st.Set(Vt.Vec2fArray([Gf.Vec2f((x - ox) / tile, (y - oy) / tile) for x, y in
                          ((-half, -half), (half, -half), (half, half), (-half, half))]))

    material = UsdShade.Material.Define(stage, f'{prim_path}Visual/Regolith')
    shader = UsdShade.Shader.Define(stage, f'{prim_path}Visual/Regolith/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(config.roughness)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(config.metallic)
    reader = UsdShade.Shader.Define(stage, f'{prim_path}Visual/Regolith/St')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    texture = UsdShade.Shader.Define(stage, f'{prim_path}Visual/Regolith/Texture')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(str(png))
    texture.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
    texture.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('raw')
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), 'result'
    )
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), 'rgb'
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


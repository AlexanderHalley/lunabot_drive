"""Lunar lighting.

The visually important part of the sim, and the part that actually breaks
perception algorithms -- which is exactly why it belongs in the skeleton
rather than being polish added later.

Three things make lighting lunar:

  1. A single hard directional source at a LOW elevation. Long shadows, sharp
     edges.
  2. Effectively NO ambient fill. On an airless body there is no atmospheric
     scattering, so shadows are genuinely black rather than dim. Stereo
     matching gets nothing at all from those regions -- a boulder's shadow is
     a hole in the point cloud, not a dark patch in it.
  3. High contrast between lit and unlit surfaces, well beyond what
     auto-exposure handles gracefully.

A detector tuned under a default Isaac dome light will look excellent and then
fail completely in the arena. Tuning it under this is the point.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LightingConfig:
    sun_elevation_deg: float = 8.0
    """Degrees above the horizon. Low sun angles are characteristic of the
    lunar poles, where these competitions are notionally set, and they produce
    the long shadows that hide obstacles."""

    sun_azimuth_deg: float = 135.0
    """Compass bearing of the sun. Worth varying between runs: a detector that
    only works with the sun behind the rover is not a detector."""

    sun_intensity: float = 3000.0

    sun_angular_diameter_deg: float = 0.53
    """The sun's real angular size seen from the Moon. Small means SHARP
    shadow edges -- a larger value gives soft penumbras that do not exist
    there and that quietly make the scene easier."""

    ambient_intensity: float = 5.0
    """Near zero, not zero. True zero makes shadowed regions render as pure
    black, which is physically right and makes the viewport unusable for a
    human trying to debug. This is a small concession to the operator, and it
    is small enough not to help the algorithms."""

    ambient_colour: tuple[float, float, float] = (0.05, 0.05, 0.06)


def build(config: LightingConfig, prim_path: str = '/World/Lighting'):
    """Create the sun and a minimal ambient fill."""
    from pxr import Gf, UsdLux
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    sun = UsdLux.DistantLight.Define(stage, f'{prim_path}/Sun')
    sun.CreateIntensityAttr(config.sun_intensity)
    sun.CreateAngleAttr(config.sun_angular_diameter_deg)
    # Slightly warm white. The sun is not tinted, but a dead-neutral render
    # reads as synthetic to a human eye judging the scene.
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))

    # A DistantLight points down -Z by default. Pitch up from straight down to
    # the sun's elevation, then yaw to its azimuth. USD rotation ops take
    # degrees, so the config values go in unconverted.
    xform = sun.AddRotateXYZOp()
    xform.Set(
        Gf.Vec3f(
            float(-(90.0 - config.sun_elevation_deg)),
            0.0,
            float(config.sun_azimuth_deg),
        )
    )

    dome = UsdLux.DomeLight.Define(stage, f'{prim_path}/Ambient')
    dome.CreateIntensityAttr(config.ambient_intensity)
    dome.CreateColorAttr(Gf.Vec3f(*config.ambient_colour))

    return sun, dome

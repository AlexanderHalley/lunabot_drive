# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Seeded boulder scattering.

Deliberately free of any Isaac import. Everything here is numpy and dataclasses,
so `test_scene_layout.py` runs in ordinary CI on a machine with no GPU and no
Omniverse install. Placing rocks is arithmetic; only *drawing* them needs a
simulator.

The layout is a pure function of the seed. Same seed, same scene, every time --
which is what makes "did that change help?" answerable at all. A perception
change evaluated against a different random scene tells you nothing.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from lunabot_sim.scene.layout import Rect


@dataclass(frozen=True)
class Boulder:
    """One rock, in world coordinates (X forward, Y left, Z up, ground at 0)."""

    position: tuple[float, float, float]
    """Centre of the boulder. z is half its height, so it sits ON the ground."""

    dimensions: tuple[float, float, float]
    """Full extent, matching vision_msgs BoundingBox3D.size."""

    yaw: float
    """Rotation about Z, radians."""

    shape: str
    """'cube' or 'sphere'. Primitives, not meshes -- a rock's contact behaviour
    is a friction question, not a geometry one, and convex meshes cost the
    solver dearly for visual detail nobody measures."""

    static: bool
    """True means an immovable collider. Large rocks are static because a rover
    nudging a 40 kg boulder across the arena is not the behaviour under test;
    small ones stay dynamic so the drivetrain can be seen to shove them."""

    mass: float
    """kg. Ignored when static."""


@dataclass
class ArenaConfig:
    """Bounds and exclusions for scattering. Metres.

    PLACEHOLDER dimensions -- roughly a Lunabotics-scale arena. Replace with
    the real competition spec when it is published.
    """

    x_min: float = 1.0
    x_max: float = 6.0
    y_min: float = -2.0
    y_max: float = 2.0

    start_zone_x: float = 1.5
    """Rocks are excluded from x < this, so the rover does not spawn inside one."""

    min_separation: float = 0.6
    """Centre-to-centre minimum. Rocks closer than the detector's cluster
    tolerance merge into one detection, which makes the detector look broken
    when it is behaving exactly as documented."""

    count: int = 12
    diameter_min: float = 0.10
    diameter_max: float = 0.45

    footprint_variation: float = 0.3
    """Each horizontal axis is scaled by 1 +- up to this, so rocks are not
    round. 0 keeps the footprint exactly `diameter` across."""

    height_variation: float = 0.3
    """The height is diameter * (1 +- up to this)."""

    density: float = 1500.0
    """kg/m^3. Basalt is nearer 2900; this is lower because the shapes are
    boxes approximating irregular rocks and the volume is overestimated."""

    static_above_diameter: float = 0.30
    """Rocks at least this big are immovable."""

    max_placement_attempts: int = 200
    """Rejection sampling gives up eventually rather than looping forever when
    count and min_separation are jointly impossible."""

    keep_outs: tuple[Rect, ...] = ()
    """No rock overlaps these: the start zone (the rover spawns there), the
    construction zone (the guidebook puts no obstacles in it), posts and the
    column. Empty for the legacy open scene, which has start_zone_x instead."""

    keep_out_clearance: float = 0.15
    """Gap kept between a rock's edge and a keep-out, metres."""

    field: Rect | None = None
    """If set, the first `count_in_field` rocks are placed inside it -- the
    guidebook's 'at least three in the obstacle zone'. The rest go anywhere."""

    count_in_field: int = 0


def scatter(config: ArenaConfig, seed: int) -> list[Boulder]:
    """Place boulders by rejection sampling. Deterministic in `seed`.

    Returns fewer than `config.count` rather than raising if the arena cannot
    hold that many at the requested separation -- a slightly sparse scene is
    more useful than a crash, and the caller logs the shortfall.
    """
    rng = np.random.default_rng(seed)
    boulders: list[Boulder] = []

    x_min = max(config.x_min, config.start_zone_x)

    for number in range(config.count):
        lo_x, hi_x, lo_y, hi_y = x_min, config.x_max, config.y_min, config.y_max
        if config.field is not None and number < config.count_in_field:
            lo_x, hi_x = max(lo_x, config.field.x_min), min(hi_x, config.field.x_max)
            lo_y, hi_y = max(lo_y, config.field.y_min), min(hi_y, config.field.y_max)
        for _attempt in range(config.max_placement_attempts):
            x = rng.uniform(lo_x, hi_x)
            y = rng.uniform(lo_y, hi_y)

            if any(
                (x - b.position[0]) ** 2 + (y - b.position[1]) ** 2 < config.min_separation**2
                for b in boulders
            ):
                continue

            diameter = rng.uniform(config.diameter_min, config.diameter_max)
            # Irregular rather than cubic: real rocks are not isotropic, and a
            # detector tuned against perfect cubes will not survive contact
            # with one that is twice as wide as it is tall.
            # One draw for all three axes, so seeds give the same scenes as
            # before these knobs existed: 1 + 0.3 * u(-1, 1) is u(0.7, 1.3).
            u = rng.uniform(-1.0, 1.0, size=3)
            variation = np.array(
                [config.footprint_variation, config.footprint_variation, config.height_variation]
            )
            extent = np.array([diameter, diameter, diameter]) * (1.0 + variation * u)

            # Half the footprint's diagonal: the rock may be yawed any way.
            reach = float(np.hypot(extent[0], extent[1])) / 2.0 + config.keep_out_clearance
            if any(zone.contains(x, y, margin=reach) for zone in config.keep_outs):
                continue

            shape = 'sphere' if rng.random() < 0.3 else 'cube'
            static = diameter >= config.static_above_diameter
            volume = float(np.prod(extent))

            boulders.append(
                Boulder(
                    # z is half the height so the rock rests on the ground
                    # plane rather than being half-buried or floating.
                    position=(float(x), float(y), float(extent[2] / 2.0)),
                    dimensions=tuple(float(v) for v in extent),
                    yaw=float(rng.uniform(0.0, 2.0 * np.pi)),
                    shape=shape,
                    static=static,
                    mass=volume * config.density,
                )
            )
            break

    return boulders


def to_ground_truth(
    boulders: list[Boulder], seed: int, config: ArenaConfig, layout: dict | None = None
) -> dict:
    """Build serialisable ground truth for scoring the detector.

    Written next to each run and published on /sim/ground_truth/boulders in
    the SAME vision_msgs/Detection3DArray type the detector emits, so scoring
    is a direct comparison rather than a format conversion. That symmetry is
    the reason this function exists at all.
    """
    truth = {
        'seed': seed,
        'arena': asdict(config),
        'count': len(boulders),
        'boulders': [asdict(b) for b in boulders],
    }
    if layout is not None:
        # Walls, zones and berm target: what a mission planner or a scorer
        # needs to know about the arena beyond its rocks.
        truth['layout'] = layout
    return truth


def write_ground_truth(
    boulders: list[Boulder],
    seed: int,
    config: ArenaConfig,
    path: Path,
    layout: dict | None = None,
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_ground_truth(boulders, seed, config, layout), indent=2))

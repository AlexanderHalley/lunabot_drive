# Copyright 2027 Lunabot. Licensed under the MIT License.

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

    density: float = 1500.0
    """kg/m^3. Basalt is nearer 2900; this is lower because the shapes are
    boxes approximating irregular rocks and the volume is overestimated."""

    static_above_diameter: float = 0.30
    """Rocks at least this big are immovable."""

    max_placement_attempts: int = 200
    """Rejection sampling gives up eventually rather than looping forever when
    count and min_separation are jointly impossible."""


def scatter(config: ArenaConfig, seed: int) -> list[Boulder]:
    """Place boulders by rejection sampling. Deterministic in `seed`.

    Returns fewer than `config.count` rather than raising if the arena cannot
    hold that many at the requested separation -- a slightly sparse scene is
    more useful than a crash, and the caller logs the shortfall.
    """
    rng = np.random.default_rng(seed)
    boulders: list[Boulder] = []

    x_min = max(config.x_min, config.start_zone_x)

    for _ in range(config.count):
        for _attempt in range(config.max_placement_attempts):
            x = rng.uniform(x_min, config.x_max)
            y = rng.uniform(config.y_min, config.y_max)

            if any(
                (x - b.position[0]) ** 2 + (y - b.position[1]) ** 2 < config.min_separation**2
                for b in boulders
            ):
                continue

            diameter = rng.uniform(config.diameter_min, config.diameter_max)
            # Irregular rather than cubic: real rocks are not isotropic, and a
            # detector tuned against perfect cubes will not survive contact
            # with one that is twice as wide as it is tall.
            extent = np.array([diameter, diameter, diameter]) * rng.uniform(0.7, 1.3, size=3)

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


def to_ground_truth(boulders: list[Boulder], seed: int, config: ArenaConfig) -> dict:
    """Build serialisable ground truth for scoring the detector.

    Written next to each run and published on /sim/ground_truth/boulders in
    the SAME vision_msgs/Detection3DArray type the detector emits, so scoring
    is a direct comparison rather than a format conversion. That symmetry is
    the reason this function exists at all.
    """
    return {
        'seed': seed,
        'arena': asdict(config),
        'count': len(boulders),
        'boulders': [asdict(b) for b in boulders],
    }


def write_ground_truth(
    boulders: list[Boulder], seed: int, config: ArenaConfig, path: Path
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_ground_truth(boulders, seed, config), indent=2))

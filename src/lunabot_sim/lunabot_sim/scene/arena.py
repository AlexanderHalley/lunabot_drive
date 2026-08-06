# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Assembles terrain, boulders and lighting into a scene.

Also spawns the boulder prims. The placement arithmetic lives in boulders.py
and is Isaac-free so it can be tested; this is the part that needs a
simulator.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

from lunabot_sim.scene import boulders as boulders_module
from lunabot_sim.scene import lighting, terrain

logger = logging.getLogger(__name__)


@dataclass
class SceneConfig:
    arena: boulders_module.ArenaConfig = field(default_factory=boulders_module.ArenaConfig)
    terrain: terrain.TerrainConfig = field(default_factory=terrain.TerrainConfig)
    lighting: lighting.LightingConfig = field(default_factory=lighting.LightingConfig)
    seed: int = 0


def build(world, config: SceneConfig, ground_truth_path: Path | None = None):
    """Build the whole scene.

    Returns the boulder list for the ground-truth publisher.
    """
    terrain.build(world, config.terrain)
    lighting.build(config.lighting)

    placed = boulders_module.scatter(config.arena, config.seed)
    if len(placed) < config.arena.count:
        # Not an error: scatter() returns short rather than looping forever
        # when count and min_separation are jointly impossible. Say so, or a
        # sparse scene looks like a bug in the detector.
        logger.warning(
            'placed %d of %d requested boulders -- arena too small for %d at %.2f m separation',
            len(placed),
            config.arena.count,
            config.arena.count,
            config.arena.min_separation,
        )

    _spawn(placed)

    if ground_truth_path is not None:
        boulders_module.write_ground_truth(placed, config.seed, config.arena, ground_truth_path)
        logger.info('ground truth written to %s', ground_truth_path)

    return placed


def _spawn(placed: list[boulders_module.Boulder]):
    """Create a prim per boulder."""
    import numpy as np
    from isaacsim.core.api.objects import (
        DynamicCuboid,
        DynamicSphere,
        FixedCuboid,
        FixedSphere,
    )
    from isaacsim.core.utils.rotations import euler_angles_to_quat

    for index, boulder in enumerate(placed):
        path = f'/World/Boulders/Boulder_{index:03d}'
        position = np.array(boulder.position)
        orientation = euler_angles_to_quat(np.array([0.0, 0.0, boulder.yaw]))

        # Grey, slightly varied. Uniform colour makes the scene read as
        # synthetic and, more importantly, gives feature-based SLAM an
        # unrealistically easy time distinguishing rock from ground.
        shade = 0.25 + (index % 5) * 0.03
        colour = np.array([shade, shade * 0.97, shade * 0.94])

        if boulder.shape == 'sphere':
            radius = float(boulder.dimensions[0] / 2.0)
            cls = FixedSphere if boulder.static else DynamicSphere
            kwargs = {'radius': radius}
        else:
            cls = FixedCuboid if boulder.static else DynamicCuboid
            kwargs = {'scale': np.array(boulder.dimensions)}

        prim_kwargs = dict(
            prim_path=path,
            name=f'boulder_{index:03d}',
            position=position,
            orientation=orientation,
            color=colour,
            **kwargs,
        )
        # Fixed prims have no mass -- passing one is an error, not a no-op.
        if not boulder.static:
            prim_kwargs['mass'] = boulder.mass

        cls(**prim_kwargs)

    logger.info(
        'spawned %d boulders (%d static)',
        len(placed),
        sum(1 for b in placed if b.static),
    )

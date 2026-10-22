# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Assembles terrain, the arena, boulders and lighting into a scene.

Also spawns the arena and boulder prims. The arithmetic lives in layout.py and
boulders.py and is Isaac-free so it can be tested; this is the part that
needs a simulator.

Walls and posts are static colliders: the camera sees them, the costmaps mark
them, and the rover cannot drive through them. Zone outlines and the berm
target are VISUAL ONLY, a few millimetres proud of the floor -- the real arena
is unmarked sand. They make the scene readable to a person, but they also
give feature-based SLAM lines that will not exist on competition day, so turn
them off (`--no-markings`) for any perception or SLAM evaluation.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

from lunabot_sim.scene import boulders as boulders_module
from lunabot_sim.scene import layout as layout_module
from lunabot_sim.scene import lighting, terrain

logger = logging.getLogger(__name__)


# Rocks stay this far inside the walls. The judges place them in the field,
# not against the walls, and a rock touching a wall is one obstacle to the
# costmap but two to a detector.
WALL_MARGIN = 0.30


@dataclass
class SceneConfig:
    arena: boulders_module.ArenaConfig = field(default_factory=boulders_module.ArenaConfig)
    terrain: terrain.TerrainConfig = field(default_factory=terrain.TerrainConfig)
    lighting: lighting.LightingConfig = field(default_factory=lighting.LightingConfig)
    layout: layout_module.ArenaLayout = layout_module.OPEN
    markings: bool = True
    seed: int = 0


def scene_config(name: str = 'default', seed: int = 0, markings: bool = True) -> SceneConfig:
    """The scene for a layout name: 'ucf' (default), 'ksc' or 'open'."""
    try:
        layout = layout_module.LAYOUTS[name]
    except KeyError:
        raise ValueError(
            f'unknown scene {name!r}; choose from {sorted(layout_module.LAYOUTS)}'
        ) from None

    arena = boulders_module.ArenaConfig()
    if layout.floor is not None:
        floor = layout.floor
        arena = boulders_module.ArenaConfig(
            x_min=floor.x_min + WALL_MARGIN,
            x_max=floor.x_max - WALL_MARGIN,
            y_min=floor.y_min + WALL_MARGIN,
            y_max=floor.y_max - WALL_MARGIN,
            # The start zone is a keep-out instead, in both axes.
            start_zone_x=floor.x_min,
            keep_outs=layout.keep_outs(),
        )
    return SceneConfig(arena=arena, layout=layout, markings=markings, seed=seed)


def build(world, config: SceneConfig, ground_truth_path: Path | None = None):
    """Build the whole scene.

    Returns the boulder list for the ground-truth publisher.
    """
    terrain.build(world, config.terrain)
    lighting.build(config.lighting)
    _build_arena(config.layout, config.markings)

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
        boulders_module.write_ground_truth(
            placed, config.seed, config.arena, ground_truth_path, layout=config.layout.to_dict()
        )
        logger.info('ground truth written to %s', ground_truth_path)

    return placed


# Plywood-ish walls, concrete posts. Colours matter only to a person looking:
# the camera's depth does not care, and RGB features come from the rocks.
WALL_COLOUR = (0.55, 0.50, 0.42)
POST_COLOUR = (0.45, 0.45, 0.45)
MARKING_COLOURS = {
    'start_zone': (0.15, 0.65, 0.25),
    'obstacle_zone': (0.85, 0.85, 0.85),
    'excavation_zone': (0.90, 0.55, 0.10),
    'construction_zone': (0.20, 0.45, 0.85),
    'berm_target': (0.85, 0.15, 0.15),
}
MARKING_WIDTH = 0.04
MARKING_HEIGHT = 0.004


def _build_arena(layout: layout_module.ArenaLayout, markings: bool, root: str = '/World/Arena'):
    """Walls, posts and (optionally) zone markings for `layout`."""
    import numpy as np
    from isaacsim.core.api.objects import FixedCuboid, FixedCylinder, VisualCuboid

    if layout.floor is None:
        logger.info('scene "%s": no arena walls', layout.name)
        return

    floor = layout.floor
    t, h = layout.wall_thickness, layout.wall_height
    length, width = floor.size
    cx, cy = floor.centre
    # Long walls run the full outside length, so the corners close.
    walls = {
        'WallStart': ((floor.x_min - t / 2, cy), (t, width + 2 * t)),
        'WallEnd': ((floor.x_max + t / 2, cy), (t, width + 2 * t)),
        'WallRight': ((cx, floor.y_min - t / 2), (length + 2 * t, t)),
        'WallLeft': ((cx, floor.y_max + t / 2), (length + 2 * t, t)),
    }
    for name, ((x, y), (sx, sy)) in walls.items():
        FixedCuboid(
            prim_path=f'{root}/{name}',
            name=f'arena_{name.lower()}',
            position=np.array([x, y, h / 2]),
            scale=np.array([sx, sy, h]),
            color=np.array(WALL_COLOUR),
        )

    for index, post in enumerate(layout.posts):
        path = f'{root}/Post_{index:02d}'
        position = np.array([post.x, post.y, post.height / 2])
        if post.round:
            FixedCylinder(
                prim_path=path,
                name=f'arena_post_{index:02d}',
                position=position,
                radius=post.size / 2,
                height=post.height,
                color=np.array(POST_COLOUR),
            )
        else:
            FixedCuboid(
                prim_path=path,
                name=f'arena_post_{index:02d}',
                position=position,
                scale=np.array([post.size, post.size, post.height]),
                color=np.array(POST_COLOUR),
            )

    if layout.fiducial_rail is not None:
        rail = layout.fiducial_rail
        VisualCuboid(
            prim_path=f'{root}/FiducialRail',
            name='arena_fiducial_rail',
            position=np.array([rail.x_min + 0.01, rail.centre[1], 0.35]),
            scale=np.array([0.02, rail.size[1], 0.10]),
            color=np.array([0.05, 0.05, 0.05]),
        )

    if markings:
        for zone, colour in MARKING_COLOURS.items():
            rect = getattr(layout, zone)
            if rect is not None:
                _outline(VisualCuboid, f'{root}/Markings/{zone}', rect, colour)

    logger.info(
        'scene "%s": %.2f x %.2f m arena, %d posts, markings %s (%s)',
        layout.name,
        length,
        width,
        len(layout.posts),
        'on' if markings else 'off',
        layout.source,
    )


def _outline(cls, path: str, rect: layout_module.Rect, colour):
    """Four thin visual strips around `rect`, just proud of the floor."""
    import numpy as np

    w, z = MARKING_WIDTH, MARKING_HEIGHT / 2 + 0.001
    (sx, sy), (cx, cy) = rect.size, rect.centre
    sides = {
        'a': ((rect.x_min, cy), (w, sy)),
        'b': ((rect.x_max, cy), (w, sy)),
        'c': ((cx, rect.y_min), (sx, w)),
        'd': ((cx, rect.y_max), (sx, w)),
    }
    for side, ((x, y), (lx, ly)) in sides.items():
        cls(
            prim_path=f'{path}_{side}',
            name=f'{path.rsplit("/", 1)[-1]}_{side}',
            position=np.array([x, y, z]),
            scale=np.array([lx, ly, MARKING_HEIGHT]),
            color=np.array(colour),
        )


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

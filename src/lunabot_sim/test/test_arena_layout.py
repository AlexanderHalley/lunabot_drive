#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Arena layouts, and the rocks scattered in them. No Isaac needed.

The layouts are hand-typed from a guidebook figure, which is exactly the kind
of data that goes quietly wrong: a zone that pokes through a wall, a start box
the rover does not fit in, a rock spawned inside the construction zone. Each
of those fails here instead of in a confusing sim run.
"""

import math

import pytest
from lunabot_sim.scene import arena
from lunabot_sim.scene.boulders import scatter
from lunabot_sim.scene.layout import LAYOUTS, UCF, Rect

WALLED = [layout for name, layout in LAYOUTS.items() if layout.floor is not None and name != 'default']

# The rover at the spawn pose: chassis + wheels, from nav2_params.yaml's
# footprint. It spawns at the origin facing +X.
ROVER = Rect(-0.46, 0.46, -0.322, 0.322)


@pytest.mark.parametrize('layout', WALLED, ids=lambda layout: layout.name)
def test_zones_are_inside_the_walls(layout):
    for name in ('start_zone', 'obstacle_zone', 'excavation_zone', 'construction_zone'):
        zone = getattr(layout, name)
        assert zone is not None, name
        assert zone.inside(layout.floor), name


@pytest.mark.parametrize('layout', WALLED, ids=lambda layout: layout.name)
def test_the_berm_target_is_in_the_construction_zone(layout):
    assert layout.berm_target.inside(layout.construction_zone)


@pytest.mark.parametrize('layout', WALLED, ids=lambda layout: layout.name)
def test_the_construction_zone_is_at_the_far_end(layout):
    """Excavate, then build: the berm is beyond the obstacle field, not in it."""
    assert layout.construction_zone.inside(layout.excavation_zone)
    assert layout.obstacle_zone.x_max <= layout.excavation_zone.x_min


@pytest.mark.parametrize('layout', WALLED, ids=lambda layout: layout.name)
def test_the_rover_spawns_inside_the_start_zone(layout):
    """The spawn pose is the origin; the whole footprint must fit, clear of walls."""
    assert ROVER.inside(layout.start_zone)
    assert ROVER.inside(layout.floor)


@pytest.mark.parametrize('layout', WALLED, ids=lambda layout: layout.name)
def test_no_post_stands_on_the_spawn(layout):
    for post in layout.posts:
        footprint = post.footprint()
        overlap = not (
            footprint.x_max < ROVER.x_min
            or footprint.x_min > ROVER.x_max
            or footprint.y_max < ROVER.y_min
            or footprint.y_min > ROVER.y_max
        )
        assert not overlap, post


def test_ucf_lane_matches_the_guidebook_figure():
    """UCF 2025 Guidebook Figure 1: 8.14 m x 4.57 m lane, zones 4.07 m long,
    construction zone 2.6 m x 2 m; Figure 4: target berm area 1.5 m x 0.9 m."""
    assert UCF.floor.size == pytest.approx((8.14, 4.57))
    assert UCF.obstacle_zone.size[0] == pytest.approx(4.07)
    assert UCF.excavation_zone.size[0] == pytest.approx(4.07)
    assert UCF.construction_zone.size == pytest.approx((2.6, 2.0))
    assert UCF.berm_target.size == pytest.approx((1.5, 0.9))


def test_the_default_scene_is_the_ucf_lane():
    assert arena.scene_config('default').layout is UCF


def test_an_unknown_scene_says_what_exists():
    with pytest.raises(ValueError, match='ucf'):
        arena.scene_config('moon')


@pytest.mark.parametrize('name', ['ucf', 'ksc'])
@pytest.mark.parametrize('seed', range(8))
def test_rocks_respect_walls_and_keep_outs(name, seed):
    config = arena.scene_config(name, seed=seed)
    layout = config.layout
    placed = scatter(config.arena, seed)
    assert placed, 'an arena with no rocks is a useless default'
    for boulder in placed:
        x, y, _ = boulder.position
        reach = math.hypot(boulder.dimensions[0], boulder.dimensions[1]) / 2.0
        assert layout.floor.contains(x, y, margin=-arena.WALL_MARGIN), boulder
        for zone in layout.keep_outs():
            assert not zone.contains(x, y, margin=reach), (zone, boulder)


def test_most_rocks_are_in_the_field_not_just_the_dig_site():
    """The obstacle zone is where navigation is tested; it must not come out empty."""
    config = arena.scene_config('ucf', seed=7)
    in_obstacle_zone = [
        b for b in scatter(config.arena, 7) if config.layout.obstacle_zone.contains(*b.position[:2])
    ]
    assert len(in_obstacle_zone) >= 3  # the guidebook's minimum


def test_ground_truth_carries_the_layout():
    config = arena.scene_config('ucf', seed=1)
    from lunabot_sim.scene.boulders import to_ground_truth

    truth = to_ground_truth([], 1, config.arena, config.layout.to_dict())
    assert truth['layout']['name'] == 'ucf'
    assert truth['layout']['berm_target']['x_min'] == UCF.berm_target.x_min


def test_the_default_lighting_is_the_indoor_arena():
    """The competition runs indoors: high, soft light with real ambient fill."""
    light = arena.scene_config('ucf').lighting
    assert light.sun_elevation_deg >= 45.0
    assert light.ambient_intensity > 100.0


def test_the_lunar_lighting_stays_harsh():
    """Kept for stress-testing perception: low sun, next to no fill."""
    light = arena.scene_config('ucf', light='lunar').lighting
    assert light.sun_elevation_deg <= 10.0
    assert light.ambient_intensity < 10.0


def test_an_unknown_lighting_says_what_exists():
    with pytest.raises(ValueError, match='arena'):
        arena.scene_config('ucf', light='disco')


# ==================== Regolith texture ====================
# Small size: the arithmetic is the same at 4096.
from lunabot_sim.scene.terrain import regolith_texture, write_png  # noqa: E402


def test_the_texture_is_deterministic_and_varies_with_seed():
    assert (regolith_texture(3, 256) == regolith_texture(3, 256)).all()
    assert (regolith_texture(3, 256) != regolith_texture(4, 256)).any()


def test_the_texture_has_contrast_around_the_albedo():
    """Flat grey gives a camera nothing to match -- the whole point."""
    grey = regolith_texture(1, 256, contrast=0.45, albedo=0.35)
    assert abs(grey.mean() / 255.0 - 0.35) < 0.05
    assert grey.std() > 8


def test_the_texture_tiles_seamlessly():
    """Frequency-domain noise wraps: the first and last rows are neighbours."""
    grey = regolith_texture(2, 256).astype(float)
    seam = abs(grey[0] - grey[-1]).mean()
    interior = abs(grey[100] - grey[101]).mean()
    assert seam < 2.0 * interior


def test_the_png_is_valid(tmp_path):
    import struct
    import zlib

    grey = regolith_texture(5, 64)
    data = write_png(tmp_path / 't.png', grey).read_bytes()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    width, height = struct.unpack('>II', data[16:24])
    assert (width, height) == (64, 64)
    start = data.index(b'IDAT') + 4
    length = struct.unpack('>I', data[start - 8 : start - 4])[0]
    raw = zlib.decompress(data[start : start + length])
    assert len(raw) == 64 * (64 + 1)


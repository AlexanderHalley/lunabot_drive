#!/usr/bin/env python3
# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Scene layout tests.

These run in ORDINARY CI -- no GPU, no Omniverse, no Isaac install -- because
lunabot_sim.scene.boulders imports nothing but numpy. Placing rocks is
arithmetic; only drawing them needs a simulator.

The property that matters most is determinism. A perception change evaluated
against a different random scene tells you nothing, so "same seed, same scene"
is not a nicety, it is what makes the sim useful for evaluation at all.
"""

import math

import pytest
from lunabot_sim.scene.boulders import ArenaConfig, Boulder, scatter, to_ground_truth


@pytest.fixture
def config():
    return ArenaConfig()


def test_same_seed_gives_an_identical_layout(config):
    """The whole reason the sim can be used to evaluate a change."""
    assert scatter(config, seed=7) == scatter(config, seed=7)


def test_different_seeds_give_different_layouts(config):
    assert scatter(config, seed=1) != scatter(config, seed=2)


def test_places_the_requested_number(config):
    # The default arena is comfortably big enough for the default count, so a
    # shortfall here means the rejection sampler is failing, not that the
    # arena is full.
    assert len(scatter(config, seed=0)) == config.count


def test_returns_short_rather_than_hanging_when_the_arena_is_full(config):
    """scatter() gives up rather than looping forever on an impossible ask."""
    impossible = ArenaConfig(count=500, min_separation=2.0)
    placed = scatter(impossible, seed=0)
    assert 0 < len(placed) < 500


def test_all_boulders_are_inside_the_arena(config):
    for boulder in scatter(config, seed=3):
        x, y, _ = boulder.position
        assert config.x_min <= x <= config.x_max, boulder
        assert config.y_min <= y <= config.y_max, boulder


def test_nothing_spawns_in_the_start_zone(config):
    """The rover spawns there.

    A boulder inside it means the rover starts intersecting a collider, which in PhysX is an
    explosion, not a collision.
    """
    for boulder in scatter(config, seed=5):
        assert boulder.position[0] >= config.start_zone_x, boulder


def test_minimum_separation_holds(config):
    """Boulders are never closer together than the configured separation.

    Rocks closer than the detector's cluster tolerance merge into one detection, which makes the
    detector look broken when it is behaving exactly as documented.
    """
    placed = scatter(config, seed=11)
    for i, a in enumerate(placed):
        for b in placed[i + 1 :]:
            distance = math.hypot(a.position[0] - b.position[0], a.position[1] - b.position[1])
            assert distance >= config.min_separation, f'{a} and {b} are {distance:.3f} m apart'


def test_boulders_rest_on_the_ground(config):
    """Z must be half the height, or rocks float or are half buried.

    A half-buried rock still produces a detection, just a smaller one, so this
    is exactly the kind of error that quietly biases every size measurement.
    """
    for boulder in scatter(config, seed=2):
        assert boulder.position[2] == pytest.approx(boulder.dimensions[2] / 2.0)


def test_dimensions_are_within_the_configured_range(config):
    # Each axis is scaled independently by 0.7-1.3 to make rocks irregular, so
    # the bound is the configured range widened by that factor.
    for boulder in scatter(config, seed=4):
        for extent in boulder.dimensions:
            assert config.diameter_min * 0.7 <= extent <= config.diameter_max * 1.3


def test_boulders_are_not_all_the_same_shape(config):
    """Uniform rocks give a detector an unrealistically easy time."""
    shapes = {b.shape for b in scatter(config, seed=6)}
    assert len(shapes) > 1


def test_large_boulders_are_static_and_small_ones_are_not(config):
    """Whether a boulder is static follows from its size.

    A rover shoving a 40 kg boulder across the arena is not the behaviour under test; a rover
    nudging a small rock is.
    """
    for boulder in scatter(config, seed=8):
        largest = max(boulder.dimensions)
        smallest = min(boulder.dimensions)
        if smallest >= config.static_above_diameter:
            assert boulder.static, boulder
        elif largest < config.static_above_diameter:
            assert not boulder.static, boulder


def test_dynamic_boulders_have_positive_mass(config):
    for boulder in scatter(config, seed=9):
        if not boulder.static:
            assert boulder.mass > 0.0


def test_yaw_is_a_real_rotation(config):
    for boulder in scatter(config, seed=10):
        assert 0.0 <= boulder.yaw <= 2 * math.pi


def test_ground_truth_round_trips(config):
    """Ground truth carries the seed and arena that produced it.

    Serialised ground truth is what the detector gets scored against, so it has to carry the seed
    and the arena that produced it -- a scoring run against the wrong scene is worse than no
    scoring run.
    """
    placed = scatter(config, seed=13)
    truth = to_ground_truth(placed, seed=13, config=config)

    assert truth['seed'] == 13
    assert truth['count'] == len(placed)
    assert len(truth['boulders']) == len(placed)

    first = truth['boulders'][0]
    assert set(first) == set(Boulder.__dataclass_fields__)
    assert tuple(first['position']) == placed[0].position

    import json

    json.dumps(truth)  # must be serialisable, not merely dataclass-shaped


def test_zero_count_is_a_valid_scene(config):
    """An empty arena is a valid scene.

    An empty arena is the right control case for "does the detector report anything when there is
    nothing there".
    """
    assert scatter(ArenaConfig(count=0), seed=0) == []

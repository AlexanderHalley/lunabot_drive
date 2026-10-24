# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavator stand-in's animation: digs, fills, dumps, empties, stows."""

import math

import pytest
from lunabot_sim.scene import demo_excavator as ex


def run(anim, state, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        anim.step(state, dt)
    return anim


def test_driving_keeps_everything_stowed_and_still():
    anim = run(ex.Animation(), 'DRIVING TO EXCAVATION ZONE', 5.0)
    assert (anim.arm, anim.drum, anim.hopper, anim.load) == (ex.ARM_STOWED, 0.0, 0.0, 0.0)


def test_an_8_s_dig_lowers_the_drum_spins_it_and_fills_the_hopper():
    anim = run(ex.Animation(), 'EXCAVATING (1/2)', 8.0)
    assert anim.arm == ex.ARM_DIGGING
    assert anim.drum != 0.0
    assert 0.5 < anim.load <= 1.0
    run(anim, 'DRIVING TO CONSTRUCTION ZONE (1/2)', 4.0)
    assert anim.arm == ex.ARM_STOWED and anim.load > 0.5  # carried, not spilled


def test_a_6_s_dump_tips_the_hopper_and_empties_it_then_it_comes_back():
    anim = ex.Animation(load=1.0)
    run(anim, 'DEPOSITING', 6.0)
    assert anim.hopper == ex.HOPPER_DUMPING and anim.load == 0.0
    run(anim, 'RETURNING TO START', 3.0)
    assert anim.hopper == ex.HOPPER_FLAT


def test_stowed_drum_stays_inside_the_chassis_outline_and_clear_of_the_tipped_hopper():
    px, _, pz = ex.ARM_PIVOT
    a = math.radians(ex.ARM_STOWED)
    drum_x = px - ex.ARM_LENGTH * math.cos(a)
    drum_z = pz + ex.ARM_LENGTH * math.sin(a)
    assert drum_x - ex.DRUM_RADIUS >= -0.46 - 0.12  # within a drum of the rear wheels
    # The hopper's back top corner, tipped fully, must not enter the drum.
    hx, _, hz = ex.HOPPER_PIVOT
    t = math.radians(ex.HOPPER_DUMPING)
    corner = (hx + ex.HOPPER_SIZE[2] * math.sin(t), hz + ex.HOPPER_SIZE[2] * math.cos(t))
    assert math.hypot(corner[0] - drum_x, corner[1] - drum_z) > ex.DRUM_RADIUS


def test_a_full_hopper_makes_a_low_mound_and_two_make_a_bigger_one():
    r1, h1 = ex.pile_size(ex.LOAD_VOLUME)
    r2, h2 = ex.pile_size(2 * ex.LOAD_VOLUME)
    assert 0.2 < r1 < 0.5 and h1 < 0.2
    assert r2 > r1 and h2 > h1
    # The mound holds what was poured: half-ellipsoid volume.
    assert (2 / 3) * math.pi * r1 * r1 * h1 == pytest.approx(ex.LOAD_VOLUME)

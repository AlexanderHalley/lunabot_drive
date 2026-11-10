# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavator stand-in's animation: digs, fills, dumps, empties, stows."""

import math

import pytest
from lunabot_sim.scene import demo_excavator as ex


def test_the_server_strokes_map_onto_the_stand_in_ends():
    anim = ex.Animation().apply({'excavator_arm': 0.0, 'excavator_hopper': 0.0}, 0.0)
    assert (anim.arm, anim.hopper, anim.load) == (ex.ARM_STOWED, ex.HOPPER_FLAT, 0.0)
    anim.apply({'excavator_arm': 1.0, 'excavator_drum': math.pi, 'excavator_hopper': 1.0}, 0.7)
    assert anim.arm == ex.ARM_DIGGING and anim.hopper == ex.HOPPER_DUMPING
    assert anim.drum == pytest.approx(180.0) and anim.load == 0.7


def test_unknown_or_missing_values_do_not_throw_the_drawing_around():
    anim = ex.Animation().apply({'excavator_arm': 0.5}, math.nan)
    arm = anim.arm
    assert ex.ARM_DIGGING < arm < ex.ARM_STOWED and anim.load == 0.0
    anim.apply({'excavator_hopper': math.nan, 'excavator_drum': math.nan}, 2.0)
    assert anim.arm == arm and anim.hopper == ex.HOPPER_FLAT and anim.load == 1.0


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
    assert 0.2 < r1 < 0.5 and h1 < 0.3
    assert r2 > r1 and h2 > h1
    # The mound holds what was poured: half-ellipsoid volume.
    assert (2 / 3) * math.pi * r1 * r1 * h1 == pytest.approx(ex.LOAD_VOLUME)

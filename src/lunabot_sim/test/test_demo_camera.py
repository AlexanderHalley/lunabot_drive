# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The demo cameras' geometry. Isaac-free: only the classes need Isaac."""

import math

import pytest

from lunabot_sim.scene import demo_camera
from lunabot_sim.scene.layout import UCF


def test_heading_smoothing_takes_the_short_way_round():
    h = demo_camera.smooth_heading(math.radians(170), math.radians(-170), dt=100.0)
    assert math.remainder(h - math.radians(-170), math.tau) == pytest.approx(0.0, abs=1e-6)
    step = demo_camera.smooth_heading(math.radians(170), math.radians(-170), dt=0.1)
    assert step > math.radians(170)  # through 180, not back through 0


def test_chase_camera_sits_behind_and_above_looking_ahead():
    eye, target = demo_camera.chase_pose(1.0, 2.0, math.pi / 2)
    assert eye == pytest.approx((1.0, 2.0 - demo_camera.CHASE_BACK, demo_camera.CHASE_UP))
    assert target[1] > 2.0 and target[2] == 0.0


@pytest.mark.parametrize('heading', [0.0, 1.0, -2.5])
def test_look_at_points_the_camera_minus_z_at_the_target(heading):
    eye, target = demo_camera.chase_pose(0.5, -0.3, heading)
    rows = demo_camera.look_at_rows(eye, target)
    direction = [t - e for t, e in zip(target, eye)]
    n = math.sqrt(sum(v * v for v in direction))
    assert [-v for v in rows[2][:3]] == pytest.approx([v / n for v in direction])
    assert rows[1][2] > 0.0  # image up is world up-ish
    assert rows[0][2] == pytest.approx(0.0)  # no roll
    for a in range(3):
        for b in range(3):
            dot = sum(rows[a][i] * rows[b][i] for i in range(3))
            assert dot == pytest.approx(1.0 if a == b else 0.0, abs=1e-9)


def test_overview_looks_into_the_arena_from_outside_it():
    eye, target = demo_camera.overview_pose(UCF.floor)
    assert UCF.floor.contains(*target[:2])
    assert eye[2] > 3.0

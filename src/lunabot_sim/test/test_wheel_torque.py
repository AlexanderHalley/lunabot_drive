# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Drive torque from the sand's force at the contact patch, and its averaging."""

import numpy as np
import pytest
from lunabot_sim.robot import traction

R = 0.15
CENTRE = np.array([[0.0, 0.0, R]] * 2)
POINT = np.array([[0.0, 0.0, 0.0]] * 2)  # contact patch, straight below the axle
AXIS = np.array([[0.0, 1.0, 0.0]] * 2)  # every wheel body's +y


def test_forward_traction_is_positive_torque_on_both_sides():
    force = np.array([[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]])  # sand pushes the rover forward
    assert traction.drive_torques(force, POINT, CENTRE, AXIS) == pytest.approx([1.5, 1.5])


def test_a_pivot_shows_as_opposite_torques_and_side_forces_as_none():
    force = np.array([[-20.0, 7.0, 0.0], [20.0, -7.0, 0.0]])  # left back, right forward
    left, right = traction.drive_torques(force, POINT, CENTRE, AXIS)
    assert left == pytest.approx(-3.0) and right == pytest.approx(3.0)


def test_the_window_averages_over_physics_time():
    w = traction.TorqueWindow(0.1)
    assert w.add([1.0, 2.0], 0.05) is None
    mean = w.add([3.0, 4.0], 0.05)
    assert mean == pytest.approx([2.0, 3.0])
    assert w.add([5.0, 5.0], 0.05) is None  # a fresh window


def test_the_drive_torque_cap_is_read_from_the_urdf(tmp_path):
    urdf = tmp_path / 'r.urdf'
    urdf.write_text(
        '<robot name="r"><joint name="front_left_wheel_joint" type="continuous">'
        '<limit effort="17.5" velocity="16"/></joint></robot>'
    )
    assert traction.wheel_effort_limit(urdf) == 17.5

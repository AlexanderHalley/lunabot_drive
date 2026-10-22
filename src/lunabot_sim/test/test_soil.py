#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Turning behaviour of the sand model, without Isaac.

The field fact these pin down: the 2026 rover could not turn on the spot in
sand at all, and drove wide arcs fine. The 2027 chassis has nearly the same
wheelbase/track ratio. A rigid planar rover with ideal velocity-held wheels is
driven by lunabot_sim.robot.soil and nothing else -- the same forces Isaac
applies, minus everything Isaac adds that is not about turning.

If the real minimum arc radius gets measured, change the 1.5 m below and
refit SoilConfig.alpha_0 until these pass again.
"""

import numpy as np
import pytest
from lunabot_sim.robot.soil import SoilConfig, wheel_forces

# 2027 design (lunabot_description properties.xacro).
WHEELBASE = 0.62
TRACK = 0.554
RADIUS = 0.15
MASS = 32.3
YAW_INERTIA = 2.6  # chassis box + wheels at the corners
DT = 1.0 / 240.0  # run_sim.py's physics step

CORNERS = np.array(
    [
        [WHEELBASE / 2, TRACK / 2],
        [WHEELBASE / 2, -TRACK / 2],
        [-WHEELBASE / 2, TRACK / 2],
        [-WHEELBASE / 2, -TRACK / 2],
    ]
)
LEFT = np.array([True, False, True, False])

ASSUMED_MIN_ARC_RADIUS = 1.5  # m, unmeasured -- see SoilConfig


# Six wheels, same wheelbase: a middle axle on the rover's centre line. With a
# drop-centre (middle axle a little lower) it carries more of the weight;
# MIDDLE_SHARE is that share, a static stand-in for what Isaac measures.
CORNERS_6 = np.array(
    [
        [WHEELBASE / 2, TRACK / 2],
        [WHEELBASE / 2, -TRACK / 2],
        [0.0, TRACK / 2],
        [0.0, -TRACK / 2],
        [-WHEELBASE / 2, TRACK / 2],
        [-WHEELBASE / 2, -TRACK / 2],
    ]
)
LEFT_6 = np.array([True, False, True, False, True, False])


def six_wheel_loads(middle_share):
    end = (1.0 - middle_share) / 4.0
    share = np.array([end, end, middle_share / 2, middle_share / 2, end, end])
    return share * MASS * 9.81


def drive(linear, angular, seconds=6.0, config=None, corners=None, left=None, loads=None):
    """Hold diff-drive wheel speeds; return the settled (speed, yaw rate)."""
    config = config or SoilConfig()
    corners = CORNERS if corners is None else corners
    left = LEFT if left is None else left
    n = len(corners)
    rim = np.where(left, linear - angular * TRACK / 2, linear + angular * TRACK / 2)

    pos = np.zeros(2)
    heading = 0.0
    vel = np.zeros(2)
    yaw_rate = 0.0
    settled = []
    steps = int(seconds / DT)
    for step in range(steps):
        c, s = np.cos(heading), np.sin(heading)
        arm = corners @ np.array([[c, -s], [s, c]]).T
        centre = np.c_[pos + arm, np.full(n, RADIUS)]
        axis = np.tile([-s, c, 0.0], (n, 1))
        v_centre = np.c_[vel + yaw_rate * np.c_[-arm[:, 1], arm[:, 0]], np.zeros(n)]
        omega = (rim / RADIUS)[:, None] * axis + [0.0, 0.0, yaw_rate]

        force, _ = wheel_forces(
            config, centre, axis, v_centre, omega, RADIUS, MASS, YAW_INERTIA, DT, loads=loads
        )
        vel = vel + force[:, :2].sum(axis=0) / MASS * DT
        torque = np.sum(arm[:, 0] * force[:, 1] - arm[:, 1] * force[:, 0])
        yaw_rate += torque / YAW_INERTIA * DT
        pos = pos + vel * DT
        heading += yaw_rate * DT
        if step > steps // 2:
            settled.append((np.linalg.norm(vel), yaw_rate))

    settled = np.array(settled)
    return settled[:, 0].mean(), settled[:, 1].mean(), settled[:, 1].std()


@pytest.mark.parametrize('angular', [0.3, 0.7, 1.0])
def test_cannot_pivot_in_place(angular):
    """The field fact. Not slowly, not at all: the wheels dig instead."""
    speed, yaw_rate, _ = drive(0.0, angular)
    assert abs(yaw_rate) < 0.01 * angular
    assert speed < 0.01


def test_tightest_arc_is_about_the_assumed_radius():
    radii = []
    for commanded in (0.4, 0.5, 0.6, 0.8, 1.0, 1.5):
        speed, yaw_rate, _ = drive(0.3, 0.3 / commanded, seconds=5.0)
        if yaw_rate > 1e-3:
            radii.append(speed / yaw_rate)
    assert min(radii) == pytest.approx(ASSUMED_MIN_ARC_RADIUS, rel=0.2)


@pytest.mark.parametrize('commanded', [2.0, 3.0])
def test_wide_arcs_turn_the_commanded_way(commanded):
    """Wider than commanded, as skid steer is, but clearly turning."""
    speed, yaw_rate, _ = drive(0.3, 0.3 / commanded)
    achieved = speed / yaw_rate
    assert yaw_rate > 0
    assert commanded < achieved < 2.5 * commanded


def test_mirrored_arc_mirrors():
    _, left, _ = drive(0.3, 0.15)
    _, right, _ = drive(0.3, -0.15)
    assert right == pytest.approx(-left, rel=0.02)


def test_straight_line_holds_speed_and_heading():
    speed, yaw_rate, _ = drive(0.3, 0.0)
    assert abs(yaw_rate) < 1e-4
    # Some slip is how sand traction works; rolling resistance costs more.
    assert 0.25 < speed < 0.3


def test_standstill_is_still():
    """The force curves are near vertical at zero slip. No buzzing."""
    speed, yaw_rate, jitter = drive(0.0, 0.0, seconds=2.0)
    assert speed < 1e-6
    assert abs(yaw_rate) < 1e-6
    assert jitter < 1e-6


def test_settled_turn_is_smooth():
    _, _, jitter = drive(0.3, 0.15)
    assert jitter < 1e-3


def test_wheel_off_the_ground_gets_no_force():
    config = SoilConfig()
    centre = np.array([[0.0, 0.0, RADIUS + 0.1]])
    force, _ = wheel_forces(
        config,
        centre,
        np.array([[0.0, 1.0, 0.0]]),
        np.array([[0.3, 0.1, 0.0]]),
        np.array([[0.0, 5.0, 0.0]]),
        RADIUS,
        MASS,
        YAW_INERTIA,
        DT,
    )
    assert np.allclose(force, 0.0)


# ==================== Six wheels, drop-centre ====================
# The 2027 design choice for turning on the spot: same 0.62 m wheelbase and
# track, plus a middle axle. Its wheels sit on the pivot axis, so they carry
# weight without adding scrub; lowered slightly, they carry more of it.


@pytest.mark.parametrize('middle_share', [1 / 3, 0.5])
def test_six_wheels_can_pivot(middle_share):
    """Even with the weight spread evenly; better with the middle loaded."""
    speed, yaw_rate, jitter = drive(
        0.0, 0.5, corners=CORNERS_6, left=LEFT_6, loads=six_wheel_loads(middle_share)
    )
    assert yaw_rate > 0.3 * 0.5
    assert speed < 0.02  # turns about its middle, does not wander
    assert jitter < 1e-2


def test_a_heavier_middle_pivots_better():
    rates = [
        drive(0.0, 0.5, corners=CORNERS_6, left=LEFT_6, loads=six_wheel_loads(share))[1]
        for share in (1 / 3, 0.5, 0.7)
    ]
    assert rates == sorted(rates)


def test_six_wheels_still_drive_straight_and_arc():
    loads = six_wheel_loads(0.5)
    speed, yaw_rate, _ = drive(0.3, 0.0, corners=CORNERS_6, left=LEFT_6, loads=loads)
    assert abs(yaw_rate) < 1e-4
    assert speed > 0.25
    speed, yaw_rate, _ = drive(0.3, 0.15, corners=CORNERS_6, left=LEFT_6, loads=loads)
    assert yaw_rate > 0.5 * 0.15


def test_loads_default_to_an_even_split():
    """Passing the even split explicitly changes nothing."""
    even = np.full(4, MASS * 9.81 / 4)
    assert drive(0.0, 0.5, loads=even)[1] == pytest.approx(drive(0.0, 0.5)[1], abs=1e-9)


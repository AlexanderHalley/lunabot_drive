#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The mission's step list, and its UCF poses against the arena zones.

Pure Python: the node imports rclpy only inside main().
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
SCRIPT = PKG / 'scripts' / 'mission.py'
CONFIG = PKG / 'config' / 'mission_ucf.yaml'

spec = importlib.util.spec_from_file_location('mission', SCRIPT)
mission = importlib.util.module_from_spec(spec)
sys.modules['mission'] = mission  # dataclasses look the module up by name
spec.loader.exec_module(mission)

PARAMS = yaml.safe_load(CONFIG.read_text())['mission']['ros__parameters']


SPOTS = mission.dump_targets(PARAMS['dump_targets'])
PARKS = [mission.park_pose(t, PARAMS['dump_reach']) for t in SPOTS]


def _plan(cycles):
    return mission.plan(
        cycles,
        PARAMS['dig_pose'],
        PARKS,
        PARAMS['home_pose'],
        PARAMS['dig_seconds'],
        PARAMS['dump_seconds'],
    )


@pytest.mark.parametrize('cycles', [1, 3])
def test_each_cycle_digs_then_dumps_and_the_mission_ends_at_home(cycles):
    steps = _plan(cycles)
    assert len(steps) == 4 * cycles + 1
    assert steps[-1].pose == tuple(PARAMS['home_pose'])
    for i in range(cycles):
        drive_dig, dig, drive_dump, dump = steps[4 * i : 4 * i + 4]
        assert drive_dig.pose == tuple(PARAMS['dig_pose'])
        assert dig.pose is None and dig.seconds > 0
        assert drive_dump.pose == PARKS[i % len(PARKS)]  # spots in turn
        assert dump.pose is None and dump.seconds > 0


def test_ucf_poses_are_in_their_zones():
    ucf = _sim_layout().UCF
    assert ucf.excavation_zone.contains(*PARAMS['dig_pose'][:2])
    for spot in SPOTS:
        assert ucf.berm_target.contains(*spot[:2])
    assert ucf.start_zone.contains(*PARAMS['home_pose'][:2])


def test_dump_targets_must_be_triples():
    with pytest.raises(ValueError):
        mission.dump_targets([1.0, 2.0])


@pytest.mark.parametrize('fix', [(0.0, 0.0, 0.0), (0.4, -0.2, 2.3), (-0.1, 0.3, -1.0)])
def test_arena_poses_go_through_the_start_fix(fix):
    """Arena poses round-trip through the start fix.

    A goal at the map origin's own arena pose is the map origin itself, and
    the arena origin seen from map lands back on it.
    """
    assert mission.to_map(fix, fix) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    x, y, yaw = mission.to_map((0.0, 0.0, 0.0), fix)
    c, s = math.cos(fix[2]), math.sin(fix[2])
    assert (fix[0] + c * x - s * y, fix[1] + s * x + c * y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert math.remainder(yaw + fix[2], math.tau) == pytest.approx(0.0, abs=1e-9)


def test_each_load_lands_inside_the_berm_target():
    """A one-load mound at each spot fits the target despite the goal checker's slack.

    Budget = the worst pour errors MEASURED in Isaac (8 pours, 2026-10-24):
    0.18 m along the pour heading, 0.33 m across it. Across is the larger:
    the goal checker latches the position, then the turn to the pour heading
    skids. Re-measure if the goal checker or the controller changes. The
    stand-in must pour where the mission thinks it does.
    """
    ex, ucf = _excavator(), _sim_layout().UCF
    assert PARAMS['dump_reach'] == pytest.approx(ex.PILE_BEHIND)
    radius, _ = ex.pile_size(ex.LOAD_VOLUME)
    for x, y, heading in SPOTS:
        across, along = 0.33, 0.18
        ex_, ey = (along, across) if abs(math.cos(heading)) > 0.5 else (across, along)
        t = ucf.berm_target
        assert t.x_min <= x - radius - ex_ and x + radius + ex_ <= t.x_max, (x, y)
        assert t.y_min <= y - radius - ey and y + radius + ey <= t.y_max, (x, y)


def test_a_parked_rover_and_its_turn_on_the_spot_clear_every_mound():
    """Parking at a spot, and the pivot that faces the pour heading, miss every mound.

    Mounds of TWO loads, so a third cycle back at the first spot still parks.
    The turning circle is the padded footprint's corner radius.
    """
    ex = _excavator()
    radius, _ = ex.pile_size(2 * ex.LOAD_VOLUME)
    nav = yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())
    costmap = nav['global_costmap']['global_costmap']['ros__parameters']
    pad = costmap['footprint_padding']
    corners = yaml.safe_load(costmap['footprint'])
    turn = max(math.hypot(abs(cx) + pad, abs(cy) + pad) for cx, cy in corners)
    for px, py, _ in PARKS:
        for sx, sy, _ in SPOTS:
            assert math.hypot(px - sx, py - sy) >= turn + radius, ((px, py), (sx, sy))


def test_parked_rover_stays_clear_of_the_walls():
    ucf = _sim_layout().UCF
    for px, py, _ in PARKS:
        for gap in (
            px - ucf.floor.x_min,
            ucf.floor.x_max - px,
            py - ucf.floor.y_min,
            ucf.floor.y_max - py,
        ):
            assert gap >= 0.35 + 0.03 + 0.25  # inscribed + padding + margin


def test_a_free_heading_goal_faces_the_way_the_rover_drives_there():
    assert mission.arrive_heading((4.0, 0.8), (6.0, 0.8)) == pytest.approx(math.pi)
    assert mission.arrive_heading((4.0, 0.8, 0.3), (6.0, 0.8)) == 0.3
    assert mission.to_map((4.0, 0.8), (1.0, 0.0, 0.0)) == pytest.approx((3.0, 0.8))


@pytest.mark.parametrize('heading', [0.0, math.pi / 2, -2.0])
def test_parking_puts_the_pour_point_on_the_target(heading):
    x, y, yaw = mission.park_pose((1.0, 2.0, heading), 0.7)
    assert yaw == heading
    assert (x - 0.7 * math.cos(yaw), y - 0.7 * math.sin(yaw)) == pytest.approx((1.0, 2.0))


def _excavator():
    _sim_layout()
    from lunabot_sim.scene import demo_excavator

    return demo_excavator


def _sim_layout():
    """lunabot_sim's arena layout, from the installed package or this repo's source.

    It is Isaac-free, but not installed into the ROS environment colcon tests in,
    and skipping would defeat the cross-check.
    """
    sys.path.append(str(PKG.parent / 'lunabot_sim'))
    return pytest.importorskip('lunabot_sim.scene.layout')

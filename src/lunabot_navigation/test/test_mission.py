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


def _plan(cycles):
    return mission.plan(
        cycles,
        PARAMS['dig_pose'],
        PARAMS['dump_pose'],
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
        assert drive_dump.pose == tuple(PARAMS['dump_pose'])
        assert dump.pose is None and dump.seconds > 0


def test_ucf_poses_are_in_their_zones():
    layout = _sim_layout()
    ucf = layout.UCF
    assert ucf.excavation_zone.contains(*PARAMS['dig_pose'][:2])
    assert ucf.construction_zone.contains(*PARAMS['dump_pose'][:2])
    assert ucf.start_zone.contains(*PARAMS['home_pose'][:2])


def _sim_layout():
    """lunabot_sim's arena layout, from the installed package or this repo's source.

    It is Isaac-free, but not installed into the ROS environment colcon tests in,
    and skipping would defeat the cross-check.
    """
    sys.path.append(str(PKG.parent / 'lunabot_sim'))
    return pytest.importorskip('lunabot_sim.scene.layout')

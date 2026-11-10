#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The demo recorder's captions: every mission phase has one, and they fit.

Pure Python: the recorder imports rclpy, cv2 and numpy work only inside main()
and at module level respectively, and the mission's plan() needs no ROS.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


recorder = _load('demo_recorder', SRC / 'lunabot_bringup' / 'scripts' / 'demo_recorder.py')
mission = _load('mission', SRC / 'lunabot_navigation' / 'scripts' / 'mission.py')


@pytest.mark.parametrize('cycles', [1, 3])
def test_every_phase_the_mission_can_announce_has_a_caption(cycles):
    pose = (0.0, 0.0, 0.0)
    states = [step.state for step in mission.plan(cycles, pose, [pose], pose, 1.0, 1.0)]
    states += [
        'WAITING FOR NAVIGATION',
        'WAITING FOR EXCAVATOR',
        'SURVEYING START AREA',
        'LOCALISING ON APRILTAGS',
        'MISSION COMPLETE in 81 s',
        f'FAILED: {states[0]}',
        f'{states[0]} - RETRYING',
    ]
    for state in states:
        assert recorder.caption(state), state


def test_retrying_is_not_captioned_as_the_leg_itself():
    leg = 'DRIVING TO EXCAVATION ZONE (1/2)'
    assert recorder.caption(f'{leg} - RETRYING') != recorder.caption(leg)


def test_the_plan_is_drawn_only_while_driving():
    assert recorder.shows_plan('DRIVING TO EXCAVATION ZONE (1/2)')
    assert recorder.shows_plan('RETURNING TO START')
    assert recorder.shows_plan('')  # no mission running: show whatever Nav2 does
    for state in ('EXCAVATING (1/2)', 'DEPOSITING', 'MISSION COMPLETE in 132 s', 'SURVEYING'):
        assert not recorder.shows_plan(state)


def test_wrapping_keeps_every_line_inside_the_width_and_loses_no_words():
    text = recorder.caption('SURVEYING START AREA')
    lines = recorder.wrap(text, 300, lambda t: 10 * len(t))
    assert all(10 * len(line) <= 300 for line in lines)
    assert ' '.join(lines) == text

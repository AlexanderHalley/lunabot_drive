#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The curvature limiter's arithmetic, and its agreement with the planner.

Pure Python: the node imports rclpy only inside main(), so CI without ROS
runs this too.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
SCRIPT = PKG / 'scripts' / 'curvature_limiter.py'
PARAMS = PKG / 'config' / 'nav2_params.yaml'

spec = importlib.util.spec_from_file_location('curvature_limiter', SCRIPT)
limiter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(limiter)


def test_gentle_turns_pass_through():
    assert limiter.limit(0.3, 0.1, 1.0) == (0.3, 0.1)


@pytest.mark.parametrize('angular', [0.5, 2.0])
def test_tight_turns_are_widened_to_the_radius(angular):
    linear, limited = limiter.limit(0.3, angular, 1.0)
    assert linear == 0.3
    assert limited == pytest.approx(0.3)
    assert limiter.limit(0.3, -angular, 1.0)[1] == pytest.approx(-0.3)


def test_reversing_turns_are_limited_too():
    assert limiter.limit(-0.3, 2.0, 1.0) == (-0.3, pytest.approx(0.3))


def test_no_turning_on_the_spot():
    """The whole point: in sand it digs in instead of turning."""
    assert limiter.limit(0.0, 1.0, 1.0) == (0.0, 0.0)


def test_zero_radius_disables_the_limit():
    assert limiter.limit(0.0, 1.0, 0.0) == (0.0, 1.0)


def test_the_limit_is_no_tighter_than_the_planner_plans():
    """The limiter clamps COMMANDS; the planner plans ACHIEVED arcs.

    A commanded arc comes out wider in sand (Isaac: commanded 1.0 m drives
    ~1.5 m), so the command limit sits below the planner's radius -- but never
    above it, or the rover could not follow its own paths.
    """
    config = yaml.safe_load(PARAMS.read_text())
    command = config['curvature_limiter']['ros__parameters']['min_turning_radius']
    planned = config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
    assert 0.0 < command <= planned

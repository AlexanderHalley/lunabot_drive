#!/usr/bin/env python3
"""Construct every launch description in this package without running it.

Launch files fail at construction far more often than they fail at runtime --
a renamed argument, a typo in a substitution, a PathJoinSubstitution missing a
component. Those all raise here, in about a second, with no ROS graph and no
hardware.

This does NOT prove the stack works. test_mock_bringup.py does that.
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'
LAUNCH_FILES = sorted(p.name for p in LAUNCH_DIR.glob('*.launch.py'))


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_files_exist():
    """Guards against this test silently passing over an empty directory."""
    assert LAUNCH_FILES, f'no launch files found in {LAUNCH_DIR}'
    assert 'robot.launch.py' in LAUNCH_FILES


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_generates_a_launch_description(name):
    description = load(name).generate_launch_description()
    assert isinstance(description, LaunchDescription)
    assert description.entities, f'{name} produced an empty description'


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_every_argument_is_documented(name):
    """A DeclareLaunchArgument with no description is invisible in --show-args.

    Cheap to enforce, and the alternative is a launch file whose options can
    only be discovered by reading it.
    """
    description = load(name).generate_launch_description()
    undocumented = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument) and not entity.description
    ]
    assert not undocumented, f'{name}: undocumented arguments {undocumented}'


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_arguments_resolve_with_their_defaults(name):
    """Every declared default must actually evaluate.

    Catches a default that is a substitution referencing a package that is not
    a dependency, which otherwise surfaces at launch time as a stack trace.
    """
    context = LaunchContext()
    description = load(name).generate_launch_description()

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)


def test_robot_launch_exposes_the_documented_arguments():
    """These names appear in the README and in docs/. Renaming one silently
    breaks copy-pasted commands, so pin them here."""
    description = load('robot.launch.py').generate_launch_description()
    names = {e.name for e in description.entities if isinstance(e, DeclareLaunchArgument)}
    assert {'hw', 'use_sim_time', 'odom_source', 'rviz'} <= names


def test_hardware_choices_match_the_urdf():
    """The three values here must be exactly the three the xacro branches on."""
    description = load('robot.launch.py').generate_launch_description()
    hw = next(
        e
        for e in description.entities
        if isinstance(e, DeclareLaunchArgument) and e.name == 'hw'
    )
    assert set(hw.choices) == {'mock', 'sim', 'real'}


@pytest.mark.parametrize(
    ('odom_source', 'expected'),
    [('wheel', 'true'), ('visual', 'false'), ('ekf', 'false')],
)
def test_odom_tf_ownership_is_exclusive(odom_source, expected):
    """diff_drive_controller must publish odom -> base_link only for
    odom_source:=wheel.

    Two publishers on one transform gives a TF tree that looks correct in
    view_frames and behaves nondeterministically, so this is worth a test
    rather than a comment.
    """
    module = load('robot.launch.py')
    context = LaunchContext()
    assert module._odom_tf_from(odom_source).perform(context) == expected

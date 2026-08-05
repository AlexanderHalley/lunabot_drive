#!/usr/bin/env python3
"""Construct the Nav2 launch description without running any of it.

Launch files fail at construction far more often than at runtime -- a renamed
argument, a typo in a substitution, a package that is not a dependency. Those
all raise here, in about a second, with no ROS graph and no robot.

The test that earns this file, though, is
test_nav2_never_publishes_to_cmd_vel. nav2_bringup's own navigation_launch.py
remaps velocity_smoother's output onto `cmd_vel`, which is twist_mux's OUTPUT
topic; including it would have put Nav2 on the wire beside the mux and made
the teleop priority in twist_mux.yaml decorative. Nothing about that failure
looks like a launch bug from the outside -- the rover simply stops obeying the
controller -- so it is pinned here.

Needs Nav2 installed (navigation.launch.py imports nav2_common), so this runs
under colcon rather than in the standalone lint job.
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parent.parent
LAUNCH_DIR = PKG / 'launch'

# twist_mux's output. Nav2 must never publish here -- see the module docstring.
BUS_TOPIC = 'cmd_vel'

# What Nav2 publishes instead, and what twist_mux subscribes to at priority 10.
NAV_TOPIC = 'cmd_vel_nav'


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def module():
    return load('navigation.launch.py')


@pytest.fixture(scope='module')
def description(module):
    return module.generate_launch_description()


def nodes(description):
    return [entity for entity in description.entities if isinstance(entity, Node)]


def test_generates_a_launch_description(description):
    assert isinstance(description, LaunchDescription)
    assert description.entities


def test_arguments_are_documented_and_resolve(description):
    """A DeclareLaunchArgument with no description is invisible in
    --show-args, and a default that does not evaluate surfaces at launch time
    as a stack trace."""
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            assert entity.description, f'{entity.name} has no description'
            entity.visit(context)


def test_every_declared_node_is_constructed(module, description):
    """NAV2_NODES is the source of truth; the description is built from it.

    Pinning the count as well catches a node quietly dropped from the
    comprehension.
    """
    assert len(nodes(description)) == len(module.NAV2_NODES) + 1  # + lifecycle_manager


def test_nav2_never_publishes_to_cmd_vel(module):
    """The one that matters. /cmd_vel belongs to twist_mux.

    Every Nav2 node that emits velocity must have `cmd_vel` remapped away.
    Publishing there directly would bypass the mux entirely, and teleop would
    stop being able to override navigation -- which is the whole reason the
    mux exists.
    """
    for node in module.NAV2_NODES:
        targets = [target for _, target in node['remappings']]
        assert BUS_TOPIC not in targets, (
            f"{node['name']} remaps onto {BUS_TOPIC}, which is twist_mux's output"
        )


def test_only_the_smoother_publishes_the_nav_topic(module):
    """Exactly one publisher on /cmd_vel_nav, and it is the last node in the
    chain.

    The controller and the recovery behaviours both feed the smoother instead.
    A recovery that skipped it would step around the acceleration limits and
    spin the wheels on regolith -- which is the one manoeuvre this rover
    should never make.
    """
    publishers = [
        node['name']
        for node in module.NAV2_NODES
        if NAV_TOPIC in [t for _, t in node['remappings']]
    ]
    assert publishers == ['velocity_smoother']


def test_the_smoother_consumes_what_the_others_produce(module):
    """The internal hop has to match at both ends or the chain is two
    disconnected halves, each of which looks healthy on its own."""
    by_name = {node['name']: node for node in module.NAV2_NODES}
    smoother_input = dict(by_name['velocity_smoother']['remappings'])[BUS_TOPIC]

    for name in ['controller_server', 'behavior_server']:
        assert dict(by_name[name]['remappings'])[BUS_TOPIC] == smoother_input


def test_every_node_is_managed(module):
    """A lifecycle node nobody manages stays in `unconfigured` forever,
    publishing nothing and reporting nothing."""
    assert module.LIFECYCLE_NODES == [node['name'] for node in module.NAV2_NODES]


def test_the_controller_activates_before_the_navigator(module):
    """bt_navigator starts sending goals as soon as it is active. Bringing it
    up before the servers it commands produces failures on the first goal that
    read as planning problems."""
    order = module.LIFECYCLE_NODES
    assert order.index('controller_server') < order.index('bt_navigator')
    assert order.index('planner_server') < order.index('bt_navigator')


def test_amcl_and_map_server_are_not_started(module):
    """rtabmap owns map -> odom and publishes /map. Starting Nav2's
    localisation stack too gives two publishers on map -> odom and a robot
    that teleports between two beliefs about where it is."""
    started = {node['executable'] for node in module.NAV2_NODES}
    assert 'amcl' not in started
    assert 'map_server' not in started

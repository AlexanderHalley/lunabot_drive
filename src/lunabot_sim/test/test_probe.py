#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Tests for the Isaac API probe, on a machine with no Isaac.

The probe's job is to check a real install against the names this package
guesses at. That makes its LIST of names the thing most worth testing here:
a probe that reports every name resolving because it forgot to ask about one
is worse than no probe, and it fails silently the moment someone adds a graph.

So the graph sources are parsed and every OmniGraph node type they create is
compared against what probe.py says it checks. No Isaac, no GPU, no
Omniverse -- just ast over four files.
"""

import ast
import json
from pathlib import Path

import pytest

from lunabot_sim import probe

GRAPHS_DIR = Path(__file__).resolve().parent.parent / 'lunabot_sim' / 'graphs'


def node_types_created_by_the_graphs():
    """Every compat.og_node_type('X') / core_node_type('X') in graphs/.

    Read out of the source rather than by importing and calling build(),
    because build() needs a running simulator. ast means a renamed node type
    cannot slip past by being spelled differently.
    """
    bridge, core = set(), set()

    for path in sorted(GRAPHS_DIR.glob('*.py')):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if not node.args or not isinstance(node.args[0], ast.Constant):
                continue
            name = node.args[0].value
            if not isinstance(name, str):
                continue
            if node.func.attr == 'og_node_type':
                bridge.add(name)
            elif node.func.attr == 'core_node_type':
                core.add(name)

    return bridge, core


def test_the_graphs_create_something():
    """Guards against the ast walk silently matching nothing."""
    bridge, core = node_types_created_by_the_graphs()
    assert bridge, f'found no og_node_type calls under {GRAPHS_DIR}'
    assert core, f'found no core_node_type calls under {GRAPHS_DIR}'


def test_probe_checks_every_bridge_node_the_graphs_create():
    bridge, _ = node_types_created_by_the_graphs()
    missing = bridge - set(probe.REQUIRED_BRIDGE_NODES)
    assert not missing, (
        f'graphs/ creates {sorted(missing)}, which the probe does not check. '
        'Add them to probe.REQUIRED_BRIDGE_NODES, or the first thing anyone learns '
        'about a renamed node type is a crash mid-startup on the sim machine.'
    )


def test_probe_checks_every_core_node_the_graphs_create():
    _, core = node_types_created_by_the_graphs()
    missing = core - set(probe.REQUIRED_CORE_NODES)
    assert not missing, (
        f'graphs/ creates {sorted(missing)}, which the probe does not check. '
        'Add them to probe.REQUIRED_CORE_NODES.'
    )


def test_the_probe_checks_nothing_the_graphs_do_not_use():
    """The other direction: a stale name makes the probe fail on a good install."""
    bridge, core = node_types_created_by_the_graphs()
    stale = (set(probe.REQUIRED_BRIDGE_NODES) - bridge) | (set(probe.REQUIRED_CORE_NODES) - core)
    assert not stale, (
        f'the probe checks {sorted(stale)}, which nothing in graphs/ creates. '
        'A name kept after the code stopped using it turns a passing install into '
        'a reported failure.'
    )


def test_camera_helper_types_match_the_camera_graph():
    """probe.CAMERA_HELPER_TYPES against the values camera.py actually sets."""
    source = (GRAPHS_DIR / 'camera.py').read_text()
    tree = ast.parse(source)

    assigned = {
        node.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
        and any(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets)
    }

    for helper_type in probe.CAMERA_HELPER_TYPES:
        assert helper_type in assigned, (
            f'{helper_type!r} is not one of the values camera.py sets: {sorted(assigned)}'
        )


# ---------------------------------------------------------------------------
# Report behaviour. Small, but the exit code is what a person reads at 2am.
# ---------------------------------------------------------------------------


@pytest.fixture
def report():
    return probe.Report()


def test_a_clean_report_exits_zero(report):
    report.add('bridge extension', probe.OK, 'isaacsim.ros2.bridge')
    report.add('isaac version', probe.INFO, '4.5.0')
    assert report.exit_code() == 0
    assert 'Every name resolved' in report.to_text()


def test_a_missing_name_exits_nonzero(report):
    report.add('node ROS2PublishClock', probe.MISSING, 'not registered', 'compat.OG_PREFIXES')
    assert report.exit_code() == 1
    assert 'compat.py' in report.to_text()


def test_unknown_is_reported_but_does_not_fail(report):
    """The probe not being able to ask is not the same as the answer being no.

    Isaac's introspection APIs have moved as much as everything else, so
    "could not list the registered node types" must not be reported as "the
    node types are missing" -- that would send someone editing compat.py to
    fix a working install.
    """
    report.add('node type registry', probe.UNKNOWN, 'could not list')
    assert report.exit_code() == 0


def test_findings_survive_json(report):
    report.add('a', probe.MISSING, 'detail', 'fix')
    findings = json.loads(report.to_json())['findings']
    assert findings == [{'name': 'a', 'status': 'missing', 'detail': 'detail', 'fix': 'fix'}]

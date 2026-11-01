#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The rover image's files agree with the workspace (docs/DEPLOYMENT.md).

Each check here is a deployment that builds and starts and is quietly wrong:
`ros2 launch` accepts a misspelled argument without a word, a package whose
manifest the Dockerfile does not copy gets none of its rosdep dependencies,
and a branch in lunabot.repos makes two builds of one commit differ.

Pure Python: no Docker, no ROS graph.
"""

import importlib.util
import re
from pathlib import Path

import pytest
import yaml
from launch.actions import DeclareLaunchArgument

REPO = Path(__file__).resolve().parents[3]
DOCKER = REPO / 'docker'
LAUNCH = REPO / 'src' / 'lunabot_bringup' / 'launch'
DOCKERFILE = (DOCKER / 'Dockerfile.robot').read_text()


def _env(name):
    pairs = {}
    for line in (DOCKER / name).read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            key, _, value = line.partition('=')
            pairs[key.strip()] = value.strip()
    return pairs


def _declared(launch_file):
    spec = importlib.util.spec_from_file_location(launch_file, LAUNCH / launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    return {e.name for e in entities if isinstance(e, DeclareLaunchArgument)}


@pytest.mark.parametrize('env_file', ['robot.env', 'sim.env'])
def test_every_launch_argument_is_one_the_launch_file_declares(env_file):
    launch_file, *args = _env(env_file)['LUNABOT_LAUNCH'].split()
    unknown = sorted({a.split(':=')[0] for a in args} - _declared(launch_file))
    assert not unknown, f'{env_file}: {launch_file} does not declare {unknown}'


@pytest.mark.parametrize('env_file', ['robot.env', 'sim.env'])
def test_the_env_files_set_what_compose_requires(env_file):
    compose = (DOCKER / 'compose.robot.yml').read_text()
    required = set(re.findall(r'\$\{(\w+):\?', compose))
    assert required <= set(_env(env_file)), required - set(_env(env_file))


def test_the_rover_waits_for_start_and_records():
    args = _env('robot.env')['LUNABOT_LAUNCH'].split()
    for wanted in ('hw:=real', 'wait_for_start:=true', 'foxglove_profile:=field', 'record:=true'):
        assert wanted in args
    compose = yaml.safe_load((DOCKER / 'compose.robot.yml').read_text())
    command = compose['services']['mission']['command']
    assert 'autostart:=false' in command
    # Later wins: a -p before the params file is overridden by it (Isaac,
    # 2026-10-31: cycles:=2 ran one cycle).
    assert command.index('--params-file') < command.index('-p ')
    # Not through `ros2 run`, which does not forward SIGINT: SIGKILL on stop.
    assert command.startswith('/opt/lunabot/lib/lunabot_navigation/mission.py ')


def test_the_image_gets_every_package_manifest():
    """Rosdep reads only the manifests copied before the source; a missing one has no deps."""
    packages = sorted(p.parent.name for p in (REPO / 'src').glob('*/package.xml'))
    copied = re.findall(r'^COPY src/(\w+)/package\.xml ', DOCKERFILE, re.M)
    assert sorted(copied) == packages


def test_the_image_skips_the_same_rosdep_keys_as_ci():
    ci = yaml.safe_load((REPO / '.github' / 'workflows' / 'ci.yml').read_text())
    keys = re.search(r'--skip-keys "([^"]+)"', DOCKERFILE).group(1)
    assert keys == ci['env']['ROSDEP_SKIP_KEYS']


def test_third_party_sources_are_pinned_to_commits():
    repos = yaml.safe_load((REPO / 'lunabot.repos').read_text())['repositories']
    for name, entry in repos.items():
        assert re.fullmatch(r'[0-9a-f]{40}', str(entry['version'])), name

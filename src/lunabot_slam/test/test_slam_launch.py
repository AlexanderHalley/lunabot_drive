#!/usr/bin/env python3
"""Every SLAM backend must at least produce a valid launch description.

This is the only automated coverage the cuVSLAM path can have: the node needs
a GPU and may not be installable on this distro at all. It still catches
typos, missing substitutions and bad argument names, which is most of what
actually breaks in launch files.

rtabmap gets real functional testing by running it against a recorded bag.
That is a manual step -- see docs/SLAM.md.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument

PKG = Path(__file__).resolve().parent.parent
LAUNCH_DIR = PKG / 'launch'
CONFIG_DIR = PKG / 'config'

BACKENDS = ['rtabmap', 'cuvslam']


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('name', ['slam.launch.py'] + [f'{b}.launch.py' for b in BACKENDS])
def test_generates_a_launch_description(name):
    description = load(name).generate_launch_description()
    assert isinstance(description, LaunchDescription)
    assert description.entities


@pytest.mark.parametrize('name', ['slam.launch.py'] + [f'{b}.launch.py' for b in BACKENDS])
def test_arguments_are_documented_and_resolve(name):
    context = LaunchContext()
    description = load(name).generate_launch_description()

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            assert entity.description, f'{name}: {entity.name} has no description'
            entity.visit(context)


def test_backend_choices_match_the_launch_files_that_exist():
    description = load('slam.launch.py').generate_launch_description()
    backend = next(
        e
        for e in description.entities
        if isinstance(e, DeclareLaunchArgument) and e.name == 'backend'
    )

    assert set(backend.choices) == set(BACKENDS) | {'none'}
    for name in BACKENDS:
        assert (LAUNCH_DIR / f'{name}.launch.py').is_file()


def test_topic_defaults_match_the_contract():
    """These defaults are a copy of docs/TOPIC_FRAME_CONTRACT.md.

    Copies drift. Pinning them here means a change to the contract that
    forgets this file fails a test rather than producing a SLAM backend
    subscribed to a topic nobody publishes -- which presents as silence, not
    as an error.
    """
    description = load('slam.launch.py').generate_launch_description()
    defaults = {
        e.name: e.default_value[0].text
        for e in description.entities
        if isinstance(e, DeclareLaunchArgument) and e.name.endswith('_topic')
    }

    assert defaults == {
        'rgb_topic': '/oak_d/rgb/image_rect',
        'depth_topic': '/oak_d/stereo/image_raw',
        'camera_info_topic': '/oak_d/rgb/camera_info',
        'left_rect_topic': '/oak_d/left/image_rect',
        'right_rect_topic': '/oak_d/right/image_rect',
        'left_info_topic': '/oak_d/left/camera_info',
        'right_info_topic': '/oak_d/right/camera_info',
        'imu_topic': '/oak_d/imu/data',
        'odom_topic': '/odom',
    }


def test_odom_tf_defaults_to_off_in_both_backends():
    """Neither backend may publish odom -> base_link by default.

    The default odom source is diff_drive_controller. A backend that also
    publishes that transform gives a TF tree that looks correct in
    view_frames and behaves nondeterministically -- so the safe value is the
    default and turning it on is explicit.
    """
    for name in ['slam.launch.py'] + [f'{b}.launch.py' for b in BACKENDS]:
        description = load(name).generate_launch_description()
        arg = next(
            e
            for e in description.entities
            if isinstance(e, DeclareLaunchArgument) and e.name == 'publish_odom_tf'
        )
        assert arg.default_value[0].text == 'false', name


@pytest.mark.parametrize('name', ['rtabmap.yaml', 'cuvslam.yaml'])
def test_config_files_parse(name):
    with open(CONFIG_DIR / name) as handle:
        config = yaml.safe_load(handle)
    assert config, f'{name} is empty'


def test_both_backends_agree_on_frame_names():
    """rtabmap and cuVSLAM use different parameter names for the same frames.

    Getting one of them wrong produces a second, disconnected TF tree rooted
    at a frame nobody else uses, which looks like SLAM not working rather
    than like a config typo.
    """
    rtabmap = yaml.safe_load(open(CONFIG_DIR / 'rtabmap.yaml'))['rtabmap']['ros__parameters']
    cuvslam = yaml.safe_load(open(CONFIG_DIR / 'cuvslam.yaml'))['visual_slam_node'][
        'ros__parameters'
    ]

    assert rtabmap['frame_id'] == cuvslam['base_frame'] == 'base_link'
    assert rtabmap['odom_frame_id'] == cuvslam['odom_frame'] == 'odom'
    assert rtabmap['map_frame_id'] == cuvslam['map_frame'] == 'map'


def test_both_backends_claim_map_to_odom():
    """Whichever backend runs must own map -> odom, or nothing does and the
    map frame never appears."""
    rtabmap = yaml.safe_load(open(CONFIG_DIR / 'rtabmap.yaml'))['rtabmap']['ros__parameters']
    cuvslam = yaml.safe_load(open(CONFIG_DIR / 'cuvslam.yaml'))['visual_slam_node'][
        'ros__parameters'
    ]

    assert rtabmap['publish_tf'] is True
    assert cuvslam['publish_map_to_odom_tf'] is True
    # ...and neither claims odom -> base_link in its config.
    assert cuvslam['publish_odom_to_base_tf'] is False


def test_rtabmap_is_configured_for_a_planar_rover():
    """Force3DoF is what stops visual noise tilting the whole map. The rover
    cannot roll, pitch or change altitude in any way the estimator should
    believe."""
    params = yaml.safe_load(open(CONFIG_DIR / 'rtabmap.yaml'))['rtabmap']['ros__parameters']
    assert params['Reg/Force3DoF'] == 'true'
    assert params['Optimizer/Slam2D'] == 'true'
    # 2D grid, because Nav2's costmap cannot consume an octomap.
    assert params['Grid/3D'] == 'false'

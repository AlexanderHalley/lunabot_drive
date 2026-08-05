#!/usr/bin/env python3
"""Structural checks on the config files.

The headline one is the YAML 1.1 exponent trap. `1.0e6` and `1e-09` both
resolve to STRINGS, not floats, because YAML 1.1 requires both a decimal point
in the mantissa and a sign on the exponent. A covariance that is secretly a
string is rejected by the node at load time, and the error names the
parameter rather than the notation -- so it reads as "this parameter is wrong"
rather than "this number is not a number".

Both ekf.yaml and controllers.yaml were written with this bug. Hence a test
rather than a comment.
"""

import re
from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
CONFIG_FILES = sorted(CONFIG_DIR.glob('*.yaml'))

# Matches a scalar that a human would read as a number.
NUMERIC_LOOKING = re.compile(r'^-?\d+(\.\d*)?([eE][-+]?\d+)?$')


def _walk(node, path=''):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _walk(value, f'{path}.{key}')
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from _walk(value, f'{path}[{index}]')
    else:
        yield path, node


def test_config_files_exist():
    assert CONFIG_FILES, f'no config files in {CONFIG_DIR}'


@pytest.mark.parametrize('path', CONFIG_FILES, ids=lambda p: p.name)
def test_parses(path):
    assert yaml.safe_load(path.read_text()) is not None, f'{path.name} is empty'


@pytest.mark.parametrize('path', CONFIG_FILES, ids=lambda p: p.name)
def test_no_numbers_hiding_as_strings(path):
    """Every scalar that looks like a number must have parsed as one."""
    offenders = [
        f'{key} = {value!r}'
        for key, value in _walk(yaml.safe_load(path.read_text()))
        if isinstance(value, str) and NUMERIC_LOOKING.match(value)
    ]
    assert not offenders, (
        f'{path.name}: these parsed as strings, not numbers -- YAML 1.1 needs a '
        f'decimal point and a signed exponent (1.0e+6, not 1.0e6):\n  '
        + '\n  '.join(offenders)
    )


def test_ekf_covariances_are_full_matrices():
    """robot_localization wants flattened 15x15 matrices, not 15 diagonal
    entries. Passing 15 is an easy mistake and stops the EKF from starting."""
    params = yaml.safe_load((CONFIG_DIR / 'ekf.yaml').read_text())['ekf_node']['ros__parameters']

    for key in ('process_noise_covariance', 'initial_estimate_covariance'):
        assert len(params[key]) == 225, f'{key} has {len(params[key])} entries, expected 225'
        assert all(isinstance(v, float) for v in params[key]), f'{key} has non-float entries'

    # The sensor configs are 15-element boolean masks, not matrices.
    for key in ('odom0_config', 'imu0_config'):
        assert len(params[key]) == 15
        assert all(isinstance(v, bool) for v in params[key])


def test_ekf_does_not_fuse_absolute_pose_from_wheel_odometry():
    """Fusing odom's pose as well as its velocity feeds the filter the same
    information twice and makes it overconfident in a fabricated number."""
    params = yaml.safe_load((CONFIG_DIR / 'ekf.yaml').read_text())['ekf_node']['ros__parameters']
    x, y, z, roll, pitch, yaw = params['odom0_config'][:6]
    assert not any([x, y, z, roll, pitch, yaw]), 'wheel odometry pose must not be fused'
    # vx and vyaw are the two channels that carry real information.
    assert params['odom0_config'][6] is True, 'vx should be fused'
    assert params['odom0_config'][11] is True, 'vyaw should be fused'


def test_controller_wheel_names_match_the_urdf():
    """These strings are the contract with lunabot_description. A rename on
    either side gives a controller that cannot claim its interfaces."""
    params = yaml.safe_load((CONFIG_DIR / 'controllers.yaml').read_text())
    diff_drive = params['diff_drive_controller']['ros__parameters']

    assert diff_drive['left_wheel_names'] == [
        'front_left_wheel_joint',
        'rear_left_wheel_joint',
    ]
    assert diff_drive['right_wheel_names'] == [
        'front_right_wheel_joint',
        'rear_right_wheel_joint',
    ]


def test_controller_update_rate_sustains_the_can_heartbeat():
    """SparkFlex controllers fault without a keep-alive roughly every 50 ms.

    The heartbeat is sent from write(), which controller_manager calls at
    update_rate -- so this parameter is not free to tune downward.
    See docs/HARDWARE_CAN.md.
    """
    params = yaml.safe_load((CONFIG_DIR / 'controllers.yaml').read_text())
    rate = params['controller_manager']['ros__parameters']['update_rate']
    assert rate >= 40, f'update_rate {rate} Hz is too slow to keep the motors alive'


def test_camera_profiles_agree_on_the_tf_prefix():
    """Every profile must name the same prefix and must not let the driver
    publish its own TF -- robot_state_publisher owns the tree."""
    for path in CONFIG_DIR.glob('oak_d_s2*.yaml'):
        camera = yaml.safe_load(path.read_text())['/**']['ros__parameters']['camera']
        assert camera['i_tf_tf_prefix'] == 'oak_d', path.name
        assert camera['i_publish_tf_from_calibration'] is False, path.name


def test_twist_mux_prefers_teleop_over_navigation():
    """A human reaching for the controller wants the rover to stop doing what
    it is doing."""
    topics = yaml.safe_load((CONFIG_DIR / 'twist_mux.yaml').read_text())['twist_mux'][
        'ros__parameters'
    ]['topics']
    assert topics['joystick']['priority'] > topics['navigation']['priority']


def test_teleop_deadman_is_required():
    """The 2026 launch file set require_enable_button false, so the rover drove
    whenever the stick moved. On 25 kg with brake-mode motors that is a safety
    property, not a preference."""
    params = yaml.safe_load((CONFIG_DIR / 'teleop_switch_pro.yaml').read_text())
    assert params['teleop_twist_joy_node']['ros__parameters']['require_enable_button'] is True

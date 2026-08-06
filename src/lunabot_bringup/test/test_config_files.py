#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

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

# The other half of the kinematic contract, in the sibling package.
PROPERTIES = (
    CONFIG_DIR.parent.parent / 'lunabot_description' / 'urdf' / 'common' / 'properties.xacro'
)

# Matches a scalar that a human would read as a number.
NUMERIC_LOOKING = re.compile(r'^-?\d+(\.\d*)?([eE][-+]?\d+)?$')


def xacro_property(name):
    """Read one <xacro:property> value out of properties.xacro.

    Regex rather than running xacro, so this test keeps working in the CI job
    that has no ROS installed. The properties in question are literal numbers.
    """
    assert PROPERTIES.is_file(), f'{PROPERTIES} not found'
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([-\d.]+)"', PROPERTIES.read_text()
    )
    assert match, f'{name} not found in {PROPERTIES.name}'
    return float(match.group(1))


def _walk(node, path=''):
    """Yield every leaf as (path, value). An EMPTY container is a leaf.

    That last part is not a detail. Recursing into an empty list visits zero
    elements and yields nothing, so a check written on top of this could not
    see an empty list at all -- test_no_empty_lists below was written that way
    first and silently passed against the very file that had one.
    """
    if isinstance(node, dict) and node:
        for key, value in node.items():
            yield from _walk(value, f'{path}.{key}')
    elif isinstance(node, list) and node:
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
        f'decimal point and a signed exponent (1.0e+6, not 1.0e6):\n  ' + '\n  '.join(offenders)
    )


@pytest.mark.parametrize('path', CONFIG_FILES, ids=lambda p: p.name)
def test_no_empty_lists(path):
    """An empty sequence in a ROS 2 parameter file leaves the parameter UNSET.

    The sibling of the exponent trap above, and it cost more. A YAML `[]`
    carries no element type, so the parameter parser cannot build a value and
    the parameter arrives at the node as PARAMETER_NOT_SET rather than as an
    empty array. A node using generate_parameter_library then refuses to
    initialize at all:

        Caught exception of type InvalidParameterValueException while
        initializing controller 'joint_state_broadcaster':
        parameter_value_from failed for parameter 'joints':
        No parameter value set

    controllers.yaml carried `joints: []` for joint_state_broadcaster, written
    to say out loud what the controller already defaults to. The spawner
    reported only "Failed loading controller", so /joint_states never
    appeared, robot_state_publisher had nothing to publish wheel transforms
    from, and both whole-stack tests failed a long way from the cause.

    A parameter whose default is an empty list must simply be left out.
    """
    offenders = [
        key
        for key, value in _walk(yaml.safe_load(path.read_text()))
        if isinstance(value, list) and not value
    ]
    assert not offenders, (
        f'{path.name}: these are empty lists, which arrive at the node as unset '
        f'rather than as empty:\n  ' + '\n  '.join(offenders) + '\n'
        'Delete the key and let the default apply.'
    )


def test_ekf_covariances_are_full_matrices():
    """robot_localization wants flattened 15x15 matrices, not 15 diagonal entries.

    Passing 15 is an easy mistake and stops the EKF from starting.
    """
    params = yaml.safe_load((CONFIG_DIR / 'ekf.yaml').read_text())['ekf_node']['ros__parameters']

    for key in ('process_noise_covariance', 'initial_estimate_covariance'):
        assert len(params[key]) == 225, f'{key} has {len(params[key])} entries, expected 225'
        assert all(isinstance(v, float) for v in params[key]), f'{key} has non-float entries'

    # The sensor configs are 15-element boolean masks, not matrices.
    for key in ('odom0_config', 'imu0_config'):
        assert len(params[key]) == 15
        assert all(isinstance(v, bool) for v in params[key])


def test_ekf_does_not_fuse_absolute_pose_from_wheel_odometry():
    """Wheel odometry contributes velocity only, never absolute pose.

    Fusing odom's pose as well as its velocity feeds the filter the same information twice and
    makes it overconfident in a fabricated number.
    """
    params = yaml.safe_load((CONFIG_DIR / 'ekf.yaml').read_text())['ekf_node']['ros__parameters']
    x, y, z, roll, pitch, yaw = params['odom0_config'][:6]
    assert not any([x, y, z, roll, pitch, yaw]), 'wheel odometry pose must not be fused'
    # vx and vyaw are the two channels that carry real information.
    assert params['odom0_config'][6] is True, 'vx should be fused'
    assert params['odom0_config'][11] is True, 'vyaw should be fused'


def test_controller_wheel_names_match_the_urdf():
    """These strings are the contract with lunabot_description.

    A rename on either side gives a controller that cannot claim its interfaces.
    """
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


def test_wheel_constants_match_the_urdf():
    """The rule CONTRIBUTING.md states and nothing enforced until now.

    wheel_radius and wheel_separation exist in properties.xacro AND in
    controllers.yaml, because xacro cannot reach into a controller YAML. Change
    one without the other and diff_drive_controller integrates the wrong
    kinematics: /odom drifts, SLAM fights it, Nav2 plans against a robot that
    is not where it thinks. Nothing errors, and the symptom is a hundred metres
    downstream of the cause.

    "Change one, change the other, same commit" is a rule a test can keep, so
    it keeps it.
    """
    diff_drive = yaml.safe_load((CONFIG_DIR / 'controllers.yaml').read_text())[
        'diff_drive_controller'
    ]['ros__parameters']

    assert diff_drive['wheel_radius'] == pytest.approx(xacro_property('wheel_radius'))
    assert diff_drive['wheel_separation'] == pytest.approx(xacro_property('wheel_separation'))


def test_the_controller_cannot_command_more_than_the_motors_deliver():
    """A velocity ceiling above what the drivetrain can reach is a lie.

    The controller clamps to its own limit, the motors saturate below it, and
    the wheels turn slower than the odometry believes -- which is the same
    failure as a wrong wheel radius, arriving by a different route.

    Linear only. The angular bound depends on wheel_separation_multiplier,
    which is a skid-steer fudge factor rather than a measured quantity, so
    pinning it here would assert against a guess.
    """
    limits = yaml.safe_load((CONFIG_DIR / 'controllers.yaml').read_text())[
        'diff_drive_controller'
    ]['ros__parameters']

    achievable = xacro_property('max_wheel_rad_s') * xacro_property('wheel_radius')
    assert limits['linear.x.max_velocity'] <= achievable, (
        f'{limits["linear.x.max_velocity"]} m/s asks for more than the '
        f'{achievable} m/s the motors can turn'
    )


def test_controller_update_rate_sustains_the_can_heartbeat():
    """SparkFlex controllers fault without a keep-alive roughly every 50 ms.

    The heartbeat is sent from write(), which controller_manager calls at
    update_rate -- so this parameter is not free to tune downward.
    See docs/HARDWARE_CAN.md.
    """
    params = yaml.safe_load((CONFIG_DIR / 'controllers.yaml').read_text())
    rate = params['controller_manager']['ros__parameters']['update_rate']
    assert rate >= 40, f'update_rate {rate} Hz is too slow to keep the motors alive'


def test_no_camera_profile_lets_the_driver_publish_tf():
    """robot_state_publisher owns the TF tree. The driver must not fight it.

    This used to also assert `i_tf_tf_prefix == 'oak_d'`, and there is no such
    parameter in depthai-ros: the test passed because it read the same YAML
    the config wrote, never the driver. An undeclared parameter is not an
    error in ROS 2 -- it is kept as an initial value and never applied -- so
    the prefix came from somewhere else entirely and nothing said so.

    What sets it is the NODE NAME, via sensor_helpers.cpp::tfPrefix(), which
    returns node->get_name() whenever publishing TF from calibration is off.
    test_launch_descriptions.py pins that name.
    """
    for path in CONFIG_DIR.glob('oak_d_s2*.yaml'):
        camera = yaml.safe_load(path.read_text())['/**']['ros__parameters']['camera']
        assert camera['i_publish_tf_from_calibration'] is False, path.name
        assert 'i_tf_tf_prefix' not in camera, (
            f'{path.name} sets i_tf_tf_prefix, which depthai-ros does not declare. '
            'It is silently ignored; the prefix comes from the node name.'
        )


def test_twist_mux_prefers_teleop_over_navigation():
    """Teleop outranks navigation.

    A human reaching for the controller wants the rover to stop doing what it is doing.
    """
    topics = yaml.safe_load((CONFIG_DIR / 'twist_mux.yaml').read_text())['twist_mux'][
        'ros__parameters'
    ]['topics']
    assert topics['joystick']['priority'] > topics['navigation']['priority']


def test_teleop_deadman_is_required():
    """The joystick deadman is mandatory.

    The 2026 launch file set require_enable_button false, so the rover drove whenever the stick
    moved.

    On 25 kg with brake-mode motors that is a safety property, not a preference.
    """
    params = yaml.safe_load((CONFIG_DIR / 'teleop_switch_pro.yaml').read_text())
    assert params['teleop_twist_joy_node']['ros__parameters']['require_enable_button'] is True

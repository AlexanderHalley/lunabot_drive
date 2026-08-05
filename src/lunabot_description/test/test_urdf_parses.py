#!/usr/bin/env python3
"""Check that lunabot.urdf.xacro expands correctly for every hardware target.

This is the cheapest test in the workspace and it catches the most annoying
class of bug: a model that is valid XML, valid URDF, and quietly wrong --
wired to the wrong plugin, missing a frame something downstream needs, or
with a wheel joint renamed out from under controllers.yaml.

Runs in CI with no ROS graph, no hardware and no GPU.
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

XACRO = Path(__file__).resolve().parent.parent / 'urdf' / 'lunabot.urdf.xacro'

# From docs/TOPIC_FRAME_CONTRACT.md. If this list and that document ever
# disagree, the document wins and this list is the bug.
EXPECTED_LINKS = {
    'base_link',
    'base_footprint',
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_left_wheel_link',
    'rear_right_wheel_link',
    'oak_d_link',
    'oak_d_rgb_camera_frame',
    'oak_d_rgb_camera_optical_frame',
    'oak_d_left_camera_frame',
    'oak_d_left_camera_optical_frame',
    'oak_d_right_camera_frame',
    'oak_d_right_camera_optical_frame',
    'oak_d_imu_frame',
}

# These strings appear verbatim in lunabot_bringup/config/controllers.yaml.
WHEEL_JOINTS = {
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
}

EXPECTED_PLUGIN = {
    'mock': 'mock_components/GenericSystem',
    'sim': 'topic_based_ros2_control/TopicBasedSystem',
    'real': 'lunabot_hardware/SparkFlexSystem',
}


def expand(**args):
    """Run xacro and return the parsed root element."""
    cmd = ['xacro', str(XACRO)] + [f'{k}:={v}' for k, v in args.items()]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    assert result.returncode == 0, f'xacro failed for {args}:\n{result.stderr}'
    return ET.fromstring(result.stdout)


@pytest.fixture(scope='module', params=['mock', 'sim', 'real'])
def hardware(request):
    return request.param


def test_expands_and_is_a_robot(hardware):
    root = expand(hardware=hardware)
    assert root.tag == 'robot'
    assert root.get('name') == 'lunabot'


def test_all_contract_frames_exist(hardware):
    root = expand(hardware=hardware)
    links = {link.get('name') for link in root.findall('link')}
    missing = EXPECTED_LINKS - links
    assert not missing, f'missing frames: {sorted(missing)}'


def test_tree_is_connected_with_base_link_as_root(hardware):
    """Exactly one link has no parent, and it is base_link.

    base_footprint being a CHILD of base_link is inverted from the usual
    convention and is load-bearing: it means exactly one node publishes
    odom -> base_link. If someone "fixes" it back, this fails.
    """
    root = expand(hardware=hardware)
    links = {link.get('name') for link in root.findall('link')}
    children = {j.find('child').get('link') for j in root.findall('joint')}

    roots = links - children
    assert roots == {'base_link'}, f'expected base_link as sole root, got {sorted(roots)}'

    # Every joint's parent must be a link that exists -- catches typos that
    # produce a silently detached subtree.
    parents = {j.find('parent').get('link') for j in root.findall('joint')}
    assert parents <= links, f'joints reference unknown links: {sorted(parents - links)}'


def test_optical_frames_have_the_rep103_rotation(hardware):
    """Optical frames must be rotated -pi/2, 0, -pi/2 from their parent.

    Collapsing or mistyping this puts a 90-degree error into every point
    cloud, which looks like a perception bug for about a day.
    """
    root = expand(hardware=hardware)
    optical_joints = [
        j for j in root.findall('joint') if j.find('child').get('link').endswith('_optical_frame')
    ]
    assert len(optical_joints) == 3, 'expected rgb, left and right optical frames'

    for joint in optical_joints:
        rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
        assert rpy == pytest.approx([-1.5707963, 0.0, -1.5707963], abs=1e-5), (
            f'{joint.get("name")} has rpy {rpy}'
        )


def test_wheels_are_continuous_and_spin_about_y(hardware):
    root = expand(hardware=hardware)
    joints = {j.get('name'): j for j in root.findall('joint')}

    for name in WHEEL_JOINTS:
        assert name in joints, f'{name} missing -- controllers.yaml references it by this string'
        joint = joints[name]
        assert joint.get('type') == 'continuous', f'{name} must be continuous, not revolute'
        axis = [float(v) for v in joint.find('axis').get('xyz').split()]
        assert axis == pytest.approx([0.0, 1.0, 0.0]), f'{name} axis is {axis}'


def test_correct_hardware_plugin_is_selected(hardware):
    root = expand(hardware=hardware)
    control = root.find('ros2_control')
    assert control is not None, 'no <ros2_control> block'

    plugin = control.find('hardware/plugin')
    assert plugin is not None, 'no hardware plugin declared'
    assert plugin.text.strip() == EXPECTED_PLUGIN[hardware]


def test_exactly_one_hardware_block(hardware):
    """The xacro:if branches must be mutually exclusive.

    Two <hardware> blocks is not a parse error -- controller_manager just
    picks one and the robot behaves inexplicably.
    """
    root = expand(hardware=hardware)
    control = root.find('ros2_control')
    assert len(control.findall('hardware')) == 1


def test_every_wheel_exports_the_interfaces_diff_drive_needs(hardware):
    """velocity command, plus position and velocity state, on all four wheels.

    position state is what allows position_feedback: true in
    controllers.yaml; without it the controller integrates velocity and
    drifts faster.
    """
    root = expand(hardware=hardware)
    control = root.find('ros2_control')
    joints = {j.get('name'): j for j in control.findall('joint')}

    assert set(joints) == WHEEL_JOINTS

    for name, joint in joints.items():
        commands = {c.get('name') for c in joint.findall('command_interface')}
        states = {s.get('name') for s in joint.findall('state_interface')}
        assert commands == {'velocity'}, f'{name} commands {commands}'
        assert states == {'position', 'velocity'}, f'{name} states {states}'


def test_can_ids_are_unique_and_match_the_2026_wiring(hardware):
    """The 2026 launch file and drive_node.cpp disagreed about CAN IDs.

    The launch file won, because it is what actually drove the robot after
    "Updated pi wheel allocation for proper turning": left front 2, right
    front 1. Duplicated IDs would put two motors on one address.
    """
    root = expand(hardware=hardware)
    control = root.find('ros2_control')

    ids = {}
    for joint in control.findall('joint'):
        can_id = joint.find("param[@name='can_id']")
        assert can_id is not None, f'{joint.get("name")} has no can_id'
        ids[joint.get('name')] = int(can_id.text)

    assert len(set(ids.values())) == 4, f'duplicate CAN IDs: {ids}'
    assert ids['front_left_wheel_joint'] == 2
    assert ids['front_right_wheel_joint'] == 1


def test_right_side_is_inverted(hardware):
    root = expand(hardware=hardware)
    control = root.find('ros2_control')

    for joint in control.findall('joint'):
        invert = joint.find("param[@name='invert']").text.strip().lower() == 'true'
        expected = 'right' in joint.get('name')
        assert invert == expected, f'{joint.get("name")} invert={invert}'


def test_use_ros2_control_false_drops_the_control_block():
    """description.launch.py uses this to show geometry without a controller stack."""
    root = expand(hardware='mock', use_ros2_control='false')
    assert root.find('ros2_control') is None
    # ...but the robot itself must survive intact.
    links = {link.get('name') for link in root.findall('link')}
    assert EXPECTED_LINKS <= links


def test_prefix_renames_every_frame():
    root = expand(hardware='mock', prefix='alpha_')
    links = {link.get('name') for link in root.findall('link')}
    assert all(name.startswith('alpha_') for name in links), sorted(links)
    assert {f'alpha_{name}' for name in EXPECTED_LINKS} <= links


def test_sim_topics_are_wired_to_the_contract_names():
    """These must match the OmniGraph nodes in lunabot_sim/graphs/joints.py."""
    root = expand(hardware='sim')
    hardware = root.find('ros2_control/hardware')
    params = {p.get('name'): p.text.strip() for p in hardware.findall('param')}
    assert params['joint_states_topic'] == '/isaac/joint_states'
    assert params['joint_commands_topic'] == '/isaac/joint_commands'


def test_real_hardware_defaults_to_can0_and_is_overridable():
    default = expand(hardware='real')
    params = {
        p.get('name'): p.text.strip() for p in default.findall('ros2_control/hardware/param')
    }
    assert params['can_interface'] == 'can0'

    # vcan0 is how the plugin gets exercised with no motors attached.
    virtual = expand(hardware='real', can_interface='vcan0')
    params = {
        p.get('name'): p.text.strip() for p in virtual.findall('ros2_control/hardware/param')
    }
    assert params['can_interface'] == 'vcan0'


def test_all_links_have_inertia_except_pure_frames(hardware):
    """Isaac reads inertia straight from the URDF.

    A body link with no inertial block gets a solver default and behaves
    bizarrely -- sinking, jittering, or launching itself. Pure coordinate
    frames (base_footprint, the camera frames) correctly have none, because
    giving them mass would add phantom links to the articulation.
    """
    root = expand(hardware=hardware)
    bodies = {
        'base_link',
        'front_left_wheel_link',
        'front_right_wheel_link',
        'rear_left_wheel_link',
        'rear_right_wheel_link',
        'oak_d_link',
    }

    for link in root.findall('link'):
        has_inertial = link.find('inertial') is not None
        assert has_inertial == (link.get('name') in bodies), (
            f'{link.get("name")}: inertial={has_inertial}'
        )


def test_positive_definite_inertia(hardware):
    """Non-positive diagonal terms make the physics solver produce nonsense."""
    root = expand(hardware=hardware)
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        assert float(inertial.find('mass').get('value')) > 0.0, link.get('name')
        inertia = inertial.find('inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(inertia.get(axis)) > 0.0, f'{link.get("name")}.{axis}'

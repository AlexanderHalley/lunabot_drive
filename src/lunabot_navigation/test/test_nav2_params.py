#!/usr/bin/env python3
# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Structural checks on nav2_params.yaml, and on what it claims about others.

Nav2's parameter file restates values that are owned elsewhere: the footprint
is the chassis from lunabot_description, the velocity limits are bounded by
diff_drive_controller's, the observation ranges are bounded by the camera's
depth clip. Every one of those was a comment saying "keep this in sync", and a
comment has never kept anything in sync.

Each restatement fails loudly here instead. The failure modes they guard
against are all quiet ones -- a footprint smaller than the robot wedges it, a
velocity limit above the controller's is silently clamped, an observation
range past the sensor clears space that was never observed.

Pure YAML. No ROS, no Nav2 install -- so this also runs standalone in the CI
job that has neither.
"""

import ast
import re
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
PARAMS = PKG / 'config' / 'nav2_params.yaml'

# Sibling packages in the same colcon src/ tree. These reads are the point of
# the file: a copy that cannot see its original cannot be checked against it.
SRC = PKG.parent
CONTROLLERS = SRC / 'lunabot_bringup' / 'config' / 'controllers.yaml'
TWIST_MUX = SRC / 'lunabot_bringup' / 'config' / 'twist_mux.yaml'
CAMERA = SRC / 'lunabot_bringup' / 'config' / 'oak_d_s2.yaml'
PROPERTIES = SRC / 'lunabot_description' / 'urdf' / 'common' / 'properties.xacro'

# Nav2 nodes that publish a velocity command. All three must agree on the
# message type or the chain breaks silently -- see test_the_whole_velocity
# _chain_is_stamped below.
VELOCITY_PUBLISHERS = ['controller_server', 'behavior_server', 'velocity_smoother']


def load(path):
    assert path.is_file(), (
        f'{path} not found. This test reads its sibling packages out of the '
        f'colcon src/ tree; it cannot check a copy against an original it '
        f'cannot see.'
    )
    return yaml.safe_load(path.read_text())


def params():
    return load(PARAMS)


def node_params(name):
    return params()[name]['ros__parameters']


def launch_constant(name):
    """Read a module-level constant without importing the launch file.

    Read a module-level constant out of navigation.launch.py without importing it.

    Importing would pull in nav2_common and launch_ros, neither of which is installed in the
    standalone CI job this file also runs in. NAV2_NODES is written as a literal so that the AST is
    enough -- test_navigation_launch.py does the same checks against the real imported module.
    """
    tree = ast.parse((PKG / 'launch' / 'navigation.launch.py').read_text())
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in statement.targets
        ):
            return ast.literal_eval(statement.value)
    raise AssertionError(f'{name} not found in navigation.launch.py')


def started_nodes():
    """Names of the Nav2 nodes navigation.launch.py starts."""
    return [node['name'] for node in launch_constant('NAV2_NODES')]


def blocks(node=None, path=''):
    """Yield (path, ros__parameters dict) for every node in the file.

    Costmaps nest a level deeper than everything else, so walking beats
    enumerating.
    """
    node = params() if node is None else node
    for key, value in node.items():
        if key == 'ros__parameters':
            yield path, value
        elif isinstance(value, dict):
            yield from blocks(value, f'{path}/{key}')


def observation_layers():
    """Yield (path, source dict) for every costmap observation source.

    A costmap layer holds `observation_sources: pointcloud` and then a
    `pointcloud:` dict beside it, one level below ros__parameters -- so this
    descends into the block rather than reading keys off it.
    """
    for path, block in blocks():
        for layer_name, layer in block.items():
            if not isinstance(layer, dict) or 'observation_sources' not in layer:
                continue
            for source in str(layer['observation_sources']).split():
                yield f'{path}/{layer_name}/{source}', layer[source]


def xacro_property(name):
    """Read one <xacro:property> value out of properties.xacro.

    Regex rather than running xacro: this test must run in a job with no ROS
    installed, and the properties in question are literal numbers.
    """
    text = load_text(PROPERTIES)
    match = re.search(rf'<xacro:property\s+name="{name}"\s+value="([-\d.]+)"', text)
    assert match, f'{name} not found in {PROPERTIES.name}'
    return float(match.group(1))


def load_text(path):
    assert path.is_file(), f'{path} not found'
    return path.read_text()


def test_parses_and_every_node_has_parameters():
    config = params()
    assert config, 'nav2_params.yaml is empty'
    assert dict(blocks()), 'no ros__parameters blocks found'


def test_every_block_states_use_sim_time():
    """Every parameter block sets use_sim_time explicitly.

    navigation.launch.py rewrites use_sim_time with nav2_common's RewrittenYaml, which REPLACES
    existing keys and does not add missing ones.

    A block without the key keeps the wall clock under hw:=sim. For a costmap that means it stops
    updating, and Nav2 reports a robot that is not stuck as permanently stuck.
    """
    missing = [path for path, block in blocks() if 'use_sim_time' not in block]
    assert not missing, f'blocks with no use_sim_time key: {missing}'


def test_frames_match_the_contract():
    """docs/TOPIC_FRAME_CONTRACT.md is authoritative.

    Anything that disagrees with it is a bug, including this file.
    """
    expected = {
        'global_frame': {'map', 'odom'},  # odom for the local costmap only
        'robot_base_frame': {'base_link'},
        'odom_frame_id': {'odom'},
        'base_frame_id': {'base_link'},
        'global_frame_id': {'map'},
        'local_frame': {'odom'},
    }
    for path, block in blocks():
        for key, allowed in expected.items():
            if key in block:
                assert block[key] in allowed, f'{path}: {key}={block[key]!r}'

    config = params()
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_ = config['global_costmap']['global_costmap']['ros__parameters']
    # The local costmap is odom-rooted so it survives a SLAM loop closure
    # jumping the map frame; the global one is map-rooted so it accumulates.
    assert local['global_frame'] == 'odom'
    assert global_['global_frame'] == 'map'


def test_footprint_matches_the_chassis():
    """The footprint is the chassis from properties.xacro, stated twice more.

    A footprint smaller than the robot is how rovers get wedged, and nothing
    about the symptom points at this file.
    """
    length = xacro_property('chassis_length')
    width = xacro_property('chassis_width')
    expected = [
        [length / 2, width / 2],
        [length / 2, -width / 2],
        [-length / 2, -width / 2],
        [-length / 2, width / 2],
    ]

    config = params()
    for name in ['local_costmap', 'global_costmap']:
        footprint = yaml.safe_load(config[name][name]['ros__parameters']['footprint'])
        message = (
            f'{name} footprint does not match the {length} x {width} chassis in properties.xacro'
        )
        assert len(footprint) == len(expected), message
        # Corner by corner: pytest.approx does not descend into nested lists.
        # strict=True is redundant given the length assert above, but it means
        # a future edit that drops that assert fails loudly rather than
        # silently comparing only the shorter list.
        for corner, want in zip(footprint, expected, strict=True):
            assert corner == pytest.approx(want), message


def test_velocity_limits_stay_within_the_controller():
    """Nav2's velocity limits stay inside diff_drive_controller's.

    Nav2 commanding beyond diff_drive_controller's limits is clamped silently, giving a robot that
    does not follow its own plan and no message anywhere saying why.

    Within, not equal: Nav2 is allowed to be more conservative, and currently is -- it plans at
    half the controller's ceiling.
    """
    limits = load(CONTROLLERS)['diff_drive_controller']['ros__parameters']
    follow = node_params('controller_server')['FollowPath']

    assert follow['max_vel_x'] <= limits['linear.x.max_velocity']
    assert follow['min_vel_x'] >= limits['linear.x.min_velocity']
    assert follow['acc_lim_x'] <= limits['linear.x.max_acceleration']

    assert follow['max_vel_theta'] <= limits['angular.z.max_velocity']
    assert follow['min_speed_theta'] >= limits['angular.z.min_velocity']
    assert follow['acc_lim_theta'] <= limits['angular.z.max_acceleration']


def test_the_smoother_agrees_with_the_controller():
    """velocity_smoother's limits must not exceed the controller's.

    velocity_smoother is the last thing to touch a command, so its limits are the ones that
    actually apply.

    Set above the controller's they do nothing; set below they quietly become the real limits and
    the tuning happens in the wrong file.
    """
    follow = node_params('controller_server')['FollowPath']
    smoother = node_params('velocity_smoother')

    assert smoother['max_velocity'][0] == follow['max_vel_x']
    assert smoother['max_velocity'][2] == follow['max_vel_theta']
    assert smoother['min_velocity'][0] == follow['min_vel_x']
    assert smoother['max_accel'][0] == follow['acc_lim_x']
    assert smoother['max_accel'][2] == follow['acc_lim_theta']


def test_nothing_asks_the_skid_steer_to_move_sideways():
    """Y is the strafe axis.

    A differential drive has no such axis, and a non-zero limit invites a planner to produce
    paths the robot cannot follow.
    """
    follow = node_params('controller_server')['FollowPath']
    smoother = node_params('velocity_smoother')

    assert follow['max_vel_y'] == 0.0
    assert follow['min_vel_y'] == 0.0
    assert follow['acc_lim_y'] == 0.0
    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0


@pytest.mark.parametrize('node', VELOCITY_PUBLISHERS)
def test_the_whole_velocity_chain_is_stamped(node):
    """TwistStamped end to end, because twist_mux runs use_stamped: true.

    Get this wrong and the topics still connect, nothing errors, and the rover
    does not move -- the failure docs/TOPIC_FRAME_CONTRACT.md budgets an
    afternoon for. Pinned against twist_mux.yaml so changing one side alone
    fails here rather than on the robot.
    """
    twist_mux = load(TWIST_MUX)['twist_mux']['ros__parameters']
    assert twist_mux['use_stamped'] is True, 'twist_mux.yaml changed; this file must follow'
    assert node_params(node)['enable_stamped_cmd_vel'] is True


def test_observation_ranges_do_not_exceed_the_camera():
    """The costmap cannot see further than the sensor.

    i_max_range in oak_d_s2.yaml is the depth clip, in millimetres. Claiming
    more range than that marks obstacles from noise and, worse, CLEARS cells
    that were never observed -- which reads as the costmap forgetting real
    obstacles.
    """
    clip_m = load(CAMERA)['/**']['ros__parameters']['stereo']['i_max_range'] / 1000.0

    checked = 0
    for path, layer in observation_layers():
        assert layer['obstacle_max_range'] <= clip_m, path
        assert layer['raytrace_max_range'] <= clip_m, path
        checked += 1

    # The costmap layers nest their sources a level below ros__parameters, so
    # a walk that stops at the block finds nothing and passes. It did.
    assert checked == 2, f'expected both costmaps to be checked, checked {checked}'


def test_amcl_is_configured_but_never_started():
    """Rtabmap owns map -> odom. AMCL is in this file for completeness only.

    Starting it would put a second publisher on map -> odom and give a robot
    that teleports between two beliefs about where it is -- so the managed
    node list is where that has to be prevented, not in a comment.
    """
    assert 'amcl' in params(), 'the block is documentation; keep it'
    assert 'amcl' not in started_nodes()


def test_every_managed_node_is_configured():
    """Every lifecycle-managed node has a parameter block.

    A lifecycle node with no parameter block starts on Nav2's defaults -- wrong frames, wrong
    limits -- and reports nothing unusual while doing it.

    The reverse is fine: amcl is configured and deliberately not managed.
    """
    configured = set(params())
    for node in started_nodes():
        assert node in configured, f'{node} is started but has no parameters'

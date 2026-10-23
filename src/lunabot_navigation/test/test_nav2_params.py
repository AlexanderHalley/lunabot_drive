#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

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
import math
import re
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
PARAMS = PKG / 'config' / 'nav2_params.yaml'
BEHAVIOR_TREES = {
    'default_nav_to_pose_bt_xml': PKG / 'config' / 'navigate_to_pose.xml',
    'default_nav_through_poses_bt_xml': PKG / 'config' / 'navigate_through_poses.xml',
}

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
    """The footprint is the robot's outline from properties.xacro, stated twice more.

    The outline is whichever reaches further, the chassis or the wheels: the
    wheels stand proud of the chassis sides, and a footprint of the chassis
    alone leaves them out. A footprint smaller than the robot is how rovers
    get wedged, and nothing about the symptom points at this file.
    """
    half_length = max(
        xacro_property('chassis_length') / 2,
        xacro_property('wheel_base') / 2 + xacro_property('wheel_radius'),
    )
    half_width = max(
        xacro_property('chassis_width') / 2,
        xacro_property('wheel_separation') / 2 + xacro_property('wheel_width') / 2,
    )
    length, width = 2 * half_length, 2 * half_width
    expected = [
        [half_length, half_width],
        [half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, half_width],
    ]

    config = params()
    for name in ['local_costmap', 'global_costmap']:
        costmap = config[name][name]['ros__parameters']
        if 'robot_radius' in costmap:
            # A rover that turns on the spot sweeps the circle round its
            # outline; the radius must cover it, and not by so much that every
            # gap closes.
            sweep = math.hypot(half_length, half_width)
            assert sweep <= costmap['robot_radius'] <= sweep + 0.10, (
                f'{name} robot_radius {costmap["robot_radius"]} does not cover the '
                f'{sweep:.3f} m pivot sweep of the {length:.3f} x {width:.3f} outline'
            )
            continue
        footprint = yaml.safe_load(costmap['footprint'])
        message = (
            f'{name} footprint does not match the {length:.3f} x {width:.3f} outline of the '
            'chassis and wheels in properties.xacro'
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

    Within, not equal: Nav2 is allowed to be more conservative, and currently is.
    Regulated Pure Pursuit has one speed and no acceleration limits of its own,
    so the smoother's limits are the ones checked against the controller.
    """
    limits = load(CONTROLLERS)['diff_drive_controller']['ros__parameters']
    follow = node_params('controller_server')['FollowPath']
    smoother = node_params('velocity_smoother')

    assert follow['desired_linear_vel'] <= limits['linear.x.max_velocity']
    assert smoother['max_velocity'][0] <= limits['linear.x.max_velocity']
    assert smoother['min_velocity'][0] >= limits['linear.x.min_velocity']
    assert smoother['max_accel'][0] <= limits['linear.x.max_acceleration']

    assert smoother['max_velocity'][2] <= limits['angular.z.max_velocity']
    assert smoother['min_velocity'][2] >= limits['angular.z.min_velocity']
    assert smoother['max_accel'][2] <= limits['angular.z.max_acceleration']


def test_the_smoother_agrees_with_the_controller():
    """velocity_smoother's limits match what the path follower asks for.

    velocity_smoother is the last thing to touch a command, so its limits are the ones that
    actually apply. Set below the follower's speed they quietly become the real limit -- and
    because pure pursuit computes the turn rate for its OWN speed, capping the speed alone
    changes the curvature the rover drives. Reversing is checked too: allow_reversing drives
    backwards at the same speed.
    """
    follow = node_params('controller_server')['FollowPath']
    smoother = node_params('velocity_smoother')

    assert smoother['max_velocity'][0] == follow['desired_linear_vel']
    if follow.get('allow_reversing'):
        assert smoother['min_velocity'][0] == -follow['desired_linear_vel']


def test_nothing_asks_the_skid_steer_to_move_sideways():
    """Y is the strafe axis.

    A differential drive has no such axis, and a non-zero limit invites a planner to produce
    paths the robot cannot follow.
    """
    smoother = node_params('velocity_smoother')

    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0
    assert smoother['max_accel'][1] == 0.0


HYBRID = 'nav2_smac_planner::SmacPlannerHybrid'


def test_turning_in_place_is_all_or_nothing():
    """Either the rover turns on the spot everywhere Nav2 might ask, or nowhere.

    A grid planner (anything but Hybrid-A*) draws corners only a pivot can
    follow, so the follower must rotate to heading and the curvature limiter
    must be off. Hybrid-A* with a turning radius means a rover that cannot
    pivot: then nothing may rotate in place -- not the follower, not a Spin
    recovery. Mixing the two gives a rover that sits with its wheels spinning
    until Nav2 declares no progress.
    """
    follow = node_params('controller_server')['FollowPath']
    planner = node_params('planner_server')['GridBased']
    limiter = node_params('curvature_limiter')
    behaviors = node_params('behavior_server')
    spins = any(behaviors[n]['plugin'] == 'nav2_behaviors::Spin' for n in behaviors['behavior_plugins'])

    if planner['plugin'] == HYBRID:
        assert follow['use_rotate_to_heading'] is False
        assert not spins
        assert limiter['min_turning_radius'] > 0.0
    else:
        assert follow['use_rotate_to_heading'] is True
        assert limiter['min_turning_radius'] == 0.0
        # A pivot sweeps beyond a rectangular footprint; the follower must
        # check the rotation and stop rather than swing into a rock.
        assert follow['use_collision_detection'] is True


def test_rotate_to_heading_and_reversing_are_not_both_on():
    """Nav2 refuses the combination at configure, and nothing else starts."""
    follow = node_params('controller_server')['FollowPath']
    assert not (follow['use_rotate_to_heading'] and follow.get('allow_reversing', False))


@pytest.mark.parametrize('key', sorted(BEHAVIOR_TREES))
def test_every_behaviour_a_tree_calls_is_served(key):
    """A tree calling Spin with no spin server stops Nav2 coming up at all.

    bt_navigator loads both trees at configure and waits for every action
    server they name; seen in Isaac when Spin was removed from the server but
    Nav2's default through-poses tree still called it.
    """
    behaviors = node_params('behavior_server')
    served = {behaviors[n]['plugin'].split('::')[-1] for n in behaviors['behavior_plugins']}
    tree = BEHAVIOR_TREES[key].read_text()
    for behaviour in ('Spin', 'BackUp', 'Wait', 'DriveOnHeading', 'AssistedTeleop'):
        if f'<{behaviour}' in tree:
            assert behaviour in served, f'{BEHAVIOR_TREES[key].name} calls {behaviour}'


def test_reverse_legs_can_be_followed():
    """Reeds-Shepp paths reverse; the follower must too, or the cusps are dead ends."""
    follow = node_params('controller_server')['FollowPath']
    planner = node_params('planner_server')['GridBased']
    if planner.get('motion_model_for_search') == 'REEDS_SHEPP':
        assert follow['allow_reversing'] is True


def test_planner_and_follower_agree_on_the_turning_radius():
    """With Hybrid-A*, one number stated twice: the tightest arc in sand.

    The planner must not plan tighter than it, and the follower slows down
    where the path bends tighter than it.
    """
    follow = node_params('controller_server')['FollowPath']
    planner = node_params('planner_server')['GridBased']
    if planner['plugin'] == HYBRID:
        assert follow['regulated_linear_scaling_min_radius'] == pytest.approx(
            planner['minimum_turning_radius']
        )


@pytest.mark.parametrize('key', sorted(BEHAVIOR_TREES))
def test_every_behaviour_tree_is_ours(key):
    """bt_navigator names our trees, which ship in config/.

    Both, not just navigate-to-pose: bt_navigator loads the through-poses tree
    at configure as well, and Nav2's default calls the Spin server that this
    configuration does not start -- so Nav2 refuses to come up at all. The
    launch file rewrites each value to an absolute path; this checks the name
    and that the file exists to be installed.
    """
    navigator = node_params('bt_navigator')
    assert Path(navigator[key]).name == BEHAVIOR_TREES[key].name
    assert BEHAVIOR_TREES[key].is_file()


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


def test_camera_obstacles_are_cleared_in_3d():
    """Every layer fed by the tilted camera must be a VoxelLayer.

    A 2D ObstacleLayer clears along the ray to every point, projected flat.
    The camera looks down over a 26 cm boulder at the ground beyond it, so
    those rays erase the boulder as fast as it is marked: the global costmap
    sat empty and Hybrid-A* planned straight through the rock.
    """
    checked = 0
    for path, block in blocks():
        for name, layer in block.items():
            if isinstance(layer, dict) and 'observation_sources' in layer:
                assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer', f'{path}/{name}'
                checked += 1
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


def test_every_dwb_controller_has_critics():
    """A DWB controller without critics cannot configure, so Nav2 never starts.

    controller_server throws "No critics defined" in on_configure, the
    lifecycle manager aborts the whole bringup, and every Nav2 node sits
    unconfigured. Nothing else in CI notices: the launch description is fine.
    """
    controller = params()['controller_server']['ros__parameters']
    for name in controller['controller_plugins']:
        plugin = controller[name]
        if plugin['plugin'] != 'dwb_core::DWBLocalPlanner':
            continue
        critics = plugin.get('critics') or []
        assert critics, f'{name}: DWB with no critics list'
        assert plugin['max_speed_xy'] <= plugin['max_vel_x'] + 1e-9, (
            f'{name}: max_speed_xy above max_vel_x'
        )


def test_the_costmaps_see_low_rocks():
    """Obstacle heights are measured from base_link -- the axle, not the ground.

    base_link is wheel_radius above the ground (and the wheels sink a few cm
    more), so a voxel grid starting at z = 0, or a minimum obstacle height
    above 0, ignores everything shorter than the axle. In Isaac the rover
    drove straight into every rock under ~20 cm.
    """
    ground = -xacro_property('wheel_radius')
    for path, block in blocks():
        for name, layer in block.items():
            if not isinstance(layer, dict) or 'observation_sources' not in layer:
                continue
            assert layer['origin_z'] < ground, f'{path}/{name}: voxels start above the ground'
            # The layer's own threshold (default 0.0) applies on top of the
            # source's, so both must be set.
            low = layer['min_obstacle_height']
            assert ground + 0.03 <= low <= ground + 0.10, f'{path}/{name}: layer-level {low}'
            top = layer['origin_z'] + layer['z_resolution'] * layer['z_voxels']
            assert top >= layer['max_obstacle_height'], f'{path}/{name}: grid too short'
            for source in str(layer['observation_sources']).split():
                low = layer[source]['min_obstacle_height']
                assert ground + 0.03 <= low <= ground + 0.10, (
                    f'{path}/{name}/{source}: min_obstacle_height {low} is not a few cm '
                    f'above the ground at {ground}'
                )


def test_no_costmap_forgets_a_rock():
    """Every camera layer marks but does not clear.

    Clearing rays over a low rock erase its top voxel as soon as the rock is
    too close to be seen: the planner then routed straight over it (global),
    and the follower's collision check drove into it (local).
    """
    checked = 0
    for path, block in blocks():
        for name, layer in block.items():
            if isinstance(layer, dict) and 'observation_sources' in layer:
                for source in str(layer['observation_sources']).split():
                    assert layer[source]['marking'] is True, f'{path}/{name}/{source}'
                    assert layer[source]['clearing'] is False, f'{path}/{name}/{source}'
                    checked += 1
    assert checked == 2


@pytest.mark.parametrize('name', ['local_costmap', 'global_costmap'])
def test_inflation_covers_the_robot(name):
    """Inflation reaches past the footprint's farthest corner.

    Otherwise cost only starts rising once a corner is already touching a
    rock, and a pivot -- which sweeps every corner round -- gets no warning.
    """
    costmap = params()[name][name]['ros__parameters']
    if 'robot_radius' in costmap:
        reach = costmap['robot_radius']
    else:
        pad = costmap.get('footprint_padding', 0.0)
        reach = max(math.hypot(abs(x) + pad, abs(y) + pad) for x, y in yaml.safe_load(costmap['footprint']))
    assert costmap['inflation_layer']['inflation_radius'] >= reach


def test_lattice_primitives_match_the_costmap():
    """A State Lattice planner needs primitives at the global costmap's resolution."""
    planner = node_params('planner_server')['GridBased']
    if planner['plugin'] != 'nav2_smac_planner::SmacPlannerLattice':
        return
    resolution_cm = round(params()['global_costmap']['global_costmap']['ros__parameters']['resolution'] * 100)
    assert planner['lattice_filepath'].startswith(f'{resolution_cm}cm_resolution/')
    assert '/diff/' in planner['lattice_filepath'], 'differential-drive primitives (rotate in place)'
    launch = (PKG / 'launch' / 'navigation.launch.py').read_text()
    assert "'lattice_filepath'" in launch, 'the launch must rewrite it to the installed path'
    if planner.get('allow_reverse_expansion'):
        assert node_params('controller_server')['FollowPath'].get('allow_reversing') is True


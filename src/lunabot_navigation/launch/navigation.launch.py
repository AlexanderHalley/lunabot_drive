# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Nav2, configured for the Lunabot rover.

    ros2 launch lunabot_bringup robot.launch.py hw:=mock slam:=rtabmap nav:=true

Nav2 output goes to /cmd_vel_nav, never to /cmd_vel. twist_mux arbitrates, and
teleop outranks navigation: a human reaching for the controller is a human who
wants the rover to stop doing what it is doing.

That single sentence is why this file starts the Nav2 nodes itself instead of
including one of nav2_bringup's launch files.

  - bringup_launch.py also starts AMCL and map_server. rtabmap already owns
    map -> odom and publishes /map. Two publishers on map -> odom gives a robot
    that teleports between two beliefs about where it is.

  - navigation_launch.py is closer, but it remaps velocity_smoother's output
    `cmd_vel_smoothed` to **`cmd_vel`** -- which is twist_mux's OUTPUT topic.
    Including it would put Nav2 straight onto the wire beside twist_mux, so the
    priority in twist_mux.yaml would arbitrate between teleop and nothing while
    Nav2 drove the robot regardless. Remappings inside an included description
    cannot be cleanly overridden from outside it, so the nodes are declared
    here where the remappings are ours.

The chain, and the reason for the middle topic:

    controller_server -+
                       +--> /cmd_vel_nav_unsmoothed --> velocity_smoother
    behavior_server  --+                                       |
                                                               v
                                                        /cmd_vel_nav
                                                               |
                                             twist_mux <-------+

Everything Nav2 commands passes through the smoother, so /cmd_vel_nav has
exactly one publisher. Recovery behaviours go through it too -- a backup that
steps around the acceleration limits is a backup that spins the wheels on
regolith and buries them.

There is no use_composition option. The composable variants of these nodes
need the same remappings stated a second time, and duplicating them to save
process overhead on a stack that has never been run is how the two copies
drift apart. Worth revisiting when this moves onto the Jetson and the
serialisation cost between costmap and controller is measurable.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

# The whole Nav2 stack, as data rather than as five near-identical Node()
# calls. Two reasons: the remappings are the contract with twist_mux and are
# worth reading in one place, and a test can assert on this without reaching
# into launch_ros internals to find out what a Node was constructed with.
#
# DECLARATION ORDER IS ACTIVATION ORDER. The controller and the planner come
# up before bt_navigator, which starts sending them goals the moment it is
# active.
#
# VERIFY on first bring-up: executable names have moved between Nav2 releases
# -- behavior_server was recoveries_server before Humble. If a node is
# silently absent, check the installed package first.
NAV2_NODES = (
    {
        'package': 'nav2_controller',
        'executable': 'controller_server',
        'name': 'controller_server',
        # Into the smoother, not onto the bus.
        'remappings': [('cmd_vel', 'cmd_vel_nav_unsmoothed')],
    },
    {
        'package': 'nav2_planner',
        'executable': 'planner_server',
        'name': 'planner_server',
        'remappings': [],
    },
    {
        'package': 'nav2_behaviors',
        'executable': 'behavior_server',
        'name': 'behavior_server',
        # Recoveries are smoothed like everything else.
        'remappings': [('cmd_vel', 'cmd_vel_nav_unsmoothed')],
    },
    {
        'package': 'nav2_bt_navigator',
        'executable': 'bt_navigator',
        'name': 'bt_navigator',
        'remappings': [],
    },
    {
        'package': 'nav2_velocity_smoother',
        'executable': 'velocity_smoother',
        'name': 'velocity_smoother',
        'remappings': [
            ('cmd_vel', 'cmd_vel_nav_unsmoothed'),
            ('cmd_vel_smoothed', 'cmd_vel_nav'),
        ],
    },
)

# Derived, not restated. An unmanaged lifecycle node sits in `unconfigured`
# forever, producing no output and no error -- so "started but not managed"
# should not be a state this file can express.
LIFECYCLE_NODES = [node['name'] for node in NAV2_NODES]

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_navigation'), 'config', 'nav2_params.yaml']
        ),
        description='Nav2 parameter file.',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Bring the Nav2 lifecycle nodes up automatically.',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # RewrittenYaml rather than a second parameters entry, because use_sim_time
    # has to reach the costmaps too. The costmaps are sub-nodes of
    # controller_server and planner_server with their own parameter sections,
    # and a {'use_sim_time': ...} dict passed to the node only lands on the
    # node's own namespace. A costmap left on the wall clock under sim stops
    # updating, and Nav2 reports the robot as permanently stuck.
    #
    # The rewrite REPLACES keys, it does not add them: every ros__parameters
    # block in nav2_params.yaml states use_sim_time for this to bite.
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    nodes = [
        Node(
            package=node['package'],
            executable=node['executable'],
            name=node['name'],
            output='screen',
            parameters=[params],
            remappings=node['remappings'],
        )
        for node in NAV2_NODES
    ]

    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': LIFECYCLE_NODES,
            }
        ],
    )

    return LaunchDescription(ARGUMENTS + nodes + [manager])

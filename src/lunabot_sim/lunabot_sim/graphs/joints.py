# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The ros2_control bridge: /isaac/joint_states and /isaac/joint_commands.

This graph is what makes sim a real test of the robot's control stack rather
than a separate simulation of it. On the ROS side,
topic_based_ros2_control/TopicBasedSystem publishes commands here and reads
states back, so the IDENTICAL controller_manager, joint_state_broadcaster and
diff_drive_controller run with the identical controllers.yaml -- only the
hardware plugin differs.

The topic names must match the sim_joint_states_topic and
sim_joint_commands_topic arguments in lunabot.urdf.xacro.

==================== THE ANGLE-INSTEAD-OF-RATE BUG ====================
ROS2SubscribeJointState chooses between position and velocity targets by which
arrays in the incoming JointState are non-empty, so a message carrying both
may drive the wheels to an absolute angle rather than at a rate -- the rover
snapping to a heading and stopping.

This was written as the LIKELY failure. It is not, and the reason is worth
keeping: TopicBasedSystem::write() pushes an array only for the command
interfaces a joint actually declares --

    if (interface.name == HW_IF_POSITION)      joint_state.position.push_back(...)
    else if (interface.name == HW_IF_VELOCITY) joint_state.velocity.push_back(...)

-- and lunabot.ros2_control.xacro declares velocity and nothing else. The
position array arrives empty, so the node picks velocity. test_sim_bringup.py
asserts exactly that against the plugin, so if it ever stops being true,
something fails in CI rather than on regolith.

The two defences stay because they cost nothing and they are what makes adding
a position command interface later a slow rover rather than a broken one:
wheel drives get zero stiffness in articulation.py, so a position target
cannot be held, and positionCommand is left unwired below.
======================================================================
"""

from __future__ import annotations

from lunabot_sim import compat
from lunabot_sim.graphs import context

STATES_TOPIC = 'isaac/joint_states'
COMMANDS_TOPIC = 'isaac/joint_commands'


def build(
    articulation_path: str,
    graph_path: str = '/World/Graphs/Joints',
    domain_id: int = context.DEFAULT_DOMAIN_ID,
):
    import omni.graph.core as og

    keys = og.Controller.Keys
    ctx_node, ctx_values = context.ros2_context_node(domain_id)

    return context.create_graph(
        graph_path,
        keys,
        nodes=[
            context.playback_tick_node(),
            ctx_node,
            context.simulation_time_node(),
            ('PublishJointState', compat.og_node_type('ROS2PublishJointState')),
            ('SubscribeJointState', compat.og_node_type('ROS2SubscribeJointState')),
            ('ArticulationController', compat.core_node_type('IsaacArticulationController')),
        ],
        values=ctx_values
        + [
            ('PublishJointState.inputs:topicName', STATES_TOPIC),
            ('PublishJointState.inputs:targetPrim', [articulation_path]),
            ('SubscribeJointState.inputs:topicName', COMMANDS_TOPIC),
            ('ArticulationController.inputs:targetPrim', [articulation_path]),
        ],
        connections=[
            ('OnTick.outputs:tick', 'PublishJointState.inputs:execIn'),
            ('OnTick.outputs:tick', 'SubscribeJointState.inputs:execIn'),
            ('SubscribeJointState.outputs:execOut', 'ArticulationController.inputs:execIn'),
            ('Context.outputs:context', 'PublishJointState.inputs:context'),
            ('Context.outputs:context', 'SubscribeJointState.inputs:context'),
            ('SimTime.outputs:simulationTime', 'PublishJointState.inputs:timeStamp'),
            (
                'SubscribeJointState.outputs:jointNames',
                'ArticulationController.inputs:jointNames',
            ),
            (
                'SubscribeJointState.outputs:velocityCommand',
                'ArticulationController.inputs:velocityCommand',
            ),
            # positionCommand is deliberately NOT connected. Leaving it
            # unwired means a position array arriving from TopicBasedSystem
            # reaches nothing, which is the second half of the defence
            # described in the module docstring.
        ],
    )

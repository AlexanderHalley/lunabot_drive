"""The ros2_control bridge: /isaac/joint_states and /isaac/joint_commands.

This graph is what makes sim a real test of the robot's control stack rather
than a separate simulation of it. On the ROS side,
topic_based_ros2_control/TopicBasedSystem publishes commands here and reads
states back, so the IDENTICAL controller_manager, joint_state_broadcaster and
diff_drive_controller run with the identical controllers.yaml -- only the
hardware plugin differs.

The topic names must match the sim_joint_states_topic and
sim_joint_commands_topic arguments in lunabot.urdf.xacro.

==================== THE LIKELY BUG ====================
ROS2SubscribeJointState chooses between position and velocity targets based on
which arrays in the incoming JointState are non-empty, and TopicBasedSystem
populates both. The rover may therefore try to drive to an absolute wheel
angle instead of spinning at a rate.

articulation.py defends against this by giving the wheel drives zero
stiffness, so a position target physically cannot be held. If the rover still
snaps to an angle and stops, look there first.
========================================================
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

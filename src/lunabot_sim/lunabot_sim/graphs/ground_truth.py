# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Ground truth: the rover's true pose, on its own topic.

==================== NEVER GOES INTO /tf ====================
Isaac publishes the true pose on /sim/ground_truth/odom and NOTHING else.
It does not publish TF.

That is the decision that keeps sim honest. robot_state_publisher builds
base_link -> * from the same URDF in sim and on hardware, and
diff_drive_controller publishes odom -> base_link in both. If Isaac also
published transforms, there would be two publishers on the same edges and a
tree that works only in sim -- so every TF-related bug would be invisible
until the robot was on regolith.

Ground truth is for SCORING, not for navigating. Nothing in the autonomy stack
may subscribe to it. In RViz, display it alongside /odom: the gap between the
two trails IS the odometry error, which is the most direct read there is on
whether a change helped.
=============================================================
"""

from __future__ import annotations

from lunabot_sim import compat
from lunabot_sim.graphs import context

TOPIC = 'sim/ground_truth/odom'


def build(
    articulation_path: str,
    graph_path: str = '/World/Graphs/GroundTruth',
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
            ('ComputeOdometry', compat.core_node_type('IsaacComputeOdometry')),
            ('PublishOdometry', compat.og_node_type('ROS2PublishOdometry')),
        ],
        values=ctx_values
        + [
            ('ComputeOdometry.inputs:chassisPrim', [articulation_path]),
            ('PublishOdometry.inputs:topicName', TOPIC),
            # Frames chosen to be readable in RViz next to the real /odom
            # while remaining outside the TF tree: nothing publishes a
            # transform for either of these names, so they cannot be
            # confused for the real odom -> base_link edge.
            ('PublishOdometry.inputs:odomFrameId', 'sim_ground_truth'),
            ('PublishOdometry.inputs:chassisFrameId', 'sim_base_link'),
        ],
        connections=[
            ('OnTick.outputs:tick', 'ComputeOdometry.inputs:execIn'),
            ('ComputeOdometry.outputs:execOut', 'PublishOdometry.inputs:execIn'),
            ('Context.outputs:context', 'PublishOdometry.inputs:context'),
            ('SimTime.outputs:simulationTime', 'PublishOdometry.inputs:timeStamp'),
            ('ComputeOdometry.outputs:position', 'PublishOdometry.inputs:position'),
            ('ComputeOdometry.outputs:orientation', 'PublishOdometry.inputs:orientation'),
            ('ComputeOdometry.outputs:linearVelocity', 'PublishOdometry.inputs:linearVelocity'),
            ('ComputeOdometry.outputs:angularVelocity', 'PublishOdometry.inputs:angularVelocity'),
        ],
    )

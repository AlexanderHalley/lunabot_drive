"""Publish /clock.

The most important graph in the package, and the one to check first when the
stack appears hung: with use_sim_time:=true and no /clock, every ROS node
blocks at time zero. Nothing errors. Nothing logs. It just sits there.

Start Isaac BEFORE the ROS bringup, or wrap the bringup in a wait-for-clock.
"""

from __future__ import annotations

from lunabot_sim import compat
from lunabot_sim.graphs import context


def build(graph_path: str = '/World/Graphs/Clock', domain_id: int = context.DEFAULT_DOMAIN_ID):
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
            ('PublishClock', compat.og_node_type('ROS2PublishClock')),
        ],
        values=ctx_values + [('PublishClock.inputs:topicName', 'clock')],
        connections=[
            ('OnTick.outputs:tick', 'PublishClock.inputs:execIn'),
            ('Context.outputs:context', 'PublishClock.inputs:context'),
            ('SimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
        ],
    )

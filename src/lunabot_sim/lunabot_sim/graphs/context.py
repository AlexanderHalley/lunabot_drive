"""Shared OmniGraph helpers.

Each concern gets its OWN graph rather than all of them sharing one. That is
deliberate: a graph that fails to evaluate takes down everything in it, and
losing the camera should not stop /clock. Without a clock, every ROS node with
use_sim_time blocks at time zero and the whole stack appears hung -- a far
worse failure than a missing image topic.
"""

from __future__ import annotations

import logging

from lunabot_sim import compat

logger = logging.getLogger(__name__)

# Must match ROS_DOMAIN_ID on every other machine. The 2026 setup used 42 and
# there is no reason to change it.
DEFAULT_DOMAIN_ID = 42


def create_graph(path: str, keys, nodes, values=None, connections=None):
    """Create an OmniGraph and evaluate it on playback tick.

    Wraps og.Controller.edit so callers do not each repeat the pipeline
    stage boilerplate.
    """
    import omni.graph.core as og

    graph_spec = {
        'graph_path': path,
        # ON_PLAYBACK_TICK: evaluate only while the simulation is playing.
        # Not ON_TICK, which also fires while paused and produces sensor
        # messages stamped with a clock that is not advancing.
        'evaluator_name': 'push',
        'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    }

    spec = {keys.CREATE_NODES: nodes}
    if values:
        spec[keys.SET_VALUES] = values
    if connections:
        spec[keys.CONNECT] = connections

    graph, _, _, _ = og.Controller.edit(graph_spec, spec)
    logger.info('created graph %s with %d nodes', path, len(nodes))
    return graph


def ros2_context_node(domain_id: int = DEFAULT_DOMAIN_ID):
    """Node spec and values for a ROS2Context.

    Every graph that publishes needs one. Sharing a single context across
    graphs would couple them, which is the thing this layout is avoiding.
    """
    node = ('Context', compat.og_node_type('ROS2Context'))
    values = [
        ('Context.inputs:domain_id', domain_id),
        # False: use the domain_id above rather than ROS_DOMAIN_ID from the
        # environment. Isaac is often launched from a shell that has not
        # sourced the ROS setup, and silently landing on domain 0 while every
        # other machine is on 42 is a confusing way to see no topics at all.
        ('Context.inputs:useDomainIDEnvVar', False),
    ]
    return node, values


def playback_tick_node():
    """The tick source. Fires each simulation step while playing."""
    return ('OnTick', 'omni.graph.action.OnPlaybackTick')


def simulation_time_node():
    """Reads the simulated clock.

    Feeds every publisher's timestamp. Using the wall clock instead would
    stamp messages with a time unrelated to /clock, and every downstream
    time-synchronised subscriber would silently drop everything.
    """
    return ('SimTime', compat.core_node_type('IsaacReadSimulationTime'))

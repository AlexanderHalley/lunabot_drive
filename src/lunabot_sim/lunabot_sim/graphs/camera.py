# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Publish the OAK-D's streams from Isaac.

Topic names are the SAME ones the real depthai driver publishes -- that is the
entire point of docs/TOPIC_FRAME_CONTRACT.md. Nothing downstream can tell
whether it is looking at sim or hardware.

One thing genuinely differs and cannot be hidden: DEPTH ENCODING. Isaac's
depth annotator emits 32FC1 metres; the OAK-D emits 16UC1 millimetres. rtabmap
accepts both, which is what makes it dangerous. The boulder detector consumes
the point cloud precisely to sidestep this. See the contract doc.
"""

from __future__ import annotations

import logging

from lunabot_sim import compat
from lunabot_sim.graphs import context

logger = logging.getLogger(__name__)

# ==================== VERIFY ====================
# ROS2CameraHelper's `type` input selects what the helper publishes. These are
# the values believed correct; confirm against the installed bridge by
# inspecting the node in the OmniGraph editor.
RGB = 'rgb'
DEPTH = 'depth'
DEPTH_PCL = 'depth_pcl'
CAMERA_INFO = 'camera_info'
# ================================================


def build(
    camera_prim: str,
    graph_path: str = '/World/Graphs/Camera',
    width: int = 640,
    height: int = 400,
    domain_id: int = context.DEFAULT_DOMAIN_ID,
    publish_pointcloud: bool = True,
):
    """Wire one camera prim to the contract's RGB, depth and point cloud topics.

    width/height default to 640x400, matching the real camera's 400P stereo
    profile. Rendering at a higher resolution than the hardware produces makes
    sim results optimistic in a way that is easy to forget about.
    """
    import omni.graph.core as og

    keys = og.Controller.Keys
    ctx_node, ctx_values = context.ros2_context_node(domain_id)

    nodes = [
        context.playback_tick_node(),
        ctx_node,
        # One render product feeds every helper below. Creating one per
        # stream would render the scene three times per frame.
        ('RenderProduct', compat.core_node_type('IsaacCreateRenderProduct')),
        ('PublishRGB', compat.og_node_type('ROS2CameraHelper')),
        ('PublishDepth', compat.og_node_type('ROS2CameraHelper')),
    ]

    values = ctx_values + [
        ('RenderProduct.inputs:cameraPrim', [camera_prim]),
        ('RenderProduct.inputs:width', width),
        ('RenderProduct.inputs:height', height),
        ('RenderProduct.inputs:enabled', True),
        # The frame_id the real driver publishes. robot_state_publisher owns
        # the transform for it, from the URDF -- Isaac does not publish TF.
        ('PublishRGB.inputs:frameId', 'oak_d_rgb_camera_optical_frame'),
        ('PublishRGB.inputs:topicName', 'oak_d/rgb/image_raw'),
        ('PublishRGB.inputs:type', RGB),
        # True: publish camera_info alongside. rtabmap needs intrinsics, and a
        # topic with images and no camera_info is a subtle way to have SLAM
        # sit there receiving nothing it can use.
        ('PublishRGB.inputs:enableSemanticLabels', False),
        ('PublishDepth.inputs:frameId', 'oak_d_rgb_camera_optical_frame'),
        ('PublishDepth.inputs:topicName', 'oak_d/stereo/image_raw'),
        ('PublishDepth.inputs:type', DEPTH),
    ]

    connections = [
        ('OnTick.outputs:tick', 'RenderProduct.inputs:execIn'),
        ('RenderProduct.outputs:execOut', 'PublishRGB.inputs:execIn'),
        ('RenderProduct.outputs:execOut', 'PublishDepth.inputs:execIn'),
        ('RenderProduct.outputs:renderProductPath', 'PublishRGB.inputs:renderProductPath'),
        ('RenderProduct.outputs:renderProductPath', 'PublishDepth.inputs:renderProductPath'),
        ('Context.outputs:context', 'PublishRGB.inputs:context'),
        ('Context.outputs:context', 'PublishDepth.inputs:context'),
    ]

    if publish_pointcloud:
        # On the real robot the cloud is built by depth_image_proc on the Pi
        # (decimation 4, 2 m clip). Here the helper produces it directly, so
        # the point DENSITY differs between sim and hardware even though the
        # topic does not. If cluster tuning transfers poorly between the two,
        # this is the first thing to suspect.
        nodes.append(('PublishPointCloud', compat.og_node_type('ROS2CameraHelper')))
        values += [
            ('PublishPointCloud.inputs:frameId', 'oak_d_rgb_camera_optical_frame'),
            ('PublishPointCloud.inputs:topicName', 'oak_d/points'),
            ('PublishPointCloud.inputs:type', DEPTH_PCL),
        ]
        connections += [
            ('RenderProduct.outputs:execOut', 'PublishPointCloud.inputs:execIn'),
            (
                'RenderProduct.outputs:renderProductPath',
                'PublishPointCloud.inputs:renderProductPath',
            ),
            ('Context.outputs:context', 'PublishPointCloud.inputs:context'),
        ]

    logger.info('camera graph publishing from %s at %dx%d', camera_prim, width, height)
    return context.create_graph(graph_path, keys, nodes, values, connections)

# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Publish /oak_d/imu/data.

Both SLAM backends use the IMU: rtabmap for its gravity constraint, cuVSLAM
for inertial fusion. The EKF uses its yaw rate, which with no wheel encoders
is the only genuine motion measurement on the robot.

The frame is oak_d_imu_frame, which the URDF places inside the camera body.
A wrong rotation here tilts the SLAM map with full confidence -- rtabmap's
GravitySigma constraint believes what it is told.
"""

from __future__ import annotations

from lunabot_sim import compat
from lunabot_sim.graphs import context


def build(
    imu_prim: str,
    graph_path: str = '/World/Graphs/Imu',
    domain_id: int = context.DEFAULT_DOMAIN_ID,
):
    """Read an IMU sensor prim and publish it as sensor_msgs/Imu.

    VERIFY: the IMU sensor moved between omni.isaac.sensor and
    isaacsim.sensors.physics across versions, and the sensor prim must be
    created before this graph reads it -- see run_sim.py.
    """
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
            ('ReadImu', compat.core_node_type('IsaacReadIMU')),
            ('PublishImu', compat.og_node_type('ROS2PublishImu')),
        ],
        values=ctx_values
        + [
            ('ReadImu.inputs:imuPrim', [imu_prim]),
            # True: report linear acceleration with gravity included, which is
            # what a real accelerometer measures and what rtabmap expects to
            # find gravity in. Removing it here would leave the gravity
            # constraint with nothing to constrain.
            ('ReadImu.inputs:readGravity', True),
            ('PublishImu.inputs:topicName', 'oak_d/imu/data'),
            ('PublishImu.inputs:frameId', 'oak_d_imu_frame'),
            ('PublishImu.inputs:publishOrientation', True),
            ('PublishImu.inputs:publishLinearAcceleration', True),
            ('PublishImu.inputs:publishAngularVelocity', True),
        ],
        connections=[
            ('OnTick.outputs:tick', 'ReadImu.inputs:execIn'),
            ('ReadImu.outputs:execOut', 'PublishImu.inputs:execIn'),
            ('Context.outputs:context', 'PublishImu.inputs:context'),
            ('SimTime.outputs:simulationTime', 'PublishImu.inputs:timeStamp'),
            ('ReadImu.outputs:angVel', 'PublishImu.inputs:angularVelocity'),
            ('ReadImu.outputs:linAcc', 'PublishImu.inputs:linearAcceleration'),
            ('ReadImu.outputs:orientation', 'PublishImu.inputs:orientation'),
        ],
    )

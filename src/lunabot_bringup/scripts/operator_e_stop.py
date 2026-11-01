#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The mission control e-stop button, as a service: /e_stop/set (std_srvs/SetBool).

    ros2 service call /e_stop/set std_srvs/srv/SetBool '{data: true}'

Publishes std_msgs/Bool on /e_stop, which twist_mux's e_stop lock and
mission.py both listen to (docs/MONITORING.md "Mission control").

Why a service and not the laptop publishing /e_stop itself: foxglove_bridge 3.x
reads client_topic_whitelist and never applies it (3.6.3,
FoxgloveBridge::clientAdvertise), so allowing the e-stop's publish allowed
every publish -- /cmd_vel_joy included, which outranks Nav2. The field profile
drops the clientPublish capability entirely, which the bridge does enforce,
and the button calls this instead.

While engaged it republishes true once a second, so a twist_mux or mission.py
that restarts mid-stop locks again. It never republishes false: a restart of
THIS node must not release a stop someone else set.
"""

from __future__ import annotations

TOPIC = '/e_stop'
SERVICE = '/e_stop/set'


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    rclpy.init()
    node = Node('operator_e_stop')
    # Default QoS, as twist_mux's lock and mission.py subscribe.
    pub = node.create_publisher(Bool, TOPIC, 10)
    engaged = {'now': False}
    log = node.get_logger()

    def serve(request, response):
        engaged['now'] = bool(request.data)
        pub.publish(Bool(data=engaged['now']))
        response.success = True
        response.message = 'e-stop ENGAGED' if engaged['now'] else 'e-stop released'
        # Two call sites, not one with a chosen method: rclpy fixes a call
        # site's severity on first use and raises if it changes, which killed
        # this node on the first release (Isaac, 2026-10-31).
        if engaged['now']:
            log.warning(response.message)
        else:
            log.info(response.message)
        return response

    def heartbeat() -> None:
        if engaged['now']:
            pub.publish(Bool(data=True))

    node.create_service(SetBool, SERVICE, serve)
    # Wall clock on purpose (no use_sim_time): the heartbeat must keep going
    # if /clock stops.
    node.create_timer(1.0, heartbeat)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

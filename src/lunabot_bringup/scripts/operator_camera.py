#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A small, slow JPEG of the onboard camera, for mission control over WiFi.

    ros2 run lunabot_bringup operator_camera.py

The raw colour stream is far too big for the competition link (docs/MONITORING.md
"Bandwidth"). This republishes it as sensor_msgs/CompressedImage on
/operator/camera/compressed: 424 px wide, JPEG quality 50, 2 frames a second --
roughly 15 kB a frame, 30 kB/s. Enough to see where the rover is and what it
is facing; not enough to drive by at speed, which is not what it is for.

LAZY: it subscribes to the camera only while something subscribes to it, and
checks once a second. Watching the camera is what makes the driver produce
frames (i_enable_lazy_publisher), so an always-on subscriber here would cost
the robot the full stream even with no laptop connected.
"""

from __future__ import annotations

INPUT = '/oak_d/rgb/image_raw'
OUTPUT = '/operator/camera/compressed'

# Isaac and the OAK-D send these; anything else is passed through as if BGR.
TO_BGR = {'rgb8': 'COLOR_RGB2BGR', 'rgba8': 'COLOR_RGBA2BGR', 'bgra8': 'COLOR_BGRA2BGR'}


def to_bgr(data: bytes, encoding: str, width: int, height: int, step: int):
    """Return a sensor_msgs/Image's pixels as an OpenCV BGR array."""
    import cv2
    import numpy as np

    channels = 4 if encoding in ('rgba8', 'bgra8') else 3
    img = np.frombuffer(data, np.uint8).reshape(height, step)
    img = img[:, : width * channels].reshape(height, width, channels)
    code = TO_BGR.get(encoding)
    return cv2.cvtColor(img, getattr(cv2, code)) if code else img[:, :, :3]


def shrink_and_encode(bgr, width: int, quality: int) -> bytes:
    """Return bgr scaled to `width` (aspect kept, never enlarged), as JPEG bytes."""
    import cv2

    h, w = bgr.shape[:2]
    if w > width:
        bgr = cv2.resize(bgr, (width, round(h * width / w)), interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init()
    node = Node('operator_camera')
    source = node.declare_parameter('input', INPUT).value
    width = node.declare_parameter('width', 424).value
    quality = node.declare_parameter('jpeg_quality', 50).value
    period = 1.0 / node.declare_parameter('rate', 2.0).value
    out = node.create_publisher(CompressedImage, OUTPUT, qos_profile_sensor_data)
    state = {'sub': None, 'last': None}
    log = node.get_logger()

    def on_image(msg: Image) -> None:
        now = node.get_clock().now()
        if state['last'] is not None and (now - state['last']).nanoseconds < period * 1e9:
            return
        state['last'] = now
        try:
            bgr = to_bgr(bytes(msg.data), msg.encoding, msg.width, msg.height, msg.step)
            jpeg = shrink_and_encode(bgr, width, quality)
        except ValueError as error:
            log.warning(f'dropped a frame: {error}', throttle_duration_sec=10.0)
            return
        out.publish(CompressedImage(header=msg.header, format='jpeg', data=jpeg))

    def follow_demand() -> None:
        wanted = out.get_subscription_count() > 0
        if wanted and state['sub'] is None:
            state['sub'] = node.create_subscription(
                Image, source, on_image, qos_profile_sensor_data
            )
            log.info(f'{OUTPUT} has a viewer: watching {source}')
        elif not wanted and state['sub'] is not None:
            node.destroy_subscription(state['sub'])
            state['sub'] = None
            log.info(f'{OUTPUT} has no viewers: stopped watching {source}')

    node.create_timer(1.0, follow_demand)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

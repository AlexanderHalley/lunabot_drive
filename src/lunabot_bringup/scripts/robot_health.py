#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Publish /diagnostics for the running stack.

    ros2 launch lunabot_bringup diagnostics.launch.py profile:=sim

This is docs/TOPIC_FRAME_CONTRACT.md turned into a live signal. check_stack.py
answers "is this graph correct" once, at bring-up, and exits; this answers "is
it still correct" continuously, in the one format every ROS 2 dashboard
already knows how to draw: rqt_robot_monitor, Foxglove's Diagnostics panel and
PlotJuggler all read /diagnostics with no per-robot configuration.

The two files share their premise deliberately -- the contract is the same
contract -- but not their code. check_stack.py is a one-shot with an exit
code, which is what a launch test and a 2am checklist want; this is a node.

===================== WHY IT SUBSCRIBES TO SO LITTLE =====================
A monitor that subscribes to /oak_d/points to check the camera is alive has
doubled the bandwidth of the single heaviest topic on a Raspberry Pi, and
defeated the camera's lazy publisher into the bargain (i_enable_lazy_publisher
in oak_d_s2.yaml: the driver only produces frames when something is
subscribed, so watching it is what makes it expensive).

So heavy topics are watched by PROXY:

  /oak_d/rgb/camera_info    stands in for the RGB stream. Same publisher,
  /oak_d/stereo/camera_info stands in for depth.  same rate, a few dozen
                            bytes instead of a few megabytes.
  /perception/boulders      stands in for /oak_d/points. It is downstream of
                            the cloud, so it going quiet means the cloud
                            stopped OR the detector died -- and both are
                            things you want to be told about.

Nothing here subscribes to an Image or a PointCloud2, and nothing should.
==========================================================================

===================== ON MEASURING STALENESS =====================
Every arrival time recorded here is time.monotonic(), never the ROS clock,
and that is load-bearing under use_sim_time.

If Isaac stops publishing /clock, the ROS clock stops advancing. A staleness
check written against it would compute `now - last_seen` from two frozen
numbers, get a constant, and report a perfectly healthy robot -- at the exact
moment the entire stack has stopped. The wall clock keeps running when the
simulation does not, which is precisely the property this needs.
==================================================================
"""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Imu, JointState
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

from lunabot_msgs.msg import DriveStatus

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR


class Watched:
    """One monitored topic: how fast it should arrive, and how bad it is when it does not.

    `severity` is per-topic rather than a constant because "quiet" does not
    mean the same thing on every topic in this stack. /joint_states going
    quiet means the control stack is dead. /perception/boulders going quiet
    can mean the detector saw an empty cloud and returned early, which is a
    legal thing for it to do -- boulder_detector_node.cpp skips the publish on
    an empty cloud, so a WARN there is honest and an ERROR would be crying
    wolf. A dashboard nobody trusts is worse than no dashboard.
    """

    def __init__(self, topic, msg_type, min_hz, severity=ERROR, qos=10):
        self.topic = topic
        self.msg_type = msg_type
        self.min_hz = min_hz
        self.severity = severity
        self.qos = qos

        self.count = 0
        self.last_seen = None
        self._window_start = time.monotonic()
        self._window_count = 0
        self.rate = 0.0

    def tick(self):
        self.count += 1
        self._window_count += 1
        self.last_seen = time.monotonic()

    def sample_rate(self):
        """Collapse the window into a rate. Called once per diagnostics period."""
        now = time.monotonic()
        elapsed = now - self._window_start
        if elapsed > 0.0:
            self.rate = self._window_count / elapsed
        self._window_start = now
        self._window_count = 0
        return self.rate

    def age(self):
        return None if self.last_seen is None else time.monotonic() - self.last_seen


# The contract, as a monitoring table. Topics and types come from
# docs/TOPIC_FRAME_CONTRACT.md; the rates come from the configs that set them,
# with margin:
#
#   /joint_states   joint_state_broadcaster runs at update_rate, 100 Hz
#   /odom           diff_drive_controller publish_rate, 50 Hz
#   /tf             same publisher, plus robot_state_publisher
#   /oak_d/imu/data i_acc_freq / i_gyro_freq, 400 Hz
#   camera_info     rgb i_fps 15, stereo i_fps 10
#   /drive/status   status_publish_rate in lunabot.ros2_control.xacro, 20 Hz
#
# Every threshold is a fraction of the configured rate, not the rate itself.
# A monitor that alarms at 99% of nominal is a monitor that gets muted.
def core_topics():
    return [
        Watched('/joint_states', JointState, min_hz=20.0),
        Watched('/odom', Odometry, min_hz=10.0),
        Watched('/tf', TFMessage, min_hz=10.0),
    ]


def sim_topics():
    return [Watched('/clock', Clock, min_hz=10.0)]


def real_topics():
    return [Watched('/drive/status', DriveStatus, min_hz=5.0)]


def camera_topics():
    return [
        Watched('/oak_d/rgb/camera_info', CameraInfo, min_hz=3.0),
        Watched('/oak_d/stereo/camera_info', CameraInfo, min_hz=2.0),
        Watched('/oak_d/imu/data', Imu, min_hz=50.0),
    ]


def perception_topics():
    return [Watched('/perception/boulders', Detection3DArray, min_hz=1.0, severity=WARN)]


# joint_state_broadcaster is what /joint_states and therefore the whole TF
# tree hangs off; diff_drive_controller is what turns /cmd_vel into motion and
# publishes /odom. Anything else loaded is reported but not judged.
REQUIRED_CONTROLLERS = ('joint_state_broadcaster', 'diff_drive_controller')


class RobotHealth(Node):
    def __init__(self):
        super().__init__('robot_health')

        self.declare_parameter('profile', 'mock')
        self.declare_parameter('camera', False)
        self.declare_parameter('perception', False)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('hardware_id', 'lunabot')

        profile = self.get_parameter('profile').value
        self._profile = profile
        self._hardware_id = self.get_parameter('hardware_id').value

        self._watched = core_topics()
        if profile == 'sim':
            self._watched += sim_topics()
        elif profile == 'real':
            self._watched += real_topics()
        if self.get_parameter('camera').value:
            self._watched += camera_topics()
        if self.get_parameter('perception').value:
            self._watched += perception_topics()

        for watched in self._watched:
            # Default argument, not a closure over the loop variable: a bare
            # `lambda _: watched.tick()` would capture the NAME and every
            # subscription would tick whichever topic the loop finished on.
            self.create_subscription(
                watched.msg_type,
                watched.topic,
                lambda _msg, watched=watched: watched.tick(),
                watched.qos,
            )

        # The drivetrain task reads the same DriveStatus the rate monitor
        # counts, so keep the last one rather than subscribing twice.
        self._drive_status = None
        self._drive_status_seen = None
        if profile == 'real':
            self.create_subscription(DriveStatus, '/drive/status', self._on_drive_status, 10)

        self._controllers = None
        self._controllers_error = 'not queried yet'
        self._controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )

        self._publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        period = self.get_parameter('publish_period').value
        self.create_timer(period, self._publish)
        # Out of step with the diagnostics period on purpose. Controllers do
        # not change state second to second, and a service round trip per
        # second against controller_manager is load on the one node whose
        # timing actually matters.
        self.create_timer(5.0, self._request_controllers)

        self.get_logger().info(
            f'publishing /diagnostics for profile {profile!r}, watching '
            f'{len(self._watched)} topics'
        )

    # -- collection ---------------------------------------------------

    def _on_drive_status(self, msg):
        self._drive_status = msg
        self._drive_status_seen = time.monotonic()

    def _request_controllers(self):
        if not self._controllers_client.service_is_ready():
            self._controllers = None
            self._controllers_error = 'controller_manager is not advertising list_controllers'
            return

        future = self._controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_controllers)

    def _on_controllers(self, future):
        try:
            self._controllers = future.result().controller
            self._controllers_error = ''
        except Exception as error:
            # Blind, on purpose. A monitor that dies because the thing it
            # monitors misbehaved has removed the only evidence of the
            # misbehaviour. The failure becomes the diagnostic.
            self._controllers = None
            self._controllers_error = f'list_controllers failed: {error}'

    # -- reporting ----------------------------------------------------

    def _status(self, name, level, message):
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = self._hardware_id
        # DiagnosticStatus.level is a `byte` field, and rosidl maps byte to a
        # one-element bytes object in Python, not to an int. Assigning a plain
        # 0 raises AssertionError from the setter. The module-level OK/WARN/
        # ERROR above are the generated constants themselves, so they are
        # whatever type this field wants, in every distro -- which is why
        # nothing here ever writes a literal level.
        status.level = level
        status.message = message
        return status

    def _topic_status(self, watched):
        rate = watched.sample_rate()
        age = watched.age()

        if age is None:
            status = self._status(f'Topics: {watched.topic}', watched.severity, 'never received')
        elif rate < watched.min_hz:
            # Two different failures wearing one number. Say which: "nothing
            # for 12 s" and "running at half rate" call for different actions,
            # and a dashboard that only shows a rate makes you work that out.
            level = watched.severity if age > 2.0 else WARN
            detail = (
                f'stale, nothing for {age:.1f} s'
                if age > 2.0
                else f'{rate:.1f} Hz, below the {watched.min_hz:.1f} Hz minimum'
            )
            status = self._status(f'Topics: {watched.topic}', level, detail)
        else:
            status = self._status(f'Topics: {watched.topic}', OK, f'{rate:.1f} Hz')

        status.values = [
            KeyValue(key='rate_hz', value=f'{rate:.2f}'),
            KeyValue(key='minimum_hz', value=f'{watched.min_hz:.2f}'),
            KeyValue(key='messages', value=str(watched.count)),
            KeyValue(key='age_s', value='never' if age is None else f'{age:.2f}'),
            KeyValue(key='publishers', value=str(self.count_publishers(watched.topic))),
        ]
        return status

    def _controllers_status(self):
        if self._controllers is None:
            return self._status('Controllers', ERROR, self._controllers_error)

        states = {controller.name: controller.state for controller in self._controllers}
        missing = [name for name in REQUIRED_CONTROLLERS if states.get(name) != 'active']

        if missing:
            # The symptom of this is silence, every time: no /joint_states, so
            # no wheel TF; or no /odom, so nothing to navigate against. The
            # spawner reports only "Failed loading controller", a long way
            # from where anyone is looking.
            detail = ', '.join(f'{name} ({states.get(name, "absent")})' for name in missing)
            status = self._status('Controllers', ERROR, f'not active: {detail}')
        else:
            status = self._status('Controllers', OK, f'{len(states)} loaded, required ones active')

        status.values = [KeyValue(key=name, value=state) for name, state in sorted(states.items())]
        return status

    def _drivetrain_status(self):
        if self._drive_status is None:
            return self._status(
                'Drivetrain',
                ERROR,
                'no /drive/status -- is the SparkFlexSystem component active?',
            )

        message = self._drive_status
        faulted = [motor.joint_name for motor in message.motors if motor.fault_bits != 0]

        if faulted:
            status = self._status('Drivetrain', ERROR, 'faults on ' + ', '.join(faulted))
        elif not message.motor_feedback_active:
            # NOT a warning, and this is a judgement about what a dashboard is
            # for. Open-loop is the known, documented, deliberate state of
            # this drivetrain (docs/HARDWARE_CAN.md) and will be for the
            # foreseeable season. A permanent amber light is one nobody reads
            # by the second week, taking the real warnings with it.
            status = self._status(
                'Drivetrain', OK, 'open loop -- odometry is dead reckoning from command'
            )
        else:
            status = self._status('Drivetrain', OK, 'closed loop on motor feedback')

        status.values = [
            KeyValue(key='can_interface', value=message.can_interface),
            KeyValue(key='motor_feedback_active', value=str(message.motor_feedback_active)),
            # Reported, never escalated. watchdog_triggered is true whenever
            # the rover is parked, which is most of its life; escalating it
            # would make Drivetrain amber on a perfectly healthy stationary
            # robot. See DriveStatus.msg for what the field can honestly mean.
            KeyValue(key='watchdog_triggered', value=str(message.watchdog_triggered)),
            KeyValue(
                key='time_since_last_command_s',
                value=f'{message.time_since_last_command:.2f}',
            ),
        ]
        for motor in message.motors:
            status.values.append(
                KeyValue(
                    key=f'{motor.joint_name}/applied_duty_cycle',
                    value=f'{motor.applied_duty_cycle:.3f}',
                )
            )
        return status

    def _publish(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self._topic_status(watched) for watched in self._watched]
        array.status.append(self._controllers_status())
        if self._profile == 'real':
            array.status.append(self._drivetrain_status())
        self._publisher.publish(array)


def main(argv=None):
    # argparse only so --help works and so a stray argument is rejected here
    # rather than by rclpy with a less useful message. Everything is a ROS
    # parameter; diagnostics.launch.py sets them.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=argv)
    node = RobotHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

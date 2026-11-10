#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavation server as a process, through its real interfaces.

Starts scripts/excavator.py (model backend, fast strokes) and drives it as
mission.py does: goals, a cancel, the e-stop, the stow service, the status.
Needs the workspace built (lunabot_msgs), so it runs under colcon.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from lunabot_msgs.action import Deposit, Excavate  # noqa: E402
from lunabot_msgs.msg import ExcavatorStatus  # noqa: E402

SCRIPT = Path(__file__).resolve().parent.parent / 'scripts' / 'excavator.py'


@pytest.fixture(scope='module')
def ros():
    # A domain of our own, so a running stack on this machine is not touched.
    env = dict(os.environ, ROS_DOMAIN_ID=str(90 + os.getpid() % 10))
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    server = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            '--ros-args',
            '-p',
            'model.arm_seconds:=0.5',
            '-p',
            'model.hopper_seconds:=0.5',
            '-p',
            'model.fill_measured:=true',
        ],
        env=env,
    )
    rclpy.init()
    node = rclpy.create_node('test_excavator_client')
    seen = {'status': [], 'joints': []}
    node.create_subscription(ExcavatorStatus, '/excavation/status', seen['status'].append, 10)
    node.create_subscription(JointState, '/excavation/joint_states', seen['joints'].append, 10)
    e_stop = node.create_publisher(Bool, '/e_stop', 10)
    dig = ActionClient(node, Excavate, '/excavation/excavate')
    dump = ActionClient(node, Deposit, '/excavation/deposit')
    stow = node.create_client(Trigger, '/excavation/stow')
    assert dig.wait_for_server(timeout_sec=20.0) and dump.wait_for_server(timeout_sec=5.0)
    assert stow.wait_for_service(timeout_sec=5.0)
    yield node, dig, dump, stow, e_stop, seen
    node.destroy_node()
    rclpy.shutdown()
    server.terminate()
    server.wait(timeout=10)


def spin_until(node, done, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


def send(node, client, goal):
    future = client.send_goal_async(goal)
    assert spin_until(node, future.done)
    return future.result()


def result_of(node, handle, timeout=20.0):
    future = handle.get_result_async()
    assert spin_until(node, future.done, timeout)
    return future.result().result


def latest_state(node, seen):
    seen['status'].clear()
    assert spin_until(node, lambda: seen['status'])
    return seen['status'][-1]


def test_dig_then_dump_end_stowed_with_the_load_moved(ros):
    node, dig, dump, _stow, _e_stop, seen = ros
    handle = send(node, dig, Excavate.Goal(target_fill=0.5, max_seconds=30.0))
    assert handle.accepted
    result = result_of(node, handle)
    assert result.outcome == Excavate.Result.FULL and result.fill >= 0.5
    assert latest_state(node, seen).state == ExcavatorStatus.STOWED
    result = result_of(node, send(node, dump, Deposit.Goal(max_seconds=10.0)))
    assert result.outcome == Deposit.Result.EMPTY
    status = latest_state(node, seen)
    assert status.state == ExcavatorStatus.STOWED and status.fill < 0.05
    assert status.backend == 'model' and status.fill_measured
    assert seen['joints'] and list(seen['joints'][-1].name) == [
        'excavator_arm',
        'excavator_drum',
        'excavator_hopper',
    ]


def test_cancel_holds_and_stow_brings_it_home(ros):
    node, dig, _dump, stow, _e_stop, seen = ros
    handle = send(node, dig, Excavate.Goal(target_fill=1.0, max_seconds=30.0))
    spin_until(node, lambda: False, timeout=1.5)  # lowered (0.5 s), digging
    cancel = handle.cancel_goal_async()
    assert spin_until(node, cancel.done)
    assert result_of(node, handle).outcome == Excavate.Result.CANCELED
    status = latest_state(node, seen)
    assert status.state == ExcavatorStatus.HOLDING and status.arm_position > 0.9
    future = stow.call_async(Trigger.Request())
    assert spin_until(node, future.done) and future.result().success
    assert spin_until(
        node, lambda: seen['status'] and seen['status'][-1].state == ExcavatorStatus.STOWED
    )


def test_the_e_stop_ends_the_goal_and_refuses_the_next(ros):
    node, dig, _dump, _stow, e_stop, seen = ros
    handle = send(node, dig, Excavate.Goal(target_fill=1.0, max_seconds=30.0))
    spin_until(node, lambda: False, timeout=0.3)
    e_stop.publish(Bool(data=True))
    assert result_of(node, handle).outcome == Excavate.Result.E_STOP
    assert latest_state(node, seen).e_stop
    assert not send(node, dig, Excavate.Goal(target_fill=1.0, max_seconds=1.0)).accepted
    e_stop.publish(Bool(data=False))
    assert spin_until(node, lambda: seen['status'] and not seen['status'][-1].e_stop)
    handle = send(node, dig, Excavate.Goal(target_fill=1.0, max_seconds=0.5))
    assert handle.accepted
    assert result_of(node, handle).outcome in (Excavate.Result.FULL, Excavate.Result.TIME_LIMIT)

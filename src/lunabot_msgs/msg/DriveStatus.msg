# Aggregate state of the drivetrain, published on /drive/status.
#
# One message per cycle covering all four motors, rather than four separate
# messages, so a snapshot is atomic and plots line up in time.

std_msgs/Header header

# One entry per wheel, in the order the hardware interface declares its joints:
# front_left, front_right, rear_left, rear_right.
MotorStatus[] motors

# CAN interface in use, e.g. "can0" or "vcan0".
string can_interface

# True when read() returns measured values from the bus. False means the
# reported velocities are the commanded values echoed back and the resulting
# odometry is dead reckoning from command -- see docs/HARDWARE_CAN.md.
bool motor_feedback_active

# Seconds since the drivetrain was last asked to move: the last control cycle
# in which a commanded velocity was non-zero, or changed.
#
# NOT "seconds since a command arrived", which cannot be measured. A
# ros2_control command interface is a bare double in shared memory with no
# timestamp and no writer identity, so a controller writing the same value
# every cycle and a controller that has died look identical to the hardware
# component. This is the strongest statement available, and its useful
# reading is the one a dashboard wants: a rising value means nothing is
# driving this robot.
#
# One false positive follows from that: an operator holding a deliberate,
# sustained zero is indistinguishable from a dead controller.
float32 time_since_last_command

# True when time_since_last_command has exceeded command_timeout (0.5 s by
# default, matching diff_drive_controller's cmd_vel_timeout).
#
# The hardware layer has no watchdog of its own -- the controller's replaced
# it, see docs/HARDWARE_CAN.md -- so this is that watchdog observed from
# below: the outputs are zero and have been for longer than the controller
# would have tolerated silence.
bool watchdog_triggered

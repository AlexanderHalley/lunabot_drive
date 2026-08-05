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

# Seconds since the last command reached the hardware interface. Compare
# against diff_drive_controller's cmd_vel_timeout (0.5 s) to see how close
# the drivetrain is to a watchdog stop.
float32 time_since_last_command

# True when the watchdog has zeroed the outputs.
bool watchdog_triggered

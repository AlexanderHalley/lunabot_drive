# Telemetry for a single SparkFlex motor controller.
#
# Fields a controller cannot report are NaN (floats) or 0 (fault_bits), not a
# sentinel like -1. Consumers must check. In particular: with
# use_motor_feedback:=false the SparkFlexSystem does not read the bus at all,
# so every measured field here is NaN.
#
# AS SHIPPED, only applied_duty_cycle carries a number. velocity and position
# follow use_motor_feedback; bus_voltage, output_current, temperature and
# fault_bits stay NaN/0 unconditionally, because SparkFlexMotor exposes no
# getter for any of them yet. A plot of bus voltage is therefore an empty
# plot rather than a convincing flat line at 0 V, which is the point of the
# convention. See docs/HARDWARE_CAN.md.

# The ros2_control joint this motor drives, e.g. "front_left_wheel_joint".
string joint_name

# SocketCAN device ID of this SparkFlex.
uint8 can_id

# Duty cycle last commanded, [-1.0, 1.0]. Post-clamp, post-inversion --
# this is what actually went on the wire.
float32 applied_duty_cycle

# Measured motor-shaft velocity, rad/s, before the gear ratio.
# NaN when feedback is unavailable.
float32 velocity

# Accumulated motor-shaft position, rad, before the gear ratio.
# NaN when feedback is unavailable.
float32 position

# Bus voltage at the controller, volts. NaN when unavailable.
float32 bus_voltage

# Motor output current, amps. NaN when unavailable.
float32 output_current

# Controller temperature, degrees Celsius. NaN when unavailable.
float32 temperature

# Vendor fault bitfield, verbatim. 0 means no faults reported OR no telemetry;
# check the corresponding measured fields for NaN to tell the two apart.
uint16 fault_bits

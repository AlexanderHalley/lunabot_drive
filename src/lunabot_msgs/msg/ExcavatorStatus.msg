# The excavator as the excavation server sees it, on /excavation/status at 10 Hz.
#
# Positions are NORMALISED, not angles: the 2027 drum, four-bar and hopper
# geometry is not final, so 0 is stowed and 1 is the working end of the
# stroke, and the backend maps that onto its actuators. Unknown values are
# NaN, as in MotorStatus: consumers must check.

std_msgs/Header header

uint8 STOWED = 0      # everything home: safe to drive
uint8 HOLDING = 1     # stopped where it was (a cancelled goal, or the e-stop): NOT safe to drive
uint8 EXCAVATING = 2
uint8 DEPOSITING = 3
uint8 STOWING = 4
uint8 FAULT = 5       # an actuator did not get where it was sent; holding. See `fault`.
uint8 state

# The step inside the state, for people: LOWERING, DIGGING, RAISING,
# TIPPING, POURING, RETURNING, STOWING, or empty.
string phase

# Drum arm (lift): 0 stowed over the chassis .. 1 drum at digging depth.
float32 arm_position
# Drum speed, rad/s, positive digs.
float32 drum_velocity
# Hopper: 0 flat .. 1 fully tipped.
float32 hopper_position

# Hopper fill, 0 empty .. 1 full.
float32 fill
# False: `fill` is the backend's ESTIMATE (the model backend integrates a dig
# rate), not a measurement. Goals then end on their time limit, not on fill.
bool fill_measured

# True while /e_stop is engaged: everything held, goals refused.
bool e_stop

# Which backend drives it: "model" (kinematic, no hardware) or a driver.
string backend

# Why it is in FAULT; empty otherwise.
string fault

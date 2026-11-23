# Status of the robot hand hardware interface.

std_msgs/Header header

# true if a microcontroller (or the simulated one) is connected and answering
bool connected

# true if the servos are attached (powered). false = limp
bool servos_enabled

# Firmware identification, e.g. "ROBOTHAND MEGA2560 1.0.0"
string firmware

# Finger names in actuator order (thumb, index, middle, ring, little)
string[] fingers

# Current servo angles reported by the firmware (ramped positions) [deg]
float64[] servo_angles_deg

# Current target servo angles sent to the firmware [deg]
float64[] servo_targets_deg

# Finger closure, 0.0 = fully open, 1.0 = fully closed
float64[] closure

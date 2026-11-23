# Strain gauge measurements of the robot hand.
# One entry per gauge, all arrays have the same length.

std_msgs/Header header

# Gauge names, e.g. "thumb", "index", ...
string[] names

# Tared raw value from the microcontroller (ADC counts / HX711 counts)
float64[] raw

# Calibrated value: (raw * scale) + offset, see robothand_interface parameters
float64[] values

# Unit of "values", e.g. "N", "microstrain" or "counts" if uncalibrated
string unit

from constants_4_drones import *

# Alternate option where we allow no extraneous torque on individual drones.
# This is just in case the primary option (which allows some extraneous torque in favor of yaw control) still has issues with drones oscillating.
name = "aviata_4_alt"
drone_roll_torque_limit = 0 # Nm

import numpy as np

g = 9.81 # m/s^2

# AVIATA CONSTANTS
Ct = 6.295 # Newtons of force generated by a single rotor at max power
Cm = Ct * 0.05 # Newton-meters of torque generated by a single rotor at max power #TODO not known, but not very important
num_rotors = 6 # Number of rotors on a single drone
r = 0.550 / 2 # Distance of each rotor from the center the drone (meters)
R = 1.25 # Distance between the center of a drone and the center of the structure (meters) # TODO
num_drones = 8 # Maximum number of drones on the structure

M_drone = 1.769 # mass of a single drone in kg TODO: drones will probably be a little heavier
M_structure = 4.1771 # mass of the structure in kg
M_payload = 2.26796 # mass of the payload in kg (5lb payload)
M_structure_payload = M_structure + M_payload

I_drone = np.array([[0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],  # TODO moment of inertia tensor of a single drone in kg*m^2
                    [0.0, 0.0, 0.0]])

I_structure = np.array([[1.3958, 0.0   , 0.0   ],
                        [0.0   , 1.3958, 0.0   ],  # moment of inertia tensor of the structure in kg*m^2
                        [0.0   , 0.0   , 2.7902]])

I_payload = np.array([[0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],  # TODO moment of inertia tensor of the payload in kg*m^2
                      [0.0, 0.0, 0.0]])

I_structure_payload = I_structure + I_payload

# TODO estimate these values (these are just guesses):
payload_structure_height = 0.1 # vertical distance between the structure center of mass and payload center of mass (meters)
structure_drone_height = 0.075 # vertical distance between a drone's center of mass and the structure center of mass (meters)
structpayload_drone_height = (M_structure * structure_drone_height + M_payload * (payload_structure_height + structure_drone_height)) / M_structure_payload  # vertical distance between a drone's center of mass and the structure+payload center of mass (meters)
drone_prop_height = 0.05 # vertical distance between a drone's propellers and its center of mass (meters)

# Control Constants
P_pos = np.array([0.95, 0.95, 1.0]) / 2

P_vel = np.array([1.8, 1.8, 4.0]) / 2
I_vel = np.array([0.4, 0.4, 2.0]) / 20
D_vel = np.array([0.2, 0.2, 0.0]) * 5

P_att = np.array([6.5, 6.5, 2.8]) / 2

P_att_rate = np.array([0.15, 0.15, 0.2])
I_att_rate = np.array([0.2, 0.2, 0.1]) / 20
D_att_rate = np.array([0.003, 0.003, 0.0]) * 5

max_vel_hor = 12.0 # m/s
max_vel_down = 1.0
max_vel_up = 3.0

max_acc_hor = 5.0 # m/s^2
max_acc_down = 3.0
max_acc_up = 4.0

max_att_rate = np.array([3.84, 3.84, 3.49]) # radians/s

# PX4 Defaults:
# P_pos = np.array([0.95, 0.95, 1.0])

# P_vel = np.array([1.8, 1.8, 4.0])
# I_vel = np.array([0.4, 0.4, 2.0])
# D_vel = np.array([0.2, 0.2, 0.0])

# P_att = np.array([6.5, 6.5, 2.8])

# P_att_rate = np.array([0.15, 0.15, 0.2])
# I_att_rate = np.array([0.2, 0.2, 0.1])
# D_att_rate = np.array([0.003, 0.003, 0.0])

# max_vel_hor = 12.0 # m/s
# max_vel_down = 1.0
# max_vel_up = 3.0

# max_acc_hor = 5.0 # m/s^2
# max_acc_down = 3.0
# max_acc_up = 4.0

# max_att_rate = np.array([3.84, 3.84, 3.49]) # radians/s

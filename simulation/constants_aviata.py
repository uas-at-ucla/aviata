import numpy as np

g = 9.81 # m/s^2

# AVIATA CONSTANTS
Ct = 0.960 * g # (for 4S batteries, 20% margin under 1.2kg spec) Newtons of force generated by a single rotor at max power (6.295 N for 3S batteries)
Cm = Ct * 0.05 # Newton-meters of torque generated by a single rotor at max power #TODO not known, but not very important
num_rotors = 6 # Number of rotors on a single drone
rotor_angle = 10 # Canted motor angle (degrees)
r = 0.550 / 2 # Distance of each rotor from the center the drone (meters)
R = 1.25 # Distance between the center of a drone and the center of the structure (meters) # TODO
num_drones = 8 # Maximum number of drones on the structure
drones_face_inward = False # True if the front of each drone faces inward when docked, False if they face outward

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
# TODO Do PID tuning like in constants_two_drones.py
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

max_att_rate = np.deg2rad(np.array([220.0, 220.0, 200.0])) # radians/s

# PX4 Defaults:
# P_pos = np.array([0.95, 0.95, 1.0]) # MPC_XY_P, MPC_Z_P

# P_vel = np.array([1.8, 1.8, 4.0]) # MPC_XY_VEL_P_ACC, MPC_Z_VEL_P_ACC
# I_vel = np.array([0.4, 0.4, 2.0]) # MPC_XY_VEL_I_ACC, MPC_Z_VEL_I_ACC
# D_vel = np.array([0.2, 0.2, 0.0]) # MPC_XY_VEL_D_ACC, MPC_Z_VEL_D_ACC

# P_att = np.array([6.5, 6.5, 2.8]) # MC_ROLL_P, MC_PITCH_P, MC_YAW_P

# P_att_rate = np.array([0.15, 0.15, 0.2]) # MC_ROLLRATE_P, MC_PITCHRATE_P, MC_YAWRATE_P
# I_att_rate = np.array([0.2, 0.2, 0.1]) # MC_ROLLRATE_I, MC_PITCHRATE_I, MC_YAWRATE_I
# D_att_rate = np.array([0.003, 0.003, 0.0]) # MC_ROLLRATE_D, MC_PITCHRATE_D, MC_YAWRATE_D
# # To scale all at once: MC_ROLLRATE_K, MC_PITCHRATE_K, MC_YAWRATE_K
# # Also relevant: MC_ROLLRATE_FF, MC_PITCHRATE_FF, MC_YAWRATE_FF; MC_RR_INT_LIM, MC_PR_INT_LIM, MC_YR_INT_LIM

# max_vel_hor = 12.0 # m/s MPC_XY_VEL_MAX, MPC_XY_CRUISE
# max_vel_down = 1.0 # MPC_Z_VEL_MAX_DN
# max_vel_up = 3.0 # MPC_Z_VEL_MAX_UP
# # Also relevant: MPC_TKO_SPEED

# max_acc_hor = 5.0 # m/s^2 MPC_ACC_HOR_MAX, MPC_ACC_HOR
# max_acc_down = 3.0 # MPC_ACC_DOWN_MAX
# max_acc_up = 4.0 # MPC_ACC_UP_MAX

# max_att_rate = np.deg2rad(np.array([220.0, 220.0, 200.0])) # radians/s MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX, MPC_YAWRAUTO_MAX

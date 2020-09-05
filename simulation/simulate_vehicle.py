#!/usr/bin/env python

import numpy as np
import px4_mixer_multirotor
import generate_matrices
import constants

# position (m)
pos = np.array([0, 0, 0])

# velocity (m/s)
vel = np.array([0, 0, 0])

# attitude (quaternion)
att = np.array([0, 0, 0]) # TODO

# angular velocity (rad/s)
att_rate = np.array([0, 0, 0])

if __name__ == '__main__':
    missing_drones = [] # 0 through 7
    geometry, header = generate_matrices.generate_aviata_matrices(missing_drones)
    mixer = geometry['mix']['B_px_4dof']
    actuator_effectiveness = geometry['mix']['A_4dof']

    setpoint = np.matrix([0, 0, 0, 0.5]).T
    print("normalized forces (used by PX4):")
    print(setpoint)
    print()

    ideal_forces = actuator_effectiveness * (mixer * setpoint)
    print("ideal forces:")
    print(ideal_forces)
    print()

    u, u_final = px4_mixer_multirotor.normal_mode(setpoint, mixer, 0.0, 1.0)

    print("motor inputs (used by PX4):")
    print(u_final)
    print()

    actual_forces = actuator_effectiveness * u_final
    print("actual forces:")
    print(actual_forces)
    print()

#!/usr/bin/env python

import numpy as np
import quaternion
import px4_mixer_multirotor
import generate_matrices
import constants
from pyquaternion import Quaternion

# position (m)
pos = np.array([0.0, 0.0, 0.0])

# velocity (m/s)
vel = np.array([0.0, 0.0, 0.0])

# attitude (quaternion)
att = Quaternion(axis=[0, 0, 1], angle=0)

# angular velocity (rad/s)
att_rate = np.array([0.0, 0.0, 0.0])

ang_acc = np.array([0.0, 0.0, 0.0])
lin_acc = np.array([0.0, 0.0, 0.0])
def simulate_forces(mixer, actuator_effectiveness, setpoint, Ts):
    global pos
    global vel
    global att
    global att_rate
    global ang_acc
    global lin_acc

    if np.linalg.norm(att_rate) != 0:
        att = Quaternion(axis=(att_rate / np.linalg.norm(att_rate)), angle=(np.linalg.norm(att_rate) * Ts)) * att
    pos += vel * Ts

    att_rate += ang_acc * Ts
    vel += lin_acc * Ts

    ideal_forces = actuator_effectiveness * (mixer * setpoint)
    u, u_final = px4_mixer_multirotor.normal_mode(setpoint, mixer, 0.0, 1.0)
    actual_forces = actuator_effectiveness * u_final
    actual_forces = np.array(actual_forces)

    torques = actual_forces[:3,0]
    force = actual_forces[3,0]

    ang_acc = torques / np.array(constants.I)
    lin_acc = force / constants.M

    # transform from body frame to global frame
    ang_acc = att.rotate(ang_acc)
    lin_acc = att.rotate(np.array((0.0, 0.0, lin_acc)))


def mixer_test():
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


if __name__ == '__main__':
    missing_drones = [] # 0 through 7
    geometry, header = generate_matrices.generate_aviata_matrices(missing_drones)
    mixer = geometry['mix']['B_px_4dof']
    actuator_effectiveness = geometry['mix']['A_4dof']

    setpoint = np.matrix([0, 0, 0, 0.5]).T
    for i in range(10):
        simulate_forces(mixer, actuator_effectiveness, setpoint, 0.1)
        print(pos)

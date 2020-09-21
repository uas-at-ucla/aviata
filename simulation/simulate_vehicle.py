#!/usr/bin/env python

import numpy as np
import quaternion
import px4_mixer_multirotor
import generate_matrices
import graphics
import constants
from pyquaternion import Quaternion
import time

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

    att_rate_magnitude = np.linalg.norm(att_rate)
    if att_rate_magnitude != 0:
        att_rate_direction = att_rate / np.linalg.norm(att_rate)
        att = Quaternion(axis=att_rate_direction, angle=(att_rate_magnitude * Ts)) * att
    pos += vel * Ts

    att_rate += ang_acc * Ts
    vel += lin_acc * Ts

    setpoint = np.matrix(setpoint).T
    ideal_forces = np.linalg.multi_dot([actuator_effectiveness, mixer, setpoint])
    u, u_final = px4_mixer_multirotor.normal_mode(setpoint, mixer, 0.0, 1.0)
    actual_forces = np.dot(actuator_effectiveness, u_final)

    torque = actual_forces[:3]
    force = actual_forces[3,0]

    ang_acc = np.dot(constants.Iinv, torque)[:,0]
    lin_acc = force / constants.M

    # transform from body frame to global frame
    ang_acc = att.rotate(ang_acc)
    lin_acc = att.rotate(np.array((0.0, 0.0, lin_acc)))
    lin_acc[2] += 9.81 # gravity is a thing


def mixer_test():
    missing_drones = [] # 0 through 7
    geometry = generate_matrices.generate_aviata_matrices(missing_drones)
    mixer = geometry['mix']['B_px_4dof']
    actuator_effectiveness = geometry['mix']['A_4dof']

    setpoint = np.matrix([0, 0, 0, 0.5]).T
    print("normalized forces (used by PX4):")
    print(setpoint)
    print()

    ideal_forces = np.linalg.multi_dot([actuator_effectiveness, mixer, setpoint])
    print("ideal forces:")
    print(ideal_forces)
    print()

    u, u_final = px4_mixer_multirotor.normal_mode(setpoint, mixer, 0.0, 1.0)

    print("motor inputs (used by PX4):")
    print(u_final)
    print()

    actual_forces = np.dot(actuator_effectiveness, u_final)
    print("actual forces:")
    print(actual_forces)
    print()


def const_forces_test():
    missing_drones = [] # 0 through 7
    geometry = generate_matrices.generate_aviata_matrices(missing_drones)
    mixer = geometry['mix']['B_px_4dof']
    actuator_effectiveness = geometry['mix']['A_4dof']

    sample_period_ms = 50
    sample_period_s = sample_period_ms / 1000
    setpoint = np.array([0.001, 0.0, 0.0, 0.6])

    def loop(GraphicsState):
        nonlocal setpoint
        simulate_forces(mixer, actuator_effectiveness, setpoint, sample_period_s)
        GraphicsState.updatePosition(pos, att)

    graphics.main(loop, sample_period_ms)


if __name__ == '__main__':
    const_forces_test()

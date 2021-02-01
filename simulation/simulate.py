#!/usr/bin/env python

import numpy as np
import px4_mixer_multirotor
import config
import graphics
from drones import *
from OpenGL.GLUT import *


def mixer_test(): # test
    missing_drones = [] # 0 through 7
    geometry, geometries = config.generate_matrices.generate_aviata_matrices(missing_drones)
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


def pos_control_test():
    missing_drones = [] # 0 through 7
    sample_period_ms = 50
    # forces_setpoint = np.matrix([0.0007, 0.0, 0.2, 0.63]).T
    pos_setpoint = np.array([0.0, 0.0, 0.0])
    yaw_setpoint = 0

    world = PhysicalWorld(config.constants.num_drones, sample_period_ms)
    world.set_missing_drones(missing_drones)
    world.leader_drone.set_pos_setpoint(pos_setpoint, yaw_setpoint)

    def keyPressed(key, mouse_x, mouse_y):
        nonlocal pos_setpoint
        nonlocal missing_drones
        fine_inc = 0.1
        coarse_inc = 1
        if key == GLUT_KEY_UP:
            pos_setpoint[0] += fine_inc
        elif key == GLUT_KEY_LEFT:
            pos_setpoint[1] -= fine_inc
        elif key == GLUT_KEY_DOWN:
            pos_setpoint[0] -= fine_inc
        elif key == GLUT_KEY_RIGHT:
            pos_setpoint[1] += fine_inc
        elif key == b'\'':
            pos_setpoint[2] -= fine_inc
        elif key == b'/':
            pos_setpoint[2] += fine_inc

        elif key == b'w':
            pos_setpoint[0] += coarse_inc
        elif key == b'a':
            pos_setpoint[1] -= coarse_inc
        elif key == b's':
            pos_setpoint[0] -= coarse_inc
        elif key == b'd':
            pos_setpoint[1] += coarse_inc
        elif key == b'r':
            pos_setpoint[2] -= coarse_inc
        elif key == b'f':
            pos_setpoint[2] += coarse_inc

        for drone in range(8):
            # Drones 0-7 correlate with keys 1-8
            if key == bytes(str(drone+1), encoding='utf-8'):
                if drone in missing_drones:
                    missing_drones.remove(drone)
                else:
                    missing_drones.append(drone)
                world.set_missing_drones(missing_drones)

    def loop(GraphicsState):
        nonlocal world
        nonlocal pos_setpoint
        nonlocal yaw_setpoint
        world.leader_drone.set_pos_setpoint(pos_setpoint, yaw_setpoint)
        world.tick()
        GraphicsState.pos_target = pos_setpoint
        GraphicsState.pos = world.structure.pos
        GraphicsState.att = world.structure.att
        GraphicsState.rotors = world.structure.geometry['rotors']
        GraphicsState.motor_inputs = world.structure.u

    graphics.main(loop, world.sample_period_ms, keyPressed)


if __name__ == '__main__':
    pos_control_test()

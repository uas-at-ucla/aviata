#!/usr/bin/env python

import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from itertools import combinations
import px4_generate_mixer

constants = None

def configure_constants(constants_selection):
    global constants
    constants = constants_selection

TWO_PI = math.pi * 2

def geometry_name(missing_drones=[]):
    return 'aviata_missing_' + '_'.join(map(str, sorted(missing_drones)))

def geometry_name_desc(missing_drones=[]):
    name = geometry_name(missing_drones)
    description = "AVIATA with these drones missing: " + ", ".join(map(str, sorted(missing_drones)))
    return name, description

# def mixer_name(drone_pos, missing_drones=[]):
#     return 'aviata_pos_' + str(drone_pos) + '_missing_' + '_'.join(map(str, sorted(missing_drones)))

# def mixer_name_desc(drone_pos, missing_drones=[]):
#     name = mixer_name(drone_pos, missing_drones)
#     description = "AVIATA drone position " + str(drone_pos) + " with these drones missing: " + ", ".join(map(str, sorted(missing_drones)))
#     return name, description


def parallel_axis_theorem(I, m, R):
    return I + m * (np.dot(R,R)*np.eye(3) - np.outer(R,R))


def generate_aviata_matrices(missing_drones=[]):
    drone_rotors_sequential = []
    CW = False # as seen from above
    angle = (TWO_PI/constants.num_rotors) / 2
    for i in range(constants.num_rotors):
        rotor_pos = constants.r * np.exp(1j * angle)
        rotor_axis = np.exp(1j * np.deg2rad(constants.rotor_angle))
        rotor_axis_z = -np.real(rotor_axis)
        rotor_axis_hor = np.imag(rotor_axis) * np.exp(1j * (angle + (-math.pi/2 if CW else math.pi/2)))
        rotor_axis_x = np.real(rotor_axis_hor)
        rotor_axis_y = np.imag(rotor_axis_hor)

        drone_rotors_sequential.append({
            'position': np.array([np.real(rotor_pos), np.imag(rotor_pos), 0]),
            'direction': 'CW' if CW else 'CCW',
            'axis': np.array([rotor_axis_x, rotor_axis_y, rotor_axis_z]),
            'Ct': constants.Ct,
            'Cm': constants.Cm
        })
        CW = (not CW)
        angle += (TWO_PI/constants.num_rotors)

    # Reorder rotors to conform to PX4 convention: https://docs.px4.io/v1.11/en/airframes/airframe_reference.html#hexarotor-x 
    drone_rotors = [None] * constants.num_rotors
    drone_rotors[4] = drone_rotors_sequential[0]
    drone_rotors[0] = drone_rotors_sequential[1]
    drone_rotors[3] = drone_rotors_sequential[2]
    drone_rotors[5] = drone_rotors_sequential[3]
    drone_rotors[1] = drone_rotors_sequential[4]
    drone_rotors[2] = drone_rotors_sequential[5]

    M = constants.M_drone
    COM = np.array([0.0, 0.0, constants.drone_prop_height])

    # make the center of mass the reference point
    COM /= M
    for rotor in drone_rotors:
        rotor['position'] -= COM

    # moment of inertia
    I = constants.I_drone

    # Relation between angular/linear acceleration and torque/force (with the exception of angular velocity's effect on angular acceleration when I is not diagonal)
    # Multiply mass by twice gravitational acceleration so that an input of 0.5 is 1g (hover)
    # The input for roll/pitch/yaw is rad/s^2
    inertia = np.row_stack((np.column_stack((I, np.zeros((3,3)))), np.column_stack((np.zeros((3,3)), np.diag((M*constants.g*2, M*constants.g*2, M*constants.g*2)))))) 

    geometry = {'rotors': drone_rotors}
    A, B = px4_generate_mixer.geometry_to_mix(geometry)
    B[abs(B) < 0.000001] = 0.0
    # A_4dof = np.delete(A, [3,4], 0)
    B_px = px4_generate_mixer.normalize_mix_px4(B, inertia)
    B_px_4dof = np.delete(B_px, [3,4], 1)
    B_px_4dof[:,3] *= -1

    geometry['mix'] = {'A': A, 'B': B, 'B_px': B_px, 'B_px_4dof': np.matrix(B_px_4dof)}
    name, description = geometry_name_desc(missing_drones)
    geometry['info'] = {}
    geometry['info']['key'] = name
    geometry['info']['name'] = name
    geometry['info']['description'] = description

    geometry['M'] = M
    geometry['thr_hover'] = 0.5
    geometry['I'] = I
    geometry['Iinv'] = np.linalg.inv(I)

    if len(missing_drones) == 0:
        geometry['drone_angles'] = [0]

    return geometry


def generate_aviata_permutations(max_missing_drones=0):
    missing_drones_permutations = [[]]
    for i in range(1, max_missing_drones+1):
        missing_drones_permutations += combinations(range(constants.num_drones), i)

    combined_geometries = {}
    combined_geometries_list = []
    for missing_drones in missing_drones_permutations:
        geometry = generate_aviata_matrices(missing_drones)
        combined_geometries[geometry['info']['key']] = geometry
        combined_geometries_list.append(geometry)
    return combined_geometries, combined_geometries_list, max_missing_drones


def main():
    import constants_standalone_drone
    configure_constants(constants_standalone_drone)

    combined_geometries, combined_geometries_list, max_missing_drones = generate_aviata_permutations()

    header = px4_generate_mixer.generate_mixer_multirotor_header(combined_geometries_list,
                                                                 use_normalized_mix=True,
                                                                 use_6dof=False,
                                                                 constants=constants,
                                                                 max_missing_drones=max_missing_drones)
    print(header)

    key = geometry_name(missing_drones=[])

    print("Matrices for " + key, file=sys.stderr)
    print("actuator effectiveness:", file=sys.stderr)
    print(combined_geometries[key]['mix']['A'], file=sys.stderr)
    print("", file=sys.stderr)
    print("mixer:", file=sys.stderr)
    print(combined_geometries[key]['mix']['B_px_4dof'], file=sys.stderr)
    print("mass:", file=sys.stderr)
    print(combined_geometries[key]['M'], file=sys.stderr)
    print("hover thrust:", file=sys.stderr)
    print(combined_geometries[key]['thr_hover'], file=sys.stderr)
    print("maximum thrust value:", file=sys.stderr)
    print(1/np.max(combined_geometries[key]['mix']['B_px_4dof'][:,3]), file=sys.stderr)
    print("maximum vertical thrust (g's):", file=sys.stderr)
    print(2 * 1/np.max(combined_geometries[key]['mix']['B_px_4dof'][:,3]), file=sys.stderr)
    print("max hover thrust percentage:", file=sys.stderr)
    print(np.max(combined_geometries[key]['mix']['B_px_4dof'][:,3]) * combined_geometries[key]['thr_hover'], file=sys.stderr)
    print("average hover thrust percentage:", file=sys.stderr)
    print(np.sum(combined_geometries[key]['mix']['B_px_4dof'][:,3])/np.count_nonzero(combined_geometries[key]['mix']['A'][5]) * combined_geometries[key]['thr_hover'], file=sys.stderr)
    print("max yaw torque:", file=sys.stderr)
    print(1/np.max(combined_geometries[key]['mix']['B'][:,2]), file=sys.stderr)
    print("moment of inertia tensor:", file=sys.stderr)
    print(combined_geometries[key]['I'], file=sys.stderr)
    print("inverse of moment of inertia tensor:", file=sys.stderr)
    print(combined_geometries[key]['Iinv'], file=sys.stderr)

    x = [0] # (0, 0) is center of mass
    y = [0]
    for rotor in combined_geometries[key]['rotors']:
        if rotor['Ct']:
            x.append(rotor['position'][1]) # swap x and y axes so it appears as NED from overhead (North is up)
            y.append(rotor['position'][0])
    plt.scatter(x, y)
    plt.show()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from itertools import combinations
import constants
import px4_generate_mixer

TWO_PI = math.pi * 2

def geometry_name(missing_drones=[]):
    return 'aviata_missing_' + '_'.join(map(str, missing_drones))

def mixer_name(drone_pos, missing_drones=[]):
    return 'aviata_pos_' + str(drone_pos) + '_missing_' + '_'.join(map(str, missing_drones))

def mixer_name_desc(drone_pos, missing_drones=[]):
    name = mixer_name(drone_pos, missing_drones)
    description = "AVIATA drone position " + str(drone_pos) + " with these drones missing: " + ", ".join(map(str, missing_drones))
    return name, description


def generate_aviata_matrices(missing_drones=[]):
    drone_rotors = []
    CW = False
    angle = (TWO_PI/constants.num_rotors) / 2
    for i in range(constants.num_rotors):
        rotor_pos = constants.r * np.exp(1j * angle)
        drone_rotors.append({
            'position': np.array([np.real(rotor_pos) + constants.R, np.imag(rotor_pos), 0]),
            'direction': 'CW' if CW else 'CCW',
            'axis': np.array([0.0, 0.0, -1.0]),
            'Ct': constants.Ct,
            'Cm': constants.Cm
        })
        CW = (not CW)
        angle += (TWO_PI/constants.num_rotors)

    structure_rotors = []
    angle = (TWO_PI/constants.num_drones) / 2
    for i in range(constants.num_drones):
        for j in range(constants.num_rotors):
            rotor = drone_rotors[j].copy()
            if i in missing_drones:
                rotor['Ct'] = 0
                rotor['Cm'] = 0
            rotation = Rotation.from_rotvec(np.array([0, 0, angle]))
            rotor['position'] = rotation.apply(drone_rotors[j]['position'])
            structure_rotors.append(rotor)
        angle += (TWO_PI/constants.num_drones)

    geometry = {'rotors': structure_rotors}
    A, B = px4_generate_mixer.geometry_to_mix(geometry)
    A_4dof = np.delete(A, [3,4], 0)
    B_px = B
    # B_px = np.sign(B_px) # Special AVIATA step: minimize motor saturation TODO np.sign only works when all drones are present
    B_px = px4_generate_mixer.normalize_mix_px4(B_px)
    B_px_4dof = np.delete(B_px, [3,4], 1)
    B_px_4dof[:,3] *= -1
    
    geometry['mix'] = {'A': A, 'A_4dof': A_4dof, 'B': B, 'B_px': B_px, 'B_px_4dof': np.matrix(B_px_4dof)}
    geometry['info'] = {}
    geometry['info']['key'] = geometry_name(missing_drones)

    # break up into a mixer for each individual drone
    geometries = []
    for i in range(constants.num_drones):
        if not i in missing_drones:
            rotor_start = i * constants.num_rotors
            rotor_end = rotor_start + constants.num_rotors
            drone_geometry = {}
            drone_geometry['rotors'] = geometry['rotors'][rotor_start:rotor_end]
            drone_geometry['mix'] = {}
            drone_geometry['mix']['A'] = geometry['mix']['A'][:,rotor_start:rotor_end]
            drone_geometry['mix']['A_4dof'] = geometry['mix']['A_4dof'][:,rotor_start:rotor_end]
            drone_geometry['mix']['B'] = geometry['mix']['B'][rotor_start:rotor_end]
            drone_geometry['mix']['B_px'] = geometry['mix']['B_px'][rotor_start:rotor_end]
            drone_geometry['mix']['B_px_4dof'] = geometry['mix']['B_px_4dof'][rotor_start:rotor_end]

            name, description = mixer_name_desc(i, missing_drones)
            drone_geometry['info'] = {}
            drone_geometry['info']['key'] = name
            drone_geometry['info']['name'] = name
            drone_geometry['info']['description'] = description
            geometries.append(drone_geometry)

    return geometry, geometries


def generate_aviata_permutations(max_missing_drones):
    missing_drones_permutations = [[]]
    for i in range(1, max_missing_drones+1):
        missing_drones_permutations += combinations(range(constants.num_drones), i)

    combined_geometries = {}
    drone_geometries = {}
    drone_geometries_list = []
    for missing_drones in missing_drones_permutations:
        geometry, geometries = generate_aviata_matrices(missing_drones)
        combined_geometries[geometry['info']['key']] = geometry
        for drone_geometry in geometries:
            drone_geometries[drone_geometry['info']['key']] = drone_geometry
        drone_geometries_list += geometries
    return combined_geometries, drone_geometries, drone_geometries_list


if __name__ == '__main__':
    combined_geometries, drone_geometries, drone_geometries_list = generate_aviata_permutations(max_missing_drones=3)

    header = px4_generate_mixer.generate_mixer_multirotor_header(drone_geometries_list,
                                                                 use_normalized_mix=True,
                                                                 use_6dof=False)
    print(header)

    key = geometry_name(missing_drones=[])
    print("Matrices for " + key, file=sys.stderr)
    print("actuator effectiveness:", file=sys.stderr)
    print(combined_geometries[key]['mix']['A_4dof'], file=sys.stderr)
    print("", file=sys.stderr)
    print("mixer:", file=sys.stderr)
    print(combined_geometries[key]['mix']['B_px_4dof'], file=sys.stderr)
    x = []
    y = []
    for rotor in combined_geometries[key]['rotors']:
        if rotor['Ct']:
            x.append(rotor['position'][0])
            y.append(rotor['position'][1])

    plt.scatter(x, y)
    plt.show()

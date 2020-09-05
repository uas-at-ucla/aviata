#!/usr/bin/env python

import math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import constants
import px4_generate_mixer

TWO_PI = math.pi * 2

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
        if not i in missing_drones:
            for j in range(constants.num_rotors):
                rotor = drone_rotors[j].copy()
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
    geometry['info']['key'] = 'aviata_' + '_'.join(map(str, missing_drones))
    geometry['info']['name'] = geometry['info']['key']
    geometry['info']['description'] = "AVIATA with these drones missing: " + ", ".join(map(str, missing_drones))
    
    header = px4_generate_mixer.generate_mixer_multirotor_header([geometry],
                                                                 use_normalized_mix=True,
                                                                 use_6dof=False)
    return geometry, header


if __name__ == '__main__':
    geometry, header = generate_aviata_matrices(missing_drones=[])
    # print(header)
    # print()
    print("actuator effectiveness:")
    print(geometry['mix']['A_4dof'])
    print()
    print("mixer:")
    print(geometry['mix']['B_px_4dof'])
    x = []
    y = []
    for rotor in geometry['rotors']:
        x.append(rotor['position'][0])
        y.append(rotor['position'][1])

    plt.scatter(x, y)
    plt.show()

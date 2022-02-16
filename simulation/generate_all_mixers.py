#!/usr/bin/env python3

import px4_generate_mixer

import constants_two_drones
import constants_4_drones
import constants_4_drones_alt
import generate_matrices_two_drones
import generate_matrices_aviata

configs = [
    (constants_4_drones    , generate_matrices_aviata),
    (constants_4_drones_alt, generate_matrices_aviata),
    (constants_two_drones  , generate_matrices_two_drones),
]

all_geometries_list = []
constants_list = []

for config in configs:
    config[1].configure_constants(config[0])
    combined_geometries, combined_geometries_list = config[1].generate_aviata_permutations()
    all_geometries_list += combined_geometries_list
    constants_list.append(config[0])

header = px4_generate_mixer.generate_mixer_multirotor_header(all_geometries_list,
                                                             use_normalized_mix=True,
                                                             use_6dof=False,
                                                             constants_list=constants_list)

with open("aviata_mixers.h", 'w') as fd:
    fd.write(header)

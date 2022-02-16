import numpy as np

constants = None

def configure_constants(constants_selection):
    global constants
    constants = constants_selection

def geometry_name(missing_drones=[]):
    return constants.name + '_missing_' + '_'.join(map(str, sorted(missing_drones)))

def geometry_name_desc(missing_drones=[]):
    name = geometry_name(missing_drones)
    description = constants.name + " with these drones missing: " + ", ".join(map(str, sorted(missing_drones)))
    return name, description

def generate_aviata_matrices(missing_drones=[]):
    # Copy the _config_hex_x mixer after building PX4 in PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h
    B_px_4dof = np.array([
        ( -1.000000,  0.000000, -1.000000,  1.000000 ),
        (  1.000000,  0.000000,  1.000000,  1.000000 ),
        (  0.500000,  0.866025, -1.000000,  1.000000 ),
        ( -0.500000, -0.866025,  1.000000,  1.000000 ),
        ( -0.500000,  0.866025,  1.000000,  1.000000 ),
        (  0.500000, -0.866025, -1.000000,  1.000000 ),
    ] * constants.num_drones)

    # Undo the steps that px4_generate_mixer.py will perform
    B_px = B_px_4dof
    for _ in range(2):
        B_px = np.insert(B_px, 3, np.zeros(constants.num_rotors * constants.num_drones), axis=1)
    B_px[:,5] *= -1

    geometry = {'rotors': [None] * constants.num_rotors}
    geometry['mix'] = {'B_px': B_px}
    name, description = geometry_name_desc(missing_drones)
    geometry['info'] = {}
    geometry['info']['key'] = name
    geometry['info']['name'] = name
    geometry['info']['description'] = description

    if len(missing_drones) == 0:
        geometry['drone_angles'] = [0, 0]

    return geometry


def generate_aviata_permutations():
    missing_drones_permutations = [[]]
    assert(constants.max_missing_drones == 0)

    combined_geometries = {}
    combined_geometries_list = []
    for missing_drones in missing_drones_permutations:
        geometry = generate_aviata_matrices(missing_drones)
        combined_geometries[geometry['info']['key']] = geometry
        combined_geometries_list.append(geometry)
    return combined_geometries, combined_geometries_list

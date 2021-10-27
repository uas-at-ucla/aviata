#!/usr/bin/env python3

# Derived from https://github.com/PX4/Firmware/blob/release/1.11/src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py

#############################################################################
#
#   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

"""
px_generate_mixers.py
Generates c/cpp header/source files for multirotor mixers
from geometry descriptions files (.toml format)
"""
import sys
import optimize_saturation

try:
    import toml
except ImportError as e:
    print("Failed to import toml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user toml")
    print("")
    sys.exit(1)

try:
    import numpy as np
except ImportError as e:
    print("Failed to import numpy: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user numpy")
    print("")
    sys.exit(1)

__author__ = "Julien Lecoeur"
__copyright__ = "Copyright (C) 2013-2017 PX4 Development Team."
__license__ = "BSD"
__email__ = "julien.lecoeur@gmail.com"


def parse_geometry_toml(filename):
    '''
    Parses toml geometry file and returns a dictionary with curated list of rotors
    '''
    import os

    # Load toml file
    d = toml.load(filename)

    # Check info section
    if 'info' not in d:
        raise AttributeError('{}: Error, missing info section'.format(filename))

    # Check info section
    for field in ['key', 'description']:
        if field not in d['info']:
            raise AttributeError('{}: Error, unspecified info field "{}"'.format(filename, field))

    # Use filename as mixer name
    d['info']['name'] = os.path.basename(filename).split('.')[0].lower()

    # Store filename
    d['info']['filename'] = filename

    # Check default rotor config
    if 'rotor_default' in d:
        default = d['rotor_default']
    else:
        default = {}

    # Convert rotors
    rotor_list = []
    if 'rotors' in d:
        for r in d['rotors']:
            # Make sure all fields are defined, fill missing with default
            for field in ['name', 'position', 'axis', 'direction', 'Ct', 'Cm']:
                if field not in r:
                    if field in default:
                        r[field] = default[field]
                    else:
                        raise AttributeError('{}: Error, unspecified field "{}" for rotor "{}"'
                                             .format(filename, field, r['name']))

            # Check direction field
            r['direction'] = r['direction'].upper()
            if r['direction'] not in ['CW', 'CCW']:
                raise AttributeError('{}: Error, invalid direction value "{}" for rotor "{}"'
                                     .format(filename, r['direction'], r['name']))

            # Check vector3 fields
            for field in ['position', 'axis']:
                if len(r[field]) != 3:
                    raise AttributeError('{}: Error, field "{}" for rotor "{}"'
                                         .format(filename, field, r['name']) +
                                         ' must be an array of length 3')

            # Add rotor to list
            rotor_list.append(r)

    # Clean dictionary
    geometry = {'info': d['info'],
            'rotors': rotor_list}

    return geometry

def torque_matrix(center, axis, dirs, Ct, Cm):
    '''
    Compute torque generated by rotors
    '''
    # normalize rotor axis
    ax = axis / np.linalg.norm(axis, axis=1)[:, np.newaxis]
    torque = Ct * np.cross(center, ax) - Cm * ax * dirs
    return torque

def geometry_to_torque_matrix(geometry):
    '''
    Compute torque matrix Am and Bm from geometry dictionnary
    Am is a 3xN matrix where N is the number of rotors
    Each column is the torque generated by one rotor
    '''
    Am = torque_matrix(center=np.array([rotor['position'] for rotor in geometry['rotors']]),
                       axis=np.array([rotor['axis'] for rotor in geometry['rotors']]),
                       dirs=np.array([[1.0 if rotor['direction'] == 'CCW' else -1.0]
                                      for rotor in geometry['rotors']]),
                       Ct=np.array([[rotor['Ct']] for rotor in geometry['rotors']]),
                       Cm=np.array([[rotor['Cm']] for rotor in geometry['rotors']])).T
    return Am

def thrust_matrix(axis, Ct):
    '''
    Compute thrust generated by rotors
    '''
    # Normalize rotor axis
    ax = axis / np.linalg.norm(axis, axis=1)[:, np.newaxis]
    thrust = Ct * ax
    return thrust

def geometry_to_thrust_matrix(geometry):
    '''
    Compute thrust matrix At from geometry dictionnary
    At is a 3xN matrix where N is the number of rotors
    Each column is the thrust generated by one rotor
    '''
    At = thrust_matrix(axis=np.array([rotor['axis'] for rotor in geometry['rotors']]),
                       Ct=np.array([[rotor['Ct']] for rotor in geometry['rotors']])).T

    return At

def geometry_to_mix(geometry):
    '''
    Compute combined torque & thrust matrix A and mix matrix B from geometry dictionnary

    A is a 6xN matrix where N is the number of rotors
    Each column is the torque and thrust generated by one rotor

    B is a Nx6 matrix where N is the number of rotors
    Each column is the command to apply to the servos to get
    roll torque, pitch torque, yaw torque, x thrust, y thrust, z thrust
    '''
    # Combined torque & thrust matrix
    At = geometry_to_thrust_matrix(geometry)
    Am = geometry_to_torque_matrix(geometry)
    A = np.vstack([Am, At])

    # Choose one of the methods below (pseudoinverse or optimize for minimal saturation)

    # Mix matrix computed as pseudoinverse of A
    B = np.linalg.pinv(A)

    # Optimal inverse to minimize motor saturation:
    # B = optimize_saturation.optimal_inverse(A)

    return A, B

def normalize_mix_px4(B, inertia):
    '''
    Normalize mix for PX4
    This has been modified for AVIATA such that setpoints can be given in [rad/s^2] and [2g's (twice gravitational acceleration)] (approximately)
    '''
    return B @ inertia

def generate_mixer_multirotor_header(geometries_list, use_normalized_mix=False, use_6dof=False, constants=None, max_missing_drones=0):
    '''
    Generate C header file with same format as multi_tables.py
    TODO: rewrite using templates (see generation of uORB headers)
    '''
    from io import StringIO
    buf = StringIO()

    # Print Header
    buf.write(u"/*\n")
    buf.write(u"* This file is automatically generated by px_generate_mixers.py - do not edit.\n")
    buf.write(u"*/\n")
    buf.write(u"\n")
    buf.write(u"#ifndef _AVIATA_MIXER_MULTI_TABLES\n")
    buf.write(u"#define _AVIATA_MIXER_MULTI_TABLES\n")
    buf.write(u"\n")
    if constants:
        buf.write(u"#define AVIATA_NUM_DRONES {}\n".format(constants.num_drones))
        buf.write(u"#define AVIATA_NUM_ROTORS {}\n".format(constants.num_rotors))
    buf.write(u"#define AVIATA_MAX_MISSING_DRONES {}\n".format(max_missing_drones))
    buf.write(u"\n")

    drone_angles = geometries_list[0]['drone_angles']
    for i in range(len(drone_angles)):
        while drone_angles[i] > np.pi:
            drone_angles[i] -= 2*np.pi
        while drone_angles[i] < -np.pi:
            drone_angles[i] += 2*np.pi

    buf.write(u"static constexpr float _config_aviata_drone_angle[] {\n")
    for drone_angle in drone_angles:
        buf.write(u"\t{:9f}, // {:.1f} degrees\n".format(drone_angle, np.rad2deg(drone_angle)))
    buf.write(u"};\n\n")
    buf.write(u"static constexpr float _config_aviata_drone_angle_cos[] {\n")
    for drone_angle in drone_angles:
        buf.write(u"\t{:9f},\n".format(np.cos(drone_angle)))
    buf.write(u"};\n\n")
    buf.write(u"static constexpr float _config_aviata_drone_angle_sin[] {\n")
    for drone_angle in drone_angles:
        buf.write(u"\t{:9f},\n".format(np.sin(drone_angle)))
    buf.write(u"};\n\n")

    buf.write(u"static constexpr float _config_aviata_relative_drone_angle[][{}] {{\n".format(len(drone_angles)))
    for drone_angle in drone_angles:
        buf.write(u"\t{ ")
        for drone_angle_ref in drone_angles:
            rel_angle = drone_angle - drone_angle_ref
            while rel_angle > np.pi:
                rel_angle -= 2*np.pi
            while rel_angle < -np.pi:
                rel_angle += 2*np.pi
            buf.write(u"{:9f}, ".format(rel_angle))
        buf.write(u"},\n")
    buf.write(u"};\n\n")

    buf.write(u"static constexpr float _config_aviata_relative_drone_angle_cos[][{}] {{\n".format(len(drone_angles)))
    for drone_angle in drone_angles:
        buf.write(u"\t{ ")
        for drone_angle_ref in drone_angles:
            buf.write(u"{:9f}, ".format(np.cos(drone_angle - drone_angle_ref)))
        buf.write(u"},\n")
    buf.write(u"};\n\n")

    buf.write(u"static constexpr float _config_aviata_relative_drone_angle_sin[][{}] {{\n".format(len(drone_angles)))
    for drone_angle in drone_angles:
        buf.write(u"\t{ ")
        for drone_angle_ref in drone_angles:
            buf.write(u"{:9f}, ".format(np.sin(drone_angle - drone_angle_ref)))
        buf.write(u"},\n")
    buf.write(u"};\n\n")
    
    # Print enum
    buf.write(u"enum class AviataMultirotorGeometry : MultirotorGeometryUnderlyingType {\n")
    for i, geometry in enumerate(geometries_list):
        buf.write(u"\t{},{}// {} (text key {})\n".format(
            geometry['info']['name'].upper(), ' ' * (max(0, 30 - len(geometry['info']['name']))),
            geometry['info']['description'], geometry['info']['key']))
    buf.write(u"\n\tMAX_GEOMETRY\n")
    buf.write(u"}; // enum class AviataMultirotorGeometry\n\n")

    # Print mixer gains
    buf.write(u"namespace {\n")
    for geometry in geometries_list:
        # Get desired mix matrix
        if use_normalized_mix:
            mix = geometry['mix']['B_px']
        else:
            mix = geometry['mix']['B']

        buf.write(u"static constexpr MultirotorMixer::Rotor _config_aviata_{}[] {{\n".format(geometry['info']['name']))

        for row in mix:
            if use_6dof:
            # 6dof mixer
                buf.write(u"\t{{ {:9f}, {:9f}, {:9f}, {:9f}, {:9f}, {:9f} }},\n".format(
                    row[0], row[1], row[2],
                    row[3], row[4], row[5]))
            else:
            # 4dof mixer
                buf.write(u"\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},\n".format(
                    row[0], row[1], row[2],
                    -row[5]))  # Upward thrust is positive TODO: to remove this, adapt PX4 to use NED correctly

        buf.write(u"};\n\n")

    # Print geometry indeces
    buf.write(u"static constexpr const MultirotorMixer::Rotor *_config_aviata_index[] {\n")
    for geometry in geometries_list:
        buf.write(u"\t&_config_aviata_{}[0],\n".format(geometry['info']['name']))
    buf.write(u"};\n\n")

    # Print geometry rotor counts
    buf.write(u"static constexpr unsigned _config_aviata_rotor_count[] {\n")
    for geometry in geometries_list:
        buf.write(u"\t{}, /* {} */\n".format(len(geometry['rotors']), geometry['info']['name']))
    buf.write(u"};\n\n")

    # Print geometry key
    buf.write(u"__attribute__((unused)) // Not really unused, but fixes compilation error\n")
    buf.write(u"const char* _config_aviata_key[] {\n")
    for geometry in geometries_list:
        buf.write(u"\t\"{}\",\t/* {} */\n".format(geometry['info']['key'], geometry['info']['name']))
    buf.write(u"};\n\n")

    # Print footer
    buf.write(u"} // anonymous namespace\n\n")
    buf.write(u"#endif /* _AVIATA_MIXER_MULTI_TABLES */\n\n")

    return buf.getvalue()


if __name__ == '__main__':
    import argparse
    import glob
    import os

    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Convert geometry .toml files to mixer headers')
    parser.add_argument('-d', dest='dir',
                        help='directory with geometry files')
    parser.add_argument('-f', dest='files',
                        help="files to convert (use only without -d)",
                        nargs="+")
    parser.add_argument('-o', dest='outputfile',
                        help='output header file')
    parser.add_argument('--verbose', help='Print details on standard output',
                        action='store_true')
    parser.add_argument('--normalize', help='Use normalized mixers (compatibility mode)',
                        action='store_true')
    parser.add_argument('--sixdof', help='Use 6dof mixers',
                        action='store_true')
    args = parser.parse_args()

    # Find toml files
    if args.files is not None:
        filenames = args.files
    elif args.dir is not None:
        filenames = glob.glob(os.path.join(args.dir, '*.toml'))
    else:
        parser.print_usage()
        raise Exception("Missing input directory (-d) or list of geometry files (-f)")

    # List of geometries
    geometries_list = []

    for filename in filenames:
        # Parse geometry file
        geometry = parse_geometry_toml(filename)

        # Compute torque and thrust matrices
        A, B = geometry_to_mix(geometry)

        # Normalize mixer
        B_px = normalize_mix_px4(B)

        # Store matrices in geometry
        geometry['mix'] = {'A': A, 'B': B, 'B_px': B_px}

        # Add to list
        geometries_list.append(geometry)

        if args.verbose:
            print('\nFilename')
            print(filename)
            print('\nGeometry')
            print(geometry)
            print('\nA:')
            print(A.round(2))
            print('\nB:')
            print(B.round(2))
            print('\nNormalized Mix (as in PX4):')
            print(B_px.round(2))
            print('\n-----------------------------')

    # Check that there are no duplicated mixer names or keys
    for i in range(len(geometries_list)):
        name_i = geometries_list[i]['info']['name']
        key_i = geometries_list[i]['info']['key']

        for j in range(i + 1, len(geometries_list)):
            name_j = geometries_list[j]['info']['name']
            key_j = geometries_list[j]['info']['key']

            # Mixers cannot share the same name
            if name_i == name_j:
                raise ValueError('Duplicated mixer name "{}" in files {} and {}'.format(
                    name_i,
                    geometries_list[i]['info']['filename'],
                    geometries_list[j]['info']['filename']))

            # Mixers cannot share the same key
            if key_i == key_j:
                raise ValueError('Duplicated mixer key "{}" for mixers "{}" and "{}"'.format(
                    key_i, name_i, name_j))

    # Generate header file
    header = generate_mixer_multirotor_header(geometries_list,
                                              use_normalized_mix=args.normalize,
                                              use_6dof=args.sixdof)

    if args.outputfile is not None:
        # Write header file
        with open(args.outputfile, 'w') as fd:
            fd.write(header)
    else:
        # Print to standard output
        print(header)

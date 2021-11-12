#!/usr/bin/env python3

from OpenGL.raw.GLUT import constants
import numpy as np
import px4_mixer_multirotor
import config
import matplotlib.pyplot as plt
import pandas as pd
import sys
import pyulog

if __name__ == '__main__':
    # Usage: ./verify_motor_outputs.py <log_file_path> <this_drone_docking_slot> [other_drone_docking_slot]
    log_file_path = sys.argv[1]
    this_drone_docking_slot = int(sys.argv[2]) # drone that the log file is from
    other_drone_docking_slot = this_drone_docking_slot # drone to show expected actuator outputs for. If it is a different drone, they won't be very accurate.
    if len(sys.argv) >= 4:
        other_drone_docking_slot = int(sys.argv[3])

    ulog = pyulog.ULog(log_file_path)
    params = ulog.initial_parameters
    PWM_MIN = params['PWM_AUX_MIN']
    PWM_MAX = params['PWM_AUX_MAX']
    THR_MDL_FAC = params['THR_MDL_FAC']
    actuator_controls_data = ulog.get_dataset('actuator_controls_0').data

    rotor_index_start = other_drone_docking_slot * config.constants.num_rotors
    rotor_index_end = rotor_index_start + config.constants.num_rotors

    missing_drones = [] # 0 through 7
    geometry = config.generate_matrices.generate_aviata_matrices(missing_drones, optimize=False) # 10/30/21 tests did not use optimized mixer
    mixer = geometry['mix']['B_px_4dof']

    motor_outputs = [[] for _ in range(config.constants.num_rotors)]
    step = 10 # downsample to speed up processing
    for t in range(0, len(actuator_controls_data['timestamp']), step):
        setpoint = np.matrix([actuator_controls_data['control[0]'][t], actuator_controls_data['control[1]'][t], actuator_controls_data['control[2]'][t], actuator_controls_data['control[3]'][t]]).T

        # rotate setpoint based on docking slot
        roll_command = setpoint[0]
        pitch_command = setpoint[1]
        setpoint[0] = roll_command * np.cos(geometry['drone_angles'][this_drone_docking_slot]) - pitch_command * np.sin(geometry['drone_angles'][this_drone_docking_slot])
        setpoint[1] = roll_command * np.sin(geometry['drone_angles'][this_drone_docking_slot]) + pitch_command * np.cos(geometry['drone_angles'][this_drone_docking_slot])

        u, u_final = px4_mixer_multirotor.normal_mode(setpoint, mixer[rotor_index_start:rotor_index_end,:], 0.0, 1.0)
        # u_final = mixer@setpoint # Use this to test without desaturation
        outputs = np.squeeze(np.array(u_final))
        for i in range(config.constants.num_rotors):
            # See https://github.com/PX4/PX4-Autopilot/blob/release/1.11/src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp#L373
            if THR_MDL_FAC > 0:
                outputs[i] = -(1 - THR_MDL_FAC) / (2 * THR_MDL_FAC) \
                             + np.sqrt((1 - THR_MDL_FAC) * (1 - THR_MDL_FAC) / (4 * THR_MDL_FAC * THR_MDL_FAC)
                                       + (0 if outputs[i] < 0 else outputs[i] / THR_MDL_FAC))

            outputs[i] = outputs[i] * (PWM_MAX - PWM_MIN) + PWM_MIN
            motor_outputs[i].append(outputs[i])

    colors = ['red', 'green', 'blue', 'black', 'gray', 'orange'] # as seen on https://review.px4.io

    timestamps = pd.to_datetime(actuator_controls_data['timestamp'][0::step], unit='us')

    plt.figure()
    plt.plot(timestamps, actuator_controls_data['control[0]'][0::step], label="Roll", color=colors[0])
    plt.plot(timestamps, actuator_controls_data['control[1]'][0::step], label="Pitch", color=colors[1])
    plt.plot(timestamps, actuator_controls_data['control[2]'][0::step], label="Yaw", color=colors[2])
    plt.plot(timestamps, actuator_controls_data['control[3]'][0::step], label="Thrust", color=colors[3])
    plt.legend()

    plt.figure()
    for i in range(config.constants.num_rotors):
        plt.plot(timestamps, motor_outputs[i], label="Output {}".format(i), color=colors[i])
    plt.ylim(1000, 2000)
    plt.legend()

    plt.show()

from dataclasses import dataclass
import numpy as np
from pyquaternion import Quaternion
import constants
import px4_mixer_multirotor
import generate_matrices
from pid import PID

combined_geometries, drone_geometries, drone_geometries_list = generate_matrices.generate_aviata_permutations(max_missing_drones=3)

@dataclass
class DroneSensors:
    pos = np.array([0.0, 0.0, 0.0]) # position (m)
    vel = np.array([0.0, 0.0, 0.0]) # velocity (m/s)
    att = Quaternion() # attitude (quaternion)
    att_rate = np.array([0.0, 0.0, 0.0]) # angular velocity (rad/s)


class Drone:
    def __init__(self):
        self.network = None
        self.drone_pos = None # numbered location on the stucture
        self.is_master = False
        self.mixer = None
        self.hover_thrust = 0
        self.sensors = DroneSensors()
        self.forces_setpoint = np.matrix([0.0, 0.0, 0.0, 0.0]).T
        self.pos_setpoint = None
        self.yaw_setpoint = None
        self.vel_pid = None
        self.att_rate_pid = None
        self.motor_inputs = np.zeros((4,1))

    def set_network(self, network):
        self.network = network

    def set_drone_pos(self, drone_pos):
        self.drone_pos = drone_pos

    def configure_mixer(self, missing_drones):
        if self.drone_pos is not None:
            mixer_name = generate_matrices.mixer_name(self.drone_pos, missing_drones)
            self.mixer = drone_geometries[mixer_name]['mix']['B_px_4dof']
            self.hover_thrust = drone_geometries[mixer_name]['thr_hover']

    def set_sensors(self, sensors):
        self.sensors = sensors

    def set_forces_setpoint(self, setpoint):
        self.is_master = False
        self.forces_setpoint = setpoint

    def set_pos_setpoint(self, pos_setpoint, yaw_setpoint):
        if not self.is_master:
            self.is_master = True
            self.vel_pid = PID(constants.P_vel, constants.I_vel, constants.D_vel)
            next(self.vel_pid)
            self.att_rate_pid = PID(constants.P_att_rate, constants.I_att_rate, constants.D_att_rate)
            next(self.att_rate_pid)
        self.pos_setpoint = pos_setpoint
        self.yaw_setpoint = yaw_setpoint

    def control_loop(self, dt):
        if self.mixer is not None:
            if self.is_master:
                #TODO Might want to add hard limits on velocity, acceleration, etc. (more PX4 parameters)

                # Roughly based on PX4 v1.11. See https://uasatucla.org/en/subsystems/controls/multirotor-physics-and-controls#part-3-control-the-control-system-architecture
                pos_err = self.pos_setpoint - self.sensors.pos
                vel_sp = constants.P_pos * pos_err

                acc_sp = self.vel_pid.send([self.sensors.vel, vel_sp, dt])

                # See https://github.com/PX4/Firmware/blob/release/1.11/src/modules/mc_pos_control/PositionControl/PositionControl.cpp#L192
                att_z_vec = -np.array([acc_sp[0], acc_sp[1], -constants.g])
                att_z_vec /= np.linalg.norm(att_z_vec)

                # achieve desired vertical thrust assuming we're facing the direction of att_z_vec (might make more sense to use current attitude (?) but this is how PX4 does it)
                thr_body_sp_z = -acc_sp[2] * (self.hover_thrust / constants.g) + self.hover_thrust
                thr_body_sp_z /= np.dot(np.array([0, 0, 1]), att_z_vec)

                # determine att_sp quaternion using att_z_vec and yaw_setpoint (see https://github.com/PX4/Firmware/blob/release/1.11/src/modules/mc_pos_control/PositionControl/ControlMath.cpp#L70)
                yaw_vec = np.exp(1j * self.yaw_setpoint)
                yaw_vec = np.array([np.real(yaw_vec), np.imag(yaw_vec), 0.0])
                att_y_vec = np.cross(att_z_vec, yaw_vec)
                att_y_vec /= np.linalg.norm(att_y_vec)
                att_x_vec = np.cross(att_y_vec, att_z_vec)
                att_rot_matrix = np.zeros((3,3))
                att_rot_matrix[:,0] = att_x_vec
                att_rot_matrix[:,1] = att_y_vec
                att_rot_matrix[:,2] = att_z_vec
                att_sp = Quaternion(matrix=att_rot_matrix)

                att_err_q = att_sp * self.sensors.att.inverse
                att_err = self.sensors.att.inverse.rotate(att_err_q.axis) * att_err_q.angle
                att_rate_sp = constants.P_att * att_err

                torque_sp = self.att_rate_pid.send([self.sensors.att_rate, att_rate_sp, dt])

                self.forces_setpoint = np.matrix([0.0, 0.0, 0.0, 0.0]).T
                self.forces_setpoint[:3,0] = torque_sp[:, np.newaxis]
                self.forces_setpoint[3,0] = thr_body_sp_z

                self.network.broadcast_from(self, Drone.set_forces_setpoint, self.forces_setpoint)

            u, u_final = px4_mixer_multirotor.normal_mode(self.forces_setpoint, self.mixer, 0.0, 1.0)
            self.motor_inputs = u_final


class Network:
    def __init__(self, drones):
        self.drones = drones
        for i in range(len(self.drones)):
            self.drones[i].set_network(self)

    def broadcast(self, drone_func, *args, **kwargs):
        for drone in self.drones:
            drone_func(drone, *args, **kwargs)

    def broadcast_from(self, src_drone, drone_func, *args, **kwargs):
        for drone in self.drones:
            if not drone is src_drone:
                drone_func(drone, *args, **kwargs)


@dataclass
class Structure:
    pos = np.array([0.0, 0.0, 0.0]) # position (m)
    vel = np.array([0.0, 0.0, 0.0]) # velocity (m/s)
    att = Quaternion() # attitude (quaternion)
    att_rate = np.array([0.0, 0.0, 0.0]) # angular velocity (rad/s)
    ang_acc = np.array([0.0, 0.0, 0.0]) # angular acceleration (rad/s^2)
    lin_acc = np.array([0.0, 0.0, 0.0]) # linear acceleration (m/s^2)
    u = None
    geometry = None


class PhysicalWorld:
    def __init__(self, num_drones, sample_period_ms):
        self.drones = []
        for i in range(num_drones):
            self.drones.append(Drone())
        self.master_drone = None
        self.network = Network(self.drones)
        for drone in self.drones:
            drone.set_network(self.network)

        self.structure = Structure()
        self.sample_period_ms = sample_period_ms
        self.sample_period_s = sample_period_ms / 1000

    def set_missing_drones(self, missing_drones):
        # Logic of assigning drones to locations is not important to the simulation, so we just put it here.
        # Detach drones that should be missing:
        for drone in self.drones:
            if drone.drone_pos in missing_drones:
                drone.set_drone_pos(None)
        # Initialize empty_slots to list all structure positions:
        empty_slots = list(range(constants.num_drones))
        # Exclude positions that should be empty from empty_slots:
        for missing_drone in missing_drones:
            empty_slots.remove(missing_drone)
        # Exclude occupied positions from empty_slots:
        for drone in self.drones:
            if drone.drone_pos in empty_slots:
                empty_slots.remove(drone.drone_pos)
        # Fill empty positions that need to be filled with available drones:
        for drone in self.drones:
            if len(empty_slots) == 0:
                break
            if drone.drone_pos is None:
                drone.set_drone_pos(empty_slots.pop(0))
        # Assign new master if needed:
        if self.master_drone is None or self.master_drone.drone_pos is None:
            for drone in self.drones:
                if drone.drone_pos is not None:
                    self.master_drone = drone
                    break

        self.network.broadcast(Drone.configure_mixer, missing_drones)

        self.structure.geometry = combined_geometries[generate_matrices.geometry_name(missing_drones)]

    def tick(self):
        att_rate_magnitude = np.linalg.norm(self.structure.att_rate)
        if att_rate_magnitude != 0:
            att_rate_direction = self.structure.att_rate / np.linalg.norm(self.structure.att_rate)
            self.structure.att = Quaternion(axis=att_rate_direction, angle=(att_rate_magnitude * self.sample_period_s)) * self.structure.att
        self.structure.pos += self.structure.vel * self.sample_period_s

        self.structure.att_rate += self.structure.ang_acc * self.sample_period_s
        self.structure.vel += self.structure.lin_acc * self.sample_period_s

        sensors = DroneSensors()
        sensors.pos = self.structure.pos
        sensors.vel = self.structure.vel
        sensors.att = self.structure.att
        sensors.att_rate = self.structure.att_rate
        for drone in self.drones:
            if drone.drone_pos is not None:
                drone.set_sensors(sensors)

        if self.master_drone:
            self.master_drone.control_loop(self.sample_period_s)
        for drone in self.drones:
            if not drone is self.master_drone:
                drone.control_loop(self.sample_period_s)

        actuator_effectiveness = self.structure.geometry['mix']['A_4dof']
        combined_mixer = self.structure.geometry['mix']['B_px_4dof']
        # ideal_forces = np.linalg.multi_dot([actuator_effectiveness, combined_mixer, setpoint]) # TODO retrieve setpoint from one of the drones
        self.structure.u = np.zeros([constants.num_drones * constants.num_rotors, 1])
        for drone in self.drones:
            if drone.drone_pos is not None:
                rotor_start = drone.drone_pos * constants.num_rotors
                rotor_end = rotor_start + constants.num_rotors
                self.structure.u[rotor_start:rotor_end] = drone.motor_inputs
        forces = np.dot(actuator_effectiveness, self.structure.u)[:,0]

        torque = forces[:3]
        force = forces[3]

        ang_acc = np.dot(self.structure.geometry['Iinv'], torque - np.cross(self.structure.att_rate, np.dot(self.structure.geometry['I'], self.structure.att_rate)))
        lin_acc = force / self.structure.geometry['M']

        # transform from body frame to global frame
        self.structure.ang_acc = self.structure.att.rotate(ang_acc)
        self.structure.lin_acc = self.structure.att.rotate(np.array((0.0, 0.0, lin_acc)))
        self.structure.lin_acc[2] += constants.g # gravity is a thing

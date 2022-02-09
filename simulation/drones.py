from dataclasses import dataclass
import numpy as np
from pyquaternion import Quaternion
import config
import px4_mixer_multirotor
from pid import PID

combined_geometries, combined_geometries_list = config.generate_matrices.generate_aviata_permutations()


def constrain(val, min, max):
    return np.minimum(max, np.maximum(min, val))

def limit_norm(val, max_norm):
    norm = np.linalg.norm(val)
    if norm > max_norm:
        return (val / norm) * max_norm
    return val


# Avoid some redundant calculations in order to run faster
forces_setpoint_cache = np.matrix([0.0, 0.0, 0.0, 0.0]).T
mixer_cache = None
u_cache = None
u_final_cache = None
def px4_normal_mode(forces_setpoint, mixer):
    global forces_setpoint_cache
    global mixer_cache
    global u_cache
    global u_final_cache
    if not (np.array_equal(forces_setpoint, forces_setpoint_cache) and mixer is mixer_cache):
        u_cache, u_final_cache = px4_mixer_multirotor.normal_mode(forces_setpoint, mixer, 0.0, 1.0)
        forces_setpoint_cache = forces_setpoint
        mixer_cache = mixer
    return u_cache, u_final_cache


@dataclass
class DroneSensors:
    pos = np.array([0.0, 0.0, 0.0]) # position (m)
    vel = np.array([0.0, 0.0, 0.0]) # velocity (m/s)
    att = Quaternion() # attitude (quaternion)
    att_rate = np.array([0.0, 0.0, 0.0]) # angular velocity (rad/s)


CONTROL_MODE_LEADER = "CONTROL_MODE_LEADER"
CONTROL_MODE_FOLLOWER = "CONTROL_MODE_FOLLOWER"
CONTROL_MODE_FORCES = "CONTROL_MODE_FORCES"

class Drone:
    def __init__(self):
        self.network = None
        self.drone_pos = None # numbered location on the stucture
        self.control_mode = CONTROL_MODE_FORCES
        self.mixer = None
        self.hover_thrust = 0
        self.rotor_start_index = 0
        self.rotor_end_index = 0
        self.sensors = DroneSensors()
        self.forces_setpoint = np.matrix([0.0, 0.0, 0.0, 0.0]).T
        self.att_setpoint = None
        self.thrust_setpoint = None
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
            geometry_name = config.generate_matrices.geometry_name(missing_drones)
            self.mixer = combined_geometries[geometry_name]['mix']['B_px_4dof']
            self.hover_thrust = combined_geometries[geometry_name]['thr_hover']
            self.rotor_start_index = self.drone_pos * config.constants.num_rotors
            self.rotor_end_index = self.rotor_start_index + config.constants.num_rotors

    def set_sensors(self, sensors):
        self.sensors = sensors

    def set_forces_setpoint(self, setpoint):
        self.control_mode = CONTROL_MODE_FORCES
        self.forces_setpoint = setpoint

    def set_att_thrust_setpoint(self, att_sp, thr_body_sp_z):
        if not self.control_mode == CONTROL_MODE_FOLLOWER:
            if not self.control_mode == CONTROL_MODE_LEADER:
                self.att_rate_pid = PID(config.constants.P_att_rate, config.constants.I_att_rate, config.constants.D_att_rate)
                next(self.att_rate_pid)
            self.control_mode = CONTROL_MODE_FOLLOWER
        self.att_setpoint = att_sp
        self.thrust_setpoint = thr_body_sp_z

    def set_pos_setpoint(self, pos_sp, yaw_sp):
        if not self.control_mode == CONTROL_MODE_LEADER:
            if not self.control_mode == CONTROL_MODE_FOLLOWER:
                self.att_rate_pid = PID(config.constants.P_att_rate, config.constants.I_att_rate, config.constants.D_att_rate)
                next(self.att_rate_pid)
            self.vel_pid = PID(config.constants.P_vel, config.constants.I_vel, config.constants.D_vel)
            next(self.vel_pid)
            self.control_mode = CONTROL_MODE_LEADER
        self.pos_setpoint = pos_sp
        self.yaw_setpoint = yaw_sp

    def control_loop(self, dt):
        if self.mixer is not None:
            if self.control_mode == CONTROL_MODE_LEADER:
                # Roughly based on PX4 v1.11. See https://uasatucla.org/en/subsystems/controls/multirotor-physics-and-controls#part-3-control-the-control-system-architecture
                pos_err = self.pos_setpoint - self.sensors.pos
                vel_sp = config.constants.P_pos * pos_err
                vel_sp[:2] = limit_norm(vel_sp[:2], config.constants.max_vel_hor)
                vel_sp[2] = constrain(vel_sp[2], -config.constants.max_vel_up, config.constants.max_vel_down)

                acc_sp = self.vel_pid.send([self.sensors.vel, vel_sp, dt])
                acc_sp[:2] = limit_norm(acc_sp[:2], config.constants.max_acc_hor)
                acc_sp[2] = constrain(acc_sp[2], -config.constants.max_acc_up, config.constants.max_acc_down)

                # See https://github.com/PX4/Firmware/blob/release/1.11/src/modules/mc_pos_control/PositionControl/PositionControl.cpp#L192
                att_z_vec = -np.array([acc_sp[0], acc_sp[1], -config.constants.g])
                att_z_vec /= np.linalg.norm(att_z_vec)

                # achieve desired vertical thrust assuming we're facing the direction of att_z_vec (might make more sense to use current attitude (?) but this is how PX4 does it)
                self.thrust_setpoint = -acc_sp[2] * (self.hover_thrust / config.constants.g) + self.hover_thrust
                self.thrust_setpoint /= np.dot(np.array([0, 0, 1]), att_z_vec)

                # determine att_sp quaternion using att_z_vec and yaw_setpoint (see https://github.com/PX4/Firmware/blob/release/1.11/src/modules/mc_pos_control/PositionControl/ControlMath.cpp#L70)
                heading = np.exp(1j * self.yaw_setpoint)
                att_y_dir = np.array([-np.imag(heading), np.real(heading), 0.0])
                att_x_vec = np.cross(att_y_dir, att_z_vec)
                att_x_vec /= np.linalg.norm(att_x_vec)
                att_y_vec = np.cross(att_z_vec, att_x_vec)
                att_rot_matrix = np.zeros((3,3))
                att_rot_matrix[:,0] = att_x_vec
                att_rot_matrix[:,1] = att_y_vec
                att_rot_matrix[:,2] = att_z_vec
                self.att_setpoint = Quaternion(matrix=att_rot_matrix)

                self.network.broadcast_from(self, Drone.set_att_thrust_setpoint, self.att_setpoint, self.thrust_setpoint)

            if self.control_mode == CONTROL_MODE_LEADER or self.control_mode == CONTROL_MODE_FOLLOWER:
                # See https://github.com/PX4/Firmware/blob/release/1.11/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp#L83
                att_err_q = self.sensors.att.inverse * self.att_setpoint
                if att_err_q[0] < 0:
                    att_err_q = -att_err_q # put quaternion in "canonical" form
                att_err = 2 * att_err_q.imaginary # PX4 approximation of att_err_q.axis * math.sin(att_err_q.angle/2) = att_err_q.imaginary
                att_rate_sp = config.constants.P_att * att_err
                att_rate_sp = constrain(att_rate_sp, -config.constants.max_att_rate, config.constants.max_att_rate)

                att_rate_body = self.sensors.att.inverse.rotate(self.sensors.att_rate)
                torque_sp = self.att_rate_pid.send([att_rate_body, att_rate_sp, dt])

                self.forces_setpoint = np.matrix([0.0, 0.0, 0.0, 0.0]).T
                self.forces_setpoint[:3,0] = torque_sp[:, np.newaxis]
                self.forces_setpoint[3,0] = self.thrust_setpoint

            u, u_final = px4_normal_mode(self.forces_setpoint, self.mixer)
            # u_final = np.dot(self.mixer, self.forces_setpoint)
            # u_final = constrain(u_final, 0, 1)
            self.motor_inputs = u_final[self.rotor_start_index:self.rotor_end_index]


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
        self.leader_drone = None
        self.network = Network(self.drones)
        for drone in self.drones:
            drone.set_network(self.network)

        self.structure = Structure()
        self.sample_period_ms = sample_period_ms
        self.sample_period_s = sample_period_ms / 1000

    def set_missing_drones(self, missing_drones):
        # Logic of assigning drones to locations is not important to the simulation, so we just put it here.
        geometry_name = config.generate_matrices.geometry_name(missing_drones)

        if geometry_name not in combined_geometries:
            return False

        self.structure.geometry = combined_geometries[geometry_name]

        # Detach drones that should be missing:
        for drone in self.drones:
            if drone.drone_pos in missing_drones:
                drone.set_drone_pos(None)
        # Initialize empty_slots to list all structure positions:
        empty_slots = list(range(config.constants.num_drones))
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
        # Assign new leader if needed:
        if self.leader_drone is None or self.leader_drone.drone_pos is None:
            for drone in self.drones:
                if drone.drone_pos is not None:
                    self.leader_drone = drone
                    break

        self.network.broadcast(Drone.configure_mixer, missing_drones)
        return True

    def tick(self):
        prev_att_rate = self.structure.att_rate
        prev_vel = self.structure.vel

        self.structure.att_rate += self.structure.ang_acc * self.sample_period_s
        self.structure.vel += self.structure.lin_acc * self.sample_period_s

        avg_att_rate = (prev_att_rate + self.structure.att_rate) / 2
        avg_vel = (prev_vel + self.structure.vel) / 2

        att_rate_magnitude = np.linalg.norm(avg_att_rate)
        if att_rate_magnitude != 0:
            att_rate_direction = avg_att_rate / att_rate_magnitude
            self.structure.att = Quaternion(axis=att_rate_direction, angle=(att_rate_magnitude * self.sample_period_s)) * self.structure.att
        self.structure.pos += avg_vel * self.sample_period_s

        sensors = DroneSensors()
        sensors.pos = self.structure.pos
        sensors.vel = self.structure.vel
        sensors.att = self.structure.att
        sensors.att_rate = self.structure.att_rate
        for drone in self.drones:
            if drone.drone_pos is not None:
                drone.set_sensors(sensors)

        if self.leader_drone:
            self.leader_drone.control_loop(self.sample_period_s)
        for drone in self.drones:
            if not drone is self.leader_drone:
                drone.control_loop(self.sample_period_s)

        actuator_effectiveness = self.structure.geometry['mix']['A']
        combined_mixer = self.structure.geometry['mix']['B_px_4dof']
        # ideal_forces = np.linalg.multi_dot([actuator_effectiveness, combined_mixer, setpoint]) # TODO retrieve setpoint from one of the drones (this probably isn't important though)
        self.structure.u = np.zeros([config.constants.num_drones * config.constants.num_rotors, 1])
        for drone in self.drones:
            if drone.drone_pos is not None:
                rotor_start = drone.drone_pos * config.constants.num_rotors
                rotor_end = rotor_start + config.constants.num_rotors
                self.structure.u[rotor_start:rotor_end] = drone.motor_inputs
        forces = np.dot(actuator_effectiveness, self.structure.u)[:,0]

        torque = forces[:3]
        force = forces[3:]

        ang_acc = np.dot(self.structure.geometry['Iinv'], torque - np.cross(self.structure.att_rate, np.dot(self.structure.geometry['I'], self.structure.att_rate)))
        lin_acc = force / self.structure.geometry['M']

        # transform from body frame to global frame
        self.structure.ang_acc = self.structure.att.rotate(ang_acc)
        self.structure.lin_acc = self.structure.att.rotate(lin_acc)
        self.structure.lin_acc[2] += config.constants.g # gravity is a thing

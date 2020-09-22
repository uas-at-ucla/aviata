from dataclasses import dataclass
import numpy as np
from pyquaternion import Quaternion
import constants
import px4_mixer_multirotor
import generate_matrices

combined_geometries, drone_geometries, drone_geometries_list = generate_matrices.generate_aviata_permutations(max_missing_drones=3)

class Drone:
    def __init__(self):
        self.network = None
        self.drone_pos = None # numbered location on the stucture
        self.mixer = None
        # self.physical_state = None # used by higher-level controllers
        self.forces_setpoint = np.matrix([0.0, 0.0, 0.0, 0.0]).T
        self.motor_inputs = np.zeros(4)

    def set_network(self, network):
        self.network = network

    def set_drone_pos(self, drone_pos):
        self.drone_pos = drone_pos

    def configure_mixer(self, missing_drones):
        if self.drone_pos is not None:
            self.mixer = drone_geometries[generate_matrices.mixer_name(self.drone_pos, missing_drones)]['mix']['B_px_4dof']

    def set_forces_setpoint(self, setpoint):
        self.forces_setpoint = setpoint

    def control_loop(self):
        if self.mixer is not None:
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
    att = Quaternion(axis=[0, 0, 1], angle=0) # attitude (quaternion)
    att_rate = np.array([0.0, 0.0, 0.0]) # angular velocity (rad/s)
    ang_acc = np.array([0.0, 0.0, 0.0]) # angular acceleration (rad/s^2)
    lin_acc = np.array([0.0, 0.0, 0.0]) # linear acceleration (m/s^2)
    actuator_effectiveness = None


class PhysicalWorld:
    def __init__(self, num_drones, sample_period_ms):
        self.drones = []
        for i in range(num_drones):
            self.drones.append(Drone())
        self.network = Network(self.drones)
        for drone in self.drones:
            drone.set_network(self.network)

        self.structure = Structure()
        self.sample_period_ms = sample_period_ms
        self.sample_period_s = sample_period_ms / 1000

    def set_missing_drones(self, missing_drones):
        for drone in self.drones:
            if drone.drone_pos in missing_drones:
                drone.set_drone_pos(None)
        empty_slots = list(range(constants.num_drones))
        for missing_drone in missing_drones:
            empty_slots.remove(missing_drone)
        for drone in self.drones:
            if drone.drone_pos in empty_slots:
                empty_slots.remove(drone.drone_pos)
        for drone in self.drones:
            if drone.drone_pos is None:
                drone.set_drone_pos(empty_slots.pop(0))
            if len(empty_slots) == 0:
                break

        self.network.broadcast(Drone.configure_mixer, missing_drones)

        self.structure.actuator_effectiveness = combined_geometries[generate_matrices.geometry_name(missing_drones)]['mix']['A_4dof']

    def tick(self):
        att_rate_magnitude = np.linalg.norm(self.structure.att_rate)
        if att_rate_magnitude != 0:
            att_rate_direction = self.structure.att_rate / np.linalg.norm(self.structure.att_rate)
            self.structure.att = Quaternion(axis=att_rate_direction, angle=(att_rate_magnitude * self.sample_period_s)) * self.structure.att
        self.structure.pos += self.structure.vel * self.sample_period_s

        self.structure.att_rate += self.structure.ang_acc * self.sample_period_s
        self.structure.vel += self.structure.lin_acc * self.sample_period_s

        for drone in self.drones:
            drone.control_loop()

        # ideal_forces = np.linalg.multi_dot([actuator_effectiveness, mixer, setpoint])
        u = np.zeros([constants.num_drones * constants.num_rotors, 1])
        for drone in self.drones:
            if drone.drone_pos is not None:
                rotor_start = drone.drone_pos * constants.num_rotors
                rotor_end = rotor_start + constants.num_rotors
                u[rotor_start:rotor_end] = drone.motor_inputs
        forces = np.dot(self.structure.actuator_effectiveness, u)

        torque = forces[:3]
        force = forces[3,0]

        ang_acc = np.dot(constants.Iinv, torque)[:,0]
        lin_acc = force / constants.M

        # transform from body frame to global frame
        self.structure.ang_acc = self.structure.att.rotate(ang_acc)
        self.structure.lin_acc = self.structure.att.rotate(np.array((0.0, 0.0, lin_acc)))
        self.structure.lin_acc[2] += 9.81 # gravity is a thing

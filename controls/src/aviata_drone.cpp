#include "lib/drone.hpp"
#include "lib/util.hpp"
#include <unistd.h>
#include <algorithm>

int main(int argc, char** argv) {
    if (argc >= 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        std::cout << "Usage: ./aviata_drone [drone_id] [initial_state] [docking_slot] [connection_url]" << std::endl;
        std::cout << "    Initial states: standby, follower, leader" << std::endl;
        std::cout << "    With no arguments, this starts a drone in STANDBY with ID based on the machine hostname, and connects to the RPi serial port." << std::endl;
        return 0;
    }

    std::string drone_id;
    if (argc >= 2) {
        drone_id = argv[1];
    } else {
        char hostname[256] = "";
        gethostname(hostname, sizeof(hostname));
        drone_id = hostname;
        if (drone_id.rfind("rpi-", 0) == 0) { // if hostname starts with "rpi-"
            drone_id = drone_id.substr(4);
        }
        std::replace(drone_id.begin(), drone_id.end(), '-', '_');
    }

    DroneState initial_state = STANDBY;
    if (argc >= 3) {
        if (strcmp(argv[2], "follower") == 0) {
            initial_state = DOCKED_FOLLOWER;
        } else if (strcmp(argv[2], "leader") == 0) {
            initial_state = DOCKED_LEADER;
        } else if (strcmp(argv[2], "standby") == 0) {
            initial_state = STANDBY;
        } else {
            std::cout << "Invalid initial state: " << argv[2] << std::endl;
            return 1;
        }
    }

    int8_t docking_slot = -1;
    if (argc >= 4) {
        docking_slot = atoi(argv[3]);
    }

    std::string connection_url = "serial:///dev/ttyAMA0:921600";
    if (argc >= 5) {
        connection_url = argv[4];
    }

    DroneSettings drone_settings;

    // TODO change for different frame configurations
    drone_settings.n_docking_slots = 4; // number of drones on the full frame
    drone_settings.max_missing_drones = 0;

    if (connection_url.rfind("udp://", 0) == 0) { // if connection_url starts with "udp://"
        // If using UDP, assume we're in the simulator that does not support physical docking.
        drone_settings.sim = true;
        drone_settings.modify_px4_mixers = false;
    } else  {
        drone_settings.sim = false;
        drone_settings.modify_px4_mixers = true;
    }

    Target t;

    Drone drone(drone_id, drone_settings, t);

    if (!drone.init(initial_state, docking_slot, connection_url)) {
        return 1;
    }

    drone.run();
}

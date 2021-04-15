#include "lib/drone.hpp"
#include "lib/util.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: ./aviata_drone <drone_id> <initial_state> [docking_slot] [connection_url]" << std::endl;
        std::cout << "initial states: follower, leader" << std::endl;
        return 1;
    }

    std::string drone_id = argv[1];

    DroneState initial_state;
    if (strcmp(argv[2], "follower") == 0) {
        initial_state = DOCKED_FOLLOWER;
    } else if (strcmp(argv[2], "leader") == 0) {
        initial_state = DOCKED_LEADER;
    } else {
        std::cout << "Invalid initial state: " << argv[2] << std::endl;
        return 1;
    }

    uint8_t docking_slot = 0;
    if (argc >= 4) {
        docking_slot = atoi(argv[3]);
    }

    std::string connection_url = "udp://:14540";
    if (argc >= 5) {
        connection_url = argv[4];
    }

    DroneSettings drone_settings;
    if (connection_url.rfind("udp://", 0) == 0) { // if connection_url starts with "udp://"
        // If using UDP, assume we're in the simulator that does not support physical docking.
        drone_settings.sim = true;
        drone_settings.modify_px4_mixers = false; 
    } else  {
        drone_settings.sim = false;
        drone_settings.modify_px4_mixers = true; 
    }

    Target t;

    Drone drone(drone_id, drone_settings,t);

    if (!drone.init(initial_state, docking_slot, connection_url)) {
        return 1;
    }

    drone.run();
}

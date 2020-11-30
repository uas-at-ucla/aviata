#include "lib/px4_io.hpp"
#include "lib/network.hpp"
#include "lib/drone.hpp"
#include <map>

//enum DroneState now in drone.hpp

// Drone SWARM[10]; // eventually switch to dynamic size?

std::map<std::string, DroneStatus> SWARM; // map by ID

int main(int argc, char** argv) {
    takeoff_and_land_test(argc, argv);
    ros2_test();

    // Example control loop
    // int i = 0;
    // while (true) {
    //     // loop 10Hz
    //     if (i == 10) {
    //         send_message("STATUS", status);
    //         i = 0;
    //     }
    //     i++;
    // }
}

// Notes on subscribing to STATUS topics with different rates:
// // undocked
// unsubscribe_from_message("STATUS_10Hz");
// handle = subscribe_to_message("STATUS_1Hz", callback);

// // docked
// unsubscribe_from_message("STATUS_1Hz");
// subscribe_to_message("STATUS_10Hz", callback);

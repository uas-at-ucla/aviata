#include "lib/px4_io.hpp"
#include "lib/network.hpp"
#include "lib/drone.hpp"
#include <map>

std::map<std::string, DroneStatus> SWARM; // map by ID

int main(int argc, char** argv) {

    //takeoff_and_land_test(argc, argv);
    ros2_test();

    // Test Drone object
    Drone* test1 = new Drone("droneTest1", argv[1]);

    test1->arm_drone();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    test1->takeoff_drone();
    std::this_thread::sleep_for(std::chrono::seconds(9));
    test1->land_drone();
    //test1->disarm_drone(); //disarm when landed


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

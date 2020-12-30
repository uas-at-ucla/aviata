#include "lib/drone.hpp"

int main(int argc, char** argv) {
    Drone::init();

    Drone drone("speedy");
    drone.test_get_att_target("udp://:14540");

    // takeoff_and_land_test(argc, argv);
    // ros2_test();

    // // Test Drone object
    // Drone* test1 = new Drone("droneTest1", argv[1]);

    // test1->arm_drone();
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // test1->takeoff_drone();
    // std::this_thread::sleep_for(std::chrono::seconds(9));
    // test1->land_drone();
    // test1->disarm_drone(); //disarm when landed
}

// Notes on subscribing to STATUS topics with different rates:
// // undocked
// unsubscribe_from_message("STATUS_10Hz");
// handle = subscribe_to_message("STATUS_1Hz", callback);

// // docked
// unsubscribe_from_message("STATUS_1Hz");
// subscribe_to_message("STATUS_10Hz", callback);

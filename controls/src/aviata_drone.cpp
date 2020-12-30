#include <map>
#include <chrono>
#include "mavlink/v2.0/common/mavlink.h"
#include "lib/px4_io.hpp"
#include "lib/network.hpp"
#include "lib/drone.hpp"

using namespace std::chrono;

std::string drone_id = "speedy";
std::map<std::string, DroneStatus> SWARM; // map by ID

int main(int argc, char** argv) {
    PX4IO px4_io(drone_id);

    if (px4_io.connect_to_pixhawk("udp://:14540", 5) == false) {
        return 1;
    }

    px4_io.subscribe_attitude_target([](const mavlink_attitude_target_t& attitude_target) {
        std::cout << "thrust: " << attitude_target.thrust << std::endl;
    });

    px4_io.arm_system();

    bool began_descent = false;
    std::cout << "Taking off!" << std::endl;
    int64_t takeoff_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    px4_io.takeoff_system();
    
    while (true) {
        if (!began_descent && duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - takeoff_time >= 10000) {
            std::cout << "Landing!" << std::endl;
            px4_io.land_system();
            began_descent = true;
        }
        px4_io.call_queued_mavsdk_callbacks();
    }

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

#include "lib/drone.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: ./aviata_drone <drone_id> <program> [connection_url]" << std::endl;
        std::cout << "programs: lead, follow" << std::endl;
        return 1;
    }
    std::string connection_url = "udp://:14540";
    if (argc >= 4) {
        connection_url = argv[3];
    }

    Drone drone(argv[1]);
    if (strcmp(argv[2], "lead") == 0) {
        drone.test_lead_att_target(connection_url);
    } else if (strcmp(argv[2], "follow") == 0) {
        drone.test_follow_att_target(connection_url);
    } else {
        std::cout << "Unkown program: " << argv[2] << std::endl;
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

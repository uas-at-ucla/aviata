#include <iostream>
#include <thread>
#include <chrono>
#include <QApplication>
#include "lib/network.hpp"
#include "lib/dronestatus.hpp"
#include "lib_ground/main_window.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    // QApplication app(argc, argv);
    // MainWindow window;
    // window.show();
    // return app.exec();

    // Temporary minimal ground station functions
    if (argc < 2) {
        std::cout << "Please enter a command" << std::endl;
        return 1;
    }

    Network::init();

    if (strcmp(argv[1], "init") == 0) {
        // init <drone_id> <state> <docking_slot>
        if (argc < 5) {
            std::cout << "Not enough arguments" << std::endl;
            return 1;
        }

        DroneState initial_state;
        if (strcmp(argv[3], "follower") == 0) {
            initial_state = DOCKED_FOLLOWER;
        } else if (strcmp(argv[3], "leader") == 0) {
            initial_state = DOCKED_LEADER;
        } else if (strcmp(argv[3], "standby") == 0) {
            initial_state = STANDBY;
        } else {
            std::cout << "Invalid initial state: " << argv[3] << std::endl;
            return 1;
        }

        std::shared_ptr<Network> network = std::make_shared<Network>("ground_init");
        std::this_thread::sleep_for(5000ms); // wait to discover nodes

        network->init_drone_command_client_if_needed(argv[2]);

        bool got_ack = false;
        network->send_drone_command(argv[2], INIT_STATE, initial_state, atoi(argv[4]), "init state, ground", [&got_ack](uint8_t ack) {
            std::cout << "INIT_STATE ACK: " << (int)ack << std::endl;
            got_ack = true;
        });

        while (!got_ack) {
            Network::spin_some(network);
            network->check_command_requests();
        }
    } else if (strcmp(argv[1], "new_leader") == 0) {
        // init <drone_id> <docking_slot>
        if (argc < 4) {
            std::cout << "Not enough arguments" << std::endl;
            return 1;
        }

        std::shared_ptr<Network> network = std::make_shared<Network>("ground_new_leader");
        std::this_thread::sleep_for(5000ms); // wait to discover nodes

        network->init_drone_command_client_if_needed(argv[2]);

        bool got_ack = false;
        network->send_drone_command(argv[2], REQUEST_NEW_LEADER, atoi(argv[3]), -1, "request new leader, ground", [&got_ack](uint8_t ack) {
            std::cout << "REQUEST_NEW_LEADER ACK: " << (int)ack << std::endl;
            got_ack = true;
        });

        while (!got_ack) {
            Network::spin_some(network);
            network->check_command_requests();
        }
    } else if (strcmp(argv[1], "log") == 0) {
        std::shared_ptr<Network> network = std::make_shared<Network>("ground_log");
        network->subscribe<DRONE_DEBUG>([](const aviata::msg::DroneDebug::SharedPtr debug_msg) {
            std::cout << "[" << debug_msg->drone_id << "] " << debug_msg->debug << std::endl;
        });
        while (true) {
            Network::spin_some(network);
        }
    } else {
        std::cout << "Unrecognized command" << std::endl;
        return 1;
    }

    Network::shutdown();
}

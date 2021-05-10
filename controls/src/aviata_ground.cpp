#include <iostream>
#include <thread>
#include <chrono>
#include <QApplication>
#include "lib/network.hpp"
#include "lib_ground/main_window.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    CalculatorForm window;
    window.show();
    return app.exec();

    // // Sends a REQUEST_NEW_LEADER command.
    // if (argc < 2) {
    //     std::cout << "Please enter the leader's drone ID" << std::endl;
    //     return 1;
    // }

    // Network::init();
    // std::shared_ptr<Network> network = std::make_shared<Network>("ground");

    // std::this_thread::sleep_for(5000ms);

    // network->init_drone_command_client_if_needed(argv[1]);

    // bool got_ack = false;
    // network->send_drone_command(argv[1], REQUEST_NEW_LEADER, 0, "request new leader, ground", [&got_ack](uint8_t ack) {
    //     std::cout << "REQUEST_NEW_LEADER ACK: " << (int)ack << std::endl;
    //     got_ack = true;
    // });

    // while (!got_ack) {
    //     Network::spin_some(network);
    //     network->check_command_requests();
    // }

    // Network::shutdown();
}

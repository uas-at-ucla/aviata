#include <iostream>
#include "lib/network.hpp"

int main(int argc, char** argv) {
    // For now, just sends a REQUEST_NEW_LEADER command
    Network::init();
    std::shared_ptr<Network> network = std::make_shared<Network>("ground");
    network->init_drone_command_client_if_needed("leady");

    bool got_ack = false;
    network->send_drone_command("leady", REQUEST_NEW_LEADER, 0, "request new leader, ground", [&got_ack](uint8_t ack) {
        std::cout << "REQUEST_NEW_LEADER ACK: " << ack << std::endl;
        got_ack = true;
    });

    while (!got_ack) {
        Network::spin_some(network);
        network->check_command_requests();
    }    

    Network::shutdown();
}

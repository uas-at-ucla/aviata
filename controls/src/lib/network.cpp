#include "network.hpp"

void Network::init() {
    rclcpp::init(0, nullptr);
}

Network::Network() : Node("drone_id_goes_here") {

}

void Network::subscribe_to_status(std::function<void(aviata::msg::DroneStatus)> callback) {
    // call ROS2 functions to subscribe to the "status" ROS topic and register the callback
}

void Network::send_status(aviata::msg::DroneStatus status) {
    // status_publisher->publish(status);
}

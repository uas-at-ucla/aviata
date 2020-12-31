#include "network.hpp"

void Network::init() {
    rclcpp::init(0, nullptr);
}

void Network::spin_some(rclcpp::Node::SharedPtr node_ptr) {
    rclcpp::spin_some(node_ptr);
}

void Network::shutdown() {
    rclcpp::shutdown();
}

Network::Network(std::string drone_id) : Node(drone_id), drone_id(drone_id) {}

void Network::init_follower_setpoint_publisher() {
    follower_setpoint_publisher = this->create_publisher<aviata::msg::FollowerSetpoint>(FOLLOWER_SETPOINT, sensor_data_qos);
}

void Network::deinit_follower_setpoint_publisher() {
    follower_setpoint_publisher = nullptr;
}

void Network::publish_follower_setpoint(const aviata::msg::FollowerSetpoint& follower_setpoint) {
    if (follower_setpoint_publisher != nullptr) {
        follower_setpoint_publisher->publish(follower_setpoint);
    }
}

void Network::subscribe_follower_setpoint(std::function<void(const aviata::msg::FollowerSetpoint::SharedPtr)> callback) {
    follower_setpoint_subscription = this->create_subscription<aviata::msg::FollowerSetpoint>(FOLLOWER_SETPOINT, sensor_data_qos, callback);
}

void Network::unsubscribe_follower_setpoint() {
    follower_setpoint_subscription = nullptr;
}

void Network::subscribe_to_status(std::function<void(aviata::msg::DroneStatus)> callback) {
    // call ROS2 functions to subscribe to the "status" ROS topic and register the callback
}

void Network::send_status(aviata::msg::DroneStatus status) {
    // status_publisher->publish(status);
}

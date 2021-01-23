#include "network.hpp"

void Network::init()
{
    rclcpp::init(0, nullptr);
}

void Network::spin_some(rclcpp::Node::SharedPtr node_ptr)
{
    rclcpp::spin_some(node_ptr);
}

void Network::shutdown()
{
    rclcpp::shutdown();
}

Network::Network(std::string drone_id) : Node(drone_id), drone_id(drone_id) {}

// Follower Setpoint

void Network::init_follower_setpoint_publisher()
{
    follower_setpoint_publisher = this->create_publisher<aviata::msg::FollowerSetpoint>(FOLLOWER_SETPOINT, sensor_data_qos);
}

void Network::deinit_follower_setpoint_publisher()
{
    follower_setpoint_publisher = nullptr;
}

void Network::publish_follower_setpoint(const aviata::msg::FollowerSetpoint &follower_setpoint)
{
    if (follower_setpoint_publisher != nullptr)
    {
        follower_setpoint_publisher->publish(follower_setpoint);
    }
}

void Network::subscribe_follower_setpoint(std::function<void(const aviata::msg::FollowerSetpoint::SharedPtr)> callback)
{
    follower_setpoint_subscription = this->create_subscription<aviata::msg::FollowerSetpoint>(FOLLOWER_SETPOINT, sensor_data_qos, callback);
}

void Network::unsubscribe_follower_setpoint()
{
    follower_setpoint_subscription = nullptr;
}

// Drone Status

void Network::init_drone_status_publisher()
{
    drone_status_publisher = this->create_publisher<aviata::msg::DroneStatus>(DRONE_STATUS, sensor_data_qos);
}

void Network::deinit_drone_status_publisher()
{
    drone_status_publisher = nullptr;
}

void Network::publish_drone_status(const aviata::msg::DroneStatus &drone_status)
{
    if (drone_status_publisher != nullptr)
    {
        drone_status_publisher->publish(drone_status);
    }
}

void Network::subscribe_drone_status(std::function<void(aviata::msg::DroneStatus)> callback)
{
    drone_status_subscription = this->create_subscription<aviata::msg::DroneStatus>(DRONE_STATUS, sensor_data_qos, callback);
}

void Network::unsubscribe_drone_status()
{
    drone_status_subscription = nullptr;
}

// @brief to be deprecated?
void Network::send_status(aviata::msg::DroneStatus status)
{
    // status_publisher->publish(status);
}

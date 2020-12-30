#ifndef NETWORK_HPP
#define NETWORK_HPP

#include "drone.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

enum Message {
    REQUEST_SWAP,
    REQUEST_UNDOCK,
    REQUEST_DOCK,
    UNDOCK,
    DOCK,
    BECOME_LEADER,
    REQUEST_NEW_LEADER,
    LEADER_SETPOINT,
    FOLLOWER_SETPOINT,
    FRAME_ARM,
    FOLLOWER_ARM,
    FRAME_DISARM,
    FOLLOWER_DISARM,
    FRAME_TAKEOFF,
    FRAME_LAND,
    CANCEL_DOCKING,
    TERMINATE_FLIGHT,
    INITIALIZE_STATE,
    DRONE_STATUS
};

class Network: public rclcpp::Node
{
public:
    Network();
    
    void subscribe_to_status(void (*callback)(DroneStatus));

    void subscribe_to_dock_command();

    void subscribe_to_undock_command();

    void send_status(DroneStatus status);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher = this->create_publisher<std_msgs::msg::String>("STATUS", 10);
};

#endif

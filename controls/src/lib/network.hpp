#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aviata/msg/drone_status.hpp"
#include "aviata/msg/follower_setpoint.hpp"

// TODO define all message types as string constants (to use as ROS topics)

// Inputs to Ground Station
// REQUEST_SWAP
// REQUEST_UNDOCK
// REQUEST_DOCK
// TERMINATE_FLIGHT

// From Ground Station to a Drone
// UNDOCK
// DOCK
// CANCEL_DOCKING

// From Ground Station to Leader Drone
// LEADER_SETPOINT
// FRAME_ARM
// FRAME_DISARM
// FRAME_TAKEOFF
// FRAME_LAND

// INITIALIZE_STATE

// Drone to Drone
// BECOME_LEADER
// REQUEST_NEW_LEADER
#define FOLLOWER_SETPOINT "FOLLOWER_SETPOINT"
// FOLLOWER_ARM
// FOLLOWER_DISARM

// DRONE_STATUS

class Network: public rclcpp::Node
{
public:
    static void init();
    static void spin_some(rclcpp::Node::SharedPtr node_ptr);
    static void shutdown();

    Network(std::string drone_id);
    
    void init_follower_setpoint_publisher();
    void deinit_follower_setpoint_publisher();
    void publish_follower_setpoint(const aviata::msg::FollowerSetpoint& follower_setpoint);

    void subscribe_follower_setpoint(std::function<void(const aviata::msg::FollowerSetpoint::SharedPtr)> callback);
    void unsubscribe_follower_setpoint();

    void subscribe_to_status(std::function<void(aviata::msg::DroneStatus)> callback);

    void subscribe_to_dock_command();

    void subscribe_to_undock_command();

    void send_status(aviata::msg::DroneStatus status);

private:
    const std::string drone_id;

    const rclcpp::QoS sensor_data_qos{rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data};

    rclcpp::Publisher<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_publisher;
    rclcpp::Subscription<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_subscription;
};

#endif

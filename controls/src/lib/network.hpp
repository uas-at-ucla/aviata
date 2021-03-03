#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <functional>
#include <map>
#include <ctime>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aviata/msg/drone_status.hpp"
#include "aviata/msg/follower_setpoint.hpp"
#include "aviata/msg/drone_debug.hpp"

#include "aviata/srv/drone_command.hpp"

// define all command message types as enum constants
enum DroneCommand
{
    // Inputs to Ground Station
    REQUEST_SWAP,
    REQUEST_UNDOCK,
    REQUEST_DOCK,
    TERMINATE_FLIGHT,
    // From Ground Station to a Drone
    UNDOCK,
    DOCK,
    CANCEL_DOCKING,
    SETPOINT,
    LEADER_SETPOINT,
    // Anywhere to Drone, have different response if leader or follower
    BECOME_LEADER,
    REQUEST_NEW_LEADER,
    LISTEN_NEW_LEADER,
    ARM,
    DISARM,
    TAKEOFF,
    LAND
};

// struct to hold data about each drone_command request (ROS2 Service)
// objects stored in drone_command_requests and drone_command_responses in drone.hpp
struct CommandRequest {
    // Request
    std::string other_drone_id;
    DroneCommand drone_command;
    int param;

    // Response
    std::shared_future<std::shared_ptr<aviata::srv::DroneCommand::Response>> command_request;
    uint8_t ack; // store the response once received
    std::shared_ptr<std::function<void(uint8_t ack)>> callback;

    // logging/debugging purposes
    std::string request_origin;
    std::chrono::high_resolution_clock::time_point timestamp_request; // timestamp_request = std::chrono::high_resolution_clock::now()
    std::chrono::high_resolution_clock::time_point timestamp_response;
};

// TOPICS
#define FOLLOWER_SETPOINT "FOLLOWER_SETPOINT"
#define DRONE_STATUS "DRONE_STATUS"
#define DRONE_DEBUG "DRONE_DEBUG"

class Network : public rclcpp::Node
{
public:
    static void init();
    static void spin_some(rclcpp::Node::SharedPtr node_ptr);
    static void shutdown();

    Network(std::string drone_id);

    void init_follower_setpoint_publisher();
    void deinit_follower_setpoint_publisher();
    void publish_follower_setpoint(const aviata::msg::FollowerSetpoint &follower_setpoint);
    void subscribe_follower_setpoint(std::function<void(const aviata::msg::FollowerSetpoint::SharedPtr)> callback);
    void unsubscribe_follower_setpoint();

    void init_drone_status_publisher();
    void deinit_drone_status_publisher();
    void publish_drone_status(const aviata::msg::DroneStatus &drone_status);
    void subscribe_drone_status(std::function<void(aviata::msg::DroneStatus::SharedPtr)> callback);
    void unsubscribe_drone_status();

    void init_drone_debug_publisher();
    void deinit_drone_debug_publisher();
    void publish_drone_debug(const aviata::msg::DroneDebug &drone_debug);
    // subscribe functions only on groundstation?

    // https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#test-the-new-interfaces
    void init_drone_command_service(std::function<void(const aviata::srv::DroneCommand::Request::SharedPtr,
                                                       aviata::srv::DroneCommand::Response::SharedPtr)> callback);
    void deinit_drone_command_service();

    void init_drone_command_client(std::string other_drone_id);
    void deinit_drone_command_client(std::string other_drone_id);

    std::shared_future<std::shared_ptr<aviata::srv::DroneCommand::Response>> 
        send_drone_command_async(std::string other_drone_id, DroneCommand drone_command, int param = -1);

private:
    const std::string drone_id;

    const rclcpp::QoS sensor_data_qos{rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data};

    rclcpp::Publisher<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_publisher;
    rclcpp::Subscription<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_subscription;

    rclcpp::Publisher<aviata::msg::DroneStatus>::SharedPtr drone_status_publisher;
    rclcpp::Subscription<aviata::msg::DroneStatus>::SharedPtr drone_status_subscription;
    
    rclcpp::Publisher<aviata::msg::DroneDebug>::SharedPtr drone_debug_publisher;

    rclcpp::Service<aviata::srv::DroneCommand>::SharedPtr drone_command_service;
    std::map<std::string, rclcpp::Client<aviata::srv::DroneCommand>::SharedPtr> drone_command_clients;
};

#endif

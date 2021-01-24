#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <functional>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aviata/msg/drone_status.hpp"
#include "aviata/msg/follower_setpoint.hpp"

// define all message types as string constants (to use as ROS topics)

// Inputs to Ground Station
#define REQUEST_SWAP "REQUEST_SWAP"
#define REQUEST_UNDOCK "REQUEST_UNDOCK"
#define REQUEST_DOCK "REQUEST_DOCK"
#define TERMINATE_FLIGHT "TERMINATE_FLIGHT"

// From Ground Station to a Drone
#define UNDOCK "UNDOCK"
#define DOCK "DOCK"
#define CANCEL_DOCKING "CANCEL_DOCKING"
// Manual control if necessary
// #define DRONE_SETPOINT "DRONE_SETPOINT"
// #define DRONE_ARM "DRONE_ARM"
// #define DRONE_DISARM "DRONE_DISARM"
// #define DRONE_TAKEOFF "DRONE_TAKEOFF"
// #define DRONE_LAND "DRONE_LAND"

// From Ground Station to Leader Drone
#define LEADER_SETPOINT "LEADER_SETPOINT"
#define FRAME_ARM "FRAME_ARM"
#define FRAME_DISARM "FRAME_DISARM"
#define FRAME_TAKEOFF "FRAME_TAKEOFF"
#define FRAME_LAND "FRAME_LAND"

// INITIALIZE_STATE

// Drone to Drone
#define BECOME_LEADER "BECOME_LEADER"
#define REQUEST_NEW_LEADER "REQUEST_NEW_LEADER"
#define FOLLOWER_ARM "FOLLOWER_ARM"
#define FOLLOWER_DISARM "FOLLOWER_DISARM"

#define FOLLOWER_SETPOINT "FOLLOWER_SETPOINT"

// DRONE_STATUS

#define DRONE_STATUS "DRONE_STATUS"

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

    void init_drone_status_publisher();
    void deinit_drone_status_publisher();
    void publish_drone_status(const aviata::msg::DroneStatus& drone_status);
    void subscribe_drone_status(std::function<void(aviata::msg::DroneStatus::SharedPtr)> callback);
    void unsubscribe_drone_status();
    void send_status(aviata::msg::DroneStatus status); //old

    // void subscribe_to_dock_command();
    // void subscribe_to_undock_command();

// https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#test-the-new-interfaces
    void init_drone_command_service(const std::string service_name = drone_id + "_SERVICE", 
                                    std::function<void(aviata::srv::DroneCommand::Request::SharedPtr, 
                                                       aviata::srv::DroneCommand::Response::SharedPtr)> callback);
    void deinit_drone_command_service();    

    void init_drone_command_client(std::string service_name);
    void deinit_drone_command_client(std::string service_name);
    uint8_t send_drone_command_async(std::string service_name, const aviata::srv::DroneCommand::Request &request);

private:
    const std::string drone_id;

    const rclcpp::QoS sensor_data_qos{rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data};

    rclcpp::Publisher<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_publisher;
    rclcpp::Subscription<aviata::msg::FollowerSetpoint>::SharedPtr follower_setpoint_subscription;

    rclcpp::Publisher<aviata::msg::DroneStatus>::SharedPtr drone_status_publisher;
    rclcpp::Subscription<aviata::msg::DroneStatus>::SharedPtr drone_status_subscription;

    rclcpp::Service<aviata::srv::DroneCommand>::SharedPtr drone_command_service;
    std::map<std::string, rclcpp::Client<aviata::srv::DroneCommand>::SharedPtr> drone_command_clients;
};

#endif

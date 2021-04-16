#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <functional>
#include <map>
#include <ctime>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "topics.hpp"

// Commands that the drone/ground station can receive or send to an individual
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
    REQUEST_NEW_LEADER,
    // From Drone to Drone
    BECOME_LEADER,
    // LISTEN_NEW_LEADER,
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

class Network : public rclcpp::Node
{
public:
    static void init();
    static void spin_some(rclcpp::Node::SharedPtr node_ptr);
    static void shutdown();

    Network(std::string drone_id);

    // generic publish/subscribe functions
    template<const std::string& ros_topic>
    void init_publisher()
    {
        std::get<PubSub<ros_topic>>(pubsubs).publisher = this->create_publisher<typename RosTopicConfig<ros_topic>::msg_type>(ros_topic, RosTopicConfig<ros_topic>::qos::value);
    }

    template<const std::string& ros_topic>
    void deinit_publisher()
    {
        std::get<PubSub<ros_topic>>(pubsubs).publisher = nullptr;
    }

    template<const std::string& ros_topic>
    void publish(const typename RosTopicConfig<ros_topic>::msg_type& msg)
    {
        if (std::get<PubSub<ros_topic>>(pubsubs).publisher != nullptr)
        {
            std::get<PubSub<ros_topic>>(pubsubs).publisher->publish(msg);
        }
    }

    template<const std::string& ros_topic>
    void subscribe(std::function<void(const typename RosTopicConfig<ros_topic>::msg_type::SharedPtr)> callback)
    {
        std::get<PubSub<ros_topic>>(pubsubs).subscription = this->create_subscription<typename RosTopicConfig<ros_topic>::msg_type>(ros_topic, RosTopicConfig<ros_topic>::qos::value, callback);
    }

    template<const std::string& ros_topic>
    void unsubscribe()
    {
        std::get<PubSub<ros_topic>>(pubsubs).subscription = nullptr;
    }

    void publish_drone_debug(const std::string & debug_msg, uint8_t code = 0);


    void init_drone_command_service(std::function<void(const aviata::srv::DroneCommand::Request::SharedPtr,
                                                       aviata::srv::DroneCommand::Response::SharedPtr)> callback); // https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#test-the-new-interfaces
    void deinit_drone_command_service();

    void init_drone_command_client_if_needed(std::string other_drone_id);
    void deinit_drone_command_client(std::string other_drone_id);

    std::shared_future<std::shared_ptr<aviata::srv::DroneCommand::Response>> 
        send_drone_command_async(std::string other_drone_id, DroneCommand drone_command, int param = -1);

    void send_drone_command(std::string other_drone_id, DroneCommand drone_command, int8_t param, std::string request_origin,
                            std::function<void(uint8_t ack)> callback);

    void check_command_requests();

private:
    const std::string drone_id;

    std::tuple<PubSub<DRONE_STATUS>, 
               PubSub<DRONE_DEBUG>,
               PubSub<FRAME_ARM>,
               PubSub<FRAME_DISARM>,
               PubSub<FRAME_TAKEOFF>,
               PubSub<FRAME_LAND>,
               PubSub<FRAME_SETPOINT>,
               PubSub<FOLLOWER_ARM>,
               PubSub<FOLLOWER_DISARM>,
               PubSub<FOLLOWER_SETPOINT>
    > pubsubs;

    rclcpp::Service<aviata::srv::DroneCommand>::SharedPtr drone_command_service;
    std::map<std::string, rclcpp::Client<aviata::srv::DroneCommand>::SharedPtr> drone_command_clients;

    // Command Request Lists
    std::vector<CommandRequest> drone_command_requests;
    std::vector<CommandRequest> drone_command_responses; // TODO: log to file at end of flight?
};

#endif

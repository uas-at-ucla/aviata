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

// ROS topics that the drone/ground station can publish and subscribe to:
// Broadcast from any Drone to anyone who subscribes
const std::string DRONE_STATUS = "DRONE_STATUS";
const std::string DRONE_DEBUG = "DRONE_DEBUG";
// Broadcast from Ground Station, only the Leader subscribes
const std::string FRAME_ARM = "FRAME_ARM";
const std::string FRAME_DISARM = "FRAME_DISARM";
const std::string FRAME_TAKEOFF = "FRAME_TAKEOFF";
const std::string FRAME_LAND = "FRAME_LAND";
const std::string FRAME_SETPOINT = "FRAME_SETPOINT";
// Broadcast from Leader, only Followers subscribe
const std::string FOLLOWER_ARM = "FOLLOWER_ARM";
const std::string FOLLOWER_DISARM = "FOLLOWER_DISARM";
const std::string FOLLOWER_SETPOINT = "FOLLOWER_SETPOINT";

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

// Qualities-of-service for ROS messages over the network (https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
const rclcpp::QoS sensor_data_qos{rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data}; // lossy, for high-rate data streams
const rclcpp::QoS services_default_qos{rclcpp::QoSInitialization(rmw_qos_profile_services_default.history, rmw_qos_profile_services_default.depth), rmw_qos_profile_services_default}; // reliable, for occasional messages that must be delivered

// ROS topic config
template <const std::string& ros_topic>
struct RosTopicConfig{  };

template<> struct RosTopicConfig<DRONE_STATUS> {
    typedef aviata::msg::DroneStatus msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<DRONE_DEBUG> {
    typedef aviata::msg::DroneDebug msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FOLLOWER_SETPOINT> {
    typedef aviata::msg::FollowerSetpoint msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, sensor_data_qos> qos;
};

template<const std::string& ros_topic>
struct PubSub {
    typename rclcpp::Publisher<typename RosTopicConfig<ros_topic>::msg_type>::SharedPtr publisher;
    typename rclcpp::Subscription<typename RosTopicConfig<ros_topic>::msg_type>::SharedPtr subscription;
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

    void publish_drone_debug(const std::string & debug_msg);

    // https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#test-the-new-interfaces
    void init_drone_command_service(std::function<void(const aviata::srv::DroneCommand::Request::SharedPtr,
                                                       aviata::srv::DroneCommand::Response::SharedPtr)> callback);
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

    std::tuple<PubSub<DRONE_STATUS>, PubSub<DRONE_DEBUG>, PubSub<FOLLOWER_SETPOINT>> pubsubs;

    rclcpp::Service<aviata::srv::DroneCommand>::SharedPtr drone_command_service;
    std::map<std::string, rclcpp::Client<aviata::srv::DroneCommand>::SharedPtr> drone_command_clients;

    // Command Request Lists
    std::vector<CommandRequest> drone_command_requests;
    std::vector<CommandRequest> drone_command_responses; // TODO: log to file at end of flight?
};

#endif

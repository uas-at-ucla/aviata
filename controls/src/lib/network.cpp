#include "network.hpp"

// static functions
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


// constructor
Network::Network(std::string drone_id) : Node(drone_id), drone_id(drone_id) {}


// DRONE DEBUG
void Network::publish_drone_debug(const std::string & debug_msg)
{
    if (std::get<PubSub<DRONE_DEBUG>>(pubsubs).publisher == nullptr)
    {
        init_publisher<DRONE_DEBUG>();
    }
    aviata::msg::DroneDebug drone_debug;
    drone_debug.debug = debug_msg;
    drone_debug.drone_id = this->get_name(); // same as drone_id
    publish<DRONE_DEBUG>(drone_debug);
}


// DRONE COMMAND SERVICE

// @brief uses default qos profile (rmw_qos_profile_services_default)
// @param callback the command handler
void Network::init_drone_command_service(std::function<void(aviata::srv::DroneCommand::Request::SharedPtr,
                                                            aviata::srv::DroneCommand::Response::SharedPtr)> callback)
{
    std::string service_name = drone_id + "_SERVICE";
    drone_command_service = this->create_service<aviata::srv::DroneCommand>(service_name, callback);
}

void Network::deinit_drone_command_service()
{
    drone_command_service = nullptr;
}

// DRONE COMMAND CLIENT

void Network::init_drone_command_client(std::string other_drone_id)
{
    std::string service_name = other_drone_id + "_SERVICE";
    if (drone_command_clients.find(service_name) == drone_command_clients.end())
        drone_command_clients[service_name] = this->create_client<aviata::srv::DroneCommand>(service_name);
}

void Network::deinit_drone_command_client(std::string other_drone_id)
{
    std::string service_name = other_drone_id + "_SERVICE";
    auto it = drone_command_clients.find(other_drone_id);
    drone_command_clients.erase(it);
}

// @brief async command request
// @return shared_future object to response of service (check if valid with response.valid() (std::shared_future object))
// https://en.cppreference.com/w/cpp/thread/shared_future
std::shared_future<std::shared_ptr<aviata::srv::DroneCommand::Response>> 
    Network::send_drone_command_async(std::string other_drone_id, DroneCommand drone_command, int param)
{
    std::string service_name = other_drone_id + "_SERVICE";

    if (!drone_command_clients[service_name]->service_is_ready())
    {
        if (!rclcpp::ok())
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), other_drone_id + " service not available");
        return std::shared_future<std::shared_ptr<aviata::srv::DroneCommand::Response>>(); //returns empty (invalid) shared_future object
    }

    auto request = std::make_shared<aviata::srv::DroneCommand::Request>(); 
    request->command = drone_command;
    request->param = param;

    return drone_command_clients[service_name]->async_send_request(request);
}

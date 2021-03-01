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

// FOLLOWER SETPOINT

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

// DRONE STATUS

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

void Network::subscribe_drone_status(std::function<void(aviata::msg::DroneStatus::SharedPtr)> callback)
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

// DRONE COMMAND SERVICE

// @brief uses default qos profile (rmw_qos_profile_services_default)
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
    drone_command_clients[service_name] = this->create_client<aviata::srv::DroneCommand>(service_name);
}

void Network::deinit_drone_command_client(std::string other_drone_id)
{
    std::string service_name = other_drone_id + "_SERVICE";
    drone_command_clients[service_name] = nullptr;
}

// @brief function is blocking- TODO: make async 
// @return acknowledgement received through the ROS2 service
uint8_t Network::send_drone_command(std::string other_drone_id, DroneCommand drone_command, int param)
{
    std::string service_name = other_drone_id + "_SERVICE";

    if (!drone_command_clients[service_name]->service_is_ready())
    {
        if (!rclcpp::ok())
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), other_drone_id + " service not available");
        return 0;
    }

    auto request = std::make_shared<aviata::srv::DroneCommand::Request>(); 
    request->command = drone_command;
    request->param = param;

    auto result = drone_command_clients[service_name]->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ack;
    }
    return 0;
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



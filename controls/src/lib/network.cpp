#define USE_ROS // Comment to disable ROS2 code

#include "network.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

#ifdef USE_ROS

// ROS2 example code
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

void ros2_test() {
    std::cout << "Starting ROS2 test" << std::endl;
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
}

#else
// Dummy code when testing w/o ROS
void ros2_test() {}
#endif

Network::Network() : Node("drone_id_goes_here") {

}

void Network::subscribe_to_status(void (*callback)(DroneStatus)) { // TODO double check correct function pointer format
    // call ROS2 functions to subscribe to the "status" ROS topic
    // ROS callback:
    // {
    //     DroneStatus status = // convert ROS msg to DroneStatus;
    //     callback();
    // }
}

void Network::send_status(DroneStatus status) {
    // convert status to a ROS message
    // status_publisher->publish(message);
}
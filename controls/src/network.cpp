// #define USE_ROS // Uncomment to enable ROS2 code

#include "network.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

#ifdef USE_ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

//TODO
void send_message() {

}

//TODO
void subscribe_to_message() {

}


#else
// Dummy code when testing w/o ROS

void ros2_test() {}

void send_message() {

}

void subscribe_to_message() {
    
}

#endif

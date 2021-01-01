#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aviata/msg/drone_status.hpp"

using namespace std::chrono_literals;

// ROS2 example code
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<aviata::msg::DroneStatus>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    size_t get_count() {
        return count_;
    }

private:
    void timer_callback()
    {
        auto message = aviata::msg::DroneStatus();
        message.drone_id = "Drone ID: " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.drone_id.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<aviata::msg::DroneStatus>::SharedPtr publisher_;
    size_t count_;
};

void ros2_test() {
    std::cout << "Starting ROS2 test" << std::endl;
    rclcpp::init(0, nullptr);
    std::shared_ptr<MinimalPublisher> node = std::make_shared<MinimalPublisher>();
    while (node->get_count() < 10) {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    ros2_test();
}

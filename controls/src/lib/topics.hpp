#ifndef TOPICS_HPP
#define TOPICS_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "aviata/msg/drone_status.hpp"
#include "aviata/msg/drone_debug.hpp"
#include "aviata/msg/empty.hpp"
#include "aviata/msg/follower_setpoint.hpp"
#include "aviata/msg/frame_setpoint.hpp"
#include "aviata/msg/docking_info.hpp"
#include "aviata/msg/reference_attitude.hpp"

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
const std::string REFERENCE_ATTITUDE = "REFERENCE_ATTITUDE";
// Broadcast from any drone that is arriving at/departing from the frame
const std::string DOCKING_INFO = "DOCKING_INFO";

// Qualities-of-service for ROS messages over the network (https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
const rclcpp::QoS sensor_data_qos{rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data}; // lossy, for high-rate data streams
const rclcpp::QoS services_default_qos{rclcpp::QoSInitialization(rmw_qos_profile_services_default.history, rmw_qos_profile_services_default.depth), rmw_qos_profile_services_default}; // reliable, for occasional messages that must be delivered

// ROS topic config
template <const std::string& ros_topic>
struct RosTopicConfig{  };

// DRONE
template<> struct RosTopicConfig<DRONE_STATUS> {
    typedef aviata::msg::DroneStatus msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<DRONE_DEBUG> {
    typedef aviata::msg::DroneDebug msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};

// FRAME
template<> struct RosTopicConfig<FRAME_ARM> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FRAME_DISARM> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FRAME_TAKEOFF> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FRAME_LAND> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FRAME_SETPOINT> {
    typedef aviata::msg::FrameSetpoint msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, sensor_data_qos> qos;
};

template<> struct RosTopicConfig<DOCKING_INFO> {
    typedef aviata::msg::DockingInfo msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};

// FOLLOWER
template<> struct RosTopicConfig<FOLLOWER_ARM> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FOLLOWER_DISARM> {
    typedef aviata::msg::Empty msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, services_default_qos> qos;
};
template<> struct RosTopicConfig<FOLLOWER_SETPOINT> {
    typedef aviata::msg::FollowerSetpoint msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, sensor_data_qos> qos;
};
template<> struct RosTopicConfig<REFERENCE_ATTITUDE> {
    typedef aviata::msg::ReferenceAttitude msg_type;
    typedef std::integral_constant<const rclcpp::QoS&, sensor_data_qos> qos;
};

template<const std::string& ros_topic>
struct PubSub {
    typename rclcpp::Publisher<typename RosTopicConfig<ros_topic>::msg_type>::SharedPtr publisher;
    typename rclcpp::Subscription<typename RosTopicConfig<ros_topic>::msg_type>::SharedPtr subscription;
};

// Timer Topics
enum TimerTopic {
    TIMER_DRONE_STATUS,
    TIMER_ATT_REFERENCE,
    NUM_TIMERS
};

#endif

#include "network.hpp"
#include "aviata/msg/drone_status.hpp"

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

#include "drone.hpp"
#include "camera_simulator.hpp" // temporary

int main(/*int argc, char** argv */)
{
    Target t;
    t.yaw = 180;

    Drone drone(t);
    drone.connect_gazebo();
    // drone.arm();
    // drone.takeoff();
    // drone.initiate_docking(3);
    drone.test2();

    return 0;
}
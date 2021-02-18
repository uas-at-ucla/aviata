#include "drone.hpp"

int main(/*int argc, char** argv */)
{
    Target t;
    // t.lat = 2;
    //t.yaw = 180;

    Drone drone(t);
    drone.connect_gazebo();
    // drone.test1();
    drone.arm();
    drone.takeoff(3);
    drone.initiate_docking(2);
    //drone.test2();

    // drone.simulation_test_moving_target();
    return 0;
}
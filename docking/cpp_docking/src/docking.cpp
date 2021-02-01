#include "drone.hpp"
#include "camera_simulator.hpp" // temporary

int main(/*int argc, char** argv */)
{
    Target t;

    Drone drone(t);
    drone.connect_gazebo();
    drone.takeoff();
    drone.initiate_docking(1);
    
    return 0;
}
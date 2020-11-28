#include "px4_io.hpp"
#include "network.hpp"
#include "drone.hpp"

//enum DroneState now in drone.hpp

drone SWARM[10]; // eventually switch to dynamic size?

int main(int argc, char** argv) {
    takeoff_and_land_test(argc, argv);
    ros2_test();
}

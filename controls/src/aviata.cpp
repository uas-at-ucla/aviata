#include "px4_io.hpp"
#include "network.hpp"
#include "drone.hpp"

//enum DroneState now in drone.hpp

drone SWARM[8];

int main(int argc, char** argv) {
    takeoff_and_land_test(argc, argv);
    ros2_test();
}

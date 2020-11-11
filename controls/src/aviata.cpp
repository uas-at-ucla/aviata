#include "px4_io.hpp"
#include "network.hpp"

enum DroneState {
    STANDBY,
    ARRIVING,
    DOCKING,
    DOCKED_FOLLOWER,
    DOCKED_LEADER,
    UNDOCKING,
    DEPARTING,
    NEEDS_SERVICE
};

int main(int argc, char** argv) {
    takeoff_and_land_test(argc, argv);
    ros2_test();
}

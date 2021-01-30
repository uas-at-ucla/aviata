#include "drone.hpp"
#include "raspi_camera.hpp" // temporary

int main(/*int argc, char** argv */)
{
    Target t;

    // Drone drone(t);
    // drone.connect_gazebo();
    // drone.takeoff();
    // drone.initiate_docking(1);
    // while (true) {
    //     CameraSimulator cs(t);
    //     cs.update_current_image(1, 1, 1, 0, 0);
    //     sleep_for(seconds(1));
    // }
    RaspiCamera camera;

    return 0;
}
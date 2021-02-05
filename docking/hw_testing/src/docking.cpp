#include "drone.hpp"
#include "raspi_camera.hpp" // temporary
#include "util.hpp"

#include <future>
using namespace std::chrono;
using std::this_thread::sleep_for;

int main(/*int argc, char** argv */)
{
    Target t;

    Drone drone(t);
    drone.connect_gazebo();
    // drone.arm();
    drone.test1();

    // RaspiCamera camera;
    // sleep_for(seconds(2));
    // high_resolution_clock::time_point t1 = high_resolution_clock::now();
    // for (int i = 0; i < 100; i++)
    // {
    //     cv::Mat img = camera.update_current_image();

    //     // cv::imshow("Test", img);
    //     // cv::waitKey(1);
    // }
    // high_resolution_clock::time_point t2 = high_resolution_clock::now();
    // duration<double, std::milli> d = (t2 - t1);
    // int time_span = d.count();
    // log("time for 100 frames:", std::to_string(time_span));

    return 0;
}


// Streaming video feed through local TCP connection:

// cv::VideoCapture _cv;
// if(_cv.open("tcp://localhost:8080")) { // change MACRO to the actual values!
//     log("Camera", "Working");
//     for(;;) {
//         high_resolution_clock::time_point t1 = high_resolution_clock::now();
//         cv::Mat img;
//         _cv.read(img);
//         if (img.empty()) {
//             log("Camera", "ERROR! blank frame grabbed", true);
//         }
//         cv::imshow("Test", img);
//         cv::waitKey(1);
//         high_resolution_clock::time_point t2 = high_resolution_clock::now();
//         duration<double, std::milli> d = (t2 - t1);
//         int time_span = d.count();
//         log("Timer", std::to_string(time_span));
//     }
// } else {
//     log("Camera", "Cannot open source");
// }
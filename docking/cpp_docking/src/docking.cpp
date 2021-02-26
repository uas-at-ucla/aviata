#include "drone.hpp"

void apriltag_test();

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
    drone.land();
    // drone.initiate_docking(2);
    //drone.test2();

    // drone.simulation_test_moving_target();
    // apriltag_test();

    return 0;
}


#include "image_analyzer.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include "util.hpp"
namespace fs = std::filesystem;

void apriltag_test() {
    ImageAnalyzer *ia = new ImageAnalyzer();
    std::string path = "/home/axel/Desktop/UAS/testing/02-24-2021-offboard-manual/7-images";
    for (const auto &entry : fs::directory_iterator(path)) {
        std::string img_name = entry.path();
        std::string test = " ";
        cv::Mat img = cv::imread(img_name);
        
        Errors errs;
        bool found = ia->processImage(img, 0, 0, test, errs);
        if (found) {
            log("test", "found: " + img_name);
        } else {
            log("test", "not found");
        }
    }
}
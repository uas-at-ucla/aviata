#include "drone.hpp"

void apriltag_test();

int main(/*int argc, char** argv */)
{
    Target t;
    // t.yaw = 180;
    
    Drone drone(t);
    drone.connect_gazebo();
    // drone.arm();
    // drone.takeoff(4);
    // drone.initiate_docking(1);
    // apriltag_test();
    drone.test1();
}


// #include "image_analyzer.hpp"
// #include <opencv2/core.hpp>
// #include <opencv2/core/types.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <filesystem>
// #include "util.hpp"
// #include <vector>
// namespace fs = std::filesystem;

// void apriltag_test() {
//     ImageAnalyzer *ia = new ImageAnalyzer();
//     std::string path = "/home/axel/Desktop/UAS/testing/03-26-2021-pid-tuning/2-images";
//     int dets = 0;
//     int nondets = 0;

//     std::vector<std::string> filenames;
//     for (const auto &entry : fs::directory_iterator(path)) {
//         filenames.push_back(entry.path());
//     }
//     std::sort(filenames.begin(), filenames.end(), [](const std::string& a, const std::string& b) { return a < b; });

//     std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
//     for (const auto img_name : filenames) {
//         std::string test = " ";
//         cv::Mat img = cv::imread(img_name);
        
//         Errors errs;
//         bool found = ia->processImage(img, 0, 0, test, errs);
//         if (found) {
//             log("test", "found: " + img_name + " ---------- " + std::to_string(errs.x) + " " + std::to_string(errs.y) + " " + std::to_string(errs.alt) + " " + std::to_string(errs.yaw));
//             dets++;
//         } else {
//             log("test", "not found: " + img_name, true);
//             nondets++;
//         }
//     }
//     std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> d = (t2 - t1);
//     int time_span = d.count();
//     log("Docking", "fps: " + std::to_string(((dets + nondets) * 1000) / time_span));
//     log("Docking", "time: " + std::to_string(time_span));

//     log("Totals", "Found in " + std::to_string(dets) + " of " + std::to_string(dets + nondets));
// }
#include <iostream>
#include <future>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "util.hpp"
#include "raspi_camera.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;
using std::this_thread::sleep_for;

RaspiCamera::RaspiCamera(Target target)
{
    UNUSED(target); // unused but kept to match CameraSimulator constructor
    int deviceID = 0; // 0 = open default camera
    camera.open(deviceID, cv::CAP_V4L2);
    log("width", std::to_string(camera.set(cv::CAP_PROP_FRAME_WIDTH, 640 * 1))); // more resolution doesn't hurt performance too much and pic is SO much better
    log("height", std::to_string(camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480 * 1)));
    // log("fps", std::to_string(camera.set(cv::CAP_PROP_FPS, 60)));
    // log("zoom", std::to_string(camera.set(cv::CAP_PROP_ZOOM, 1)));

    // check if we succeeded
    if (!camera.isOpened())
    {
        log("Camera", "ERROR! Unable to open camera", true);
    }
    else
    {
        log("Camera", "Camera initialization successful");
        log("Camera", "Recording at " + std::to_string(camera.get(cv::CAP_PROP_FPS)) +
                          ", " + std::to_string(camera.get(cv::CAP_PROP_FRAME_WIDTH)) +
                          "x" + std::to_string(camera.get(cv::CAP_PROP_FRAME_HEIGHT)));
    }
}

Mat RaspiCamera::update_current_image(float unused1, float unused2, float unused3, float unused4, int unused5)
{
    // unused variables, kept to match CameraSimulator's API
    UNUSED(unused1);
    UNUSED(unused2);
    UNUSED(unused3);
    UNUSED(unused3);
    UNUSED(unused4);
    UNUSED(unused5);

    Mat frame;
    camera.read(frame);

    // check if we succeeded
    if (frame.empty())
    {
        cerr << "ERROR! blank frame grabbed\n";
        log("Camera", "ERROR! blank frame grabbed", true);
    }

    return frame;
}
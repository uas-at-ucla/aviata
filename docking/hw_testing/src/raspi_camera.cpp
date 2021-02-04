#include <iostream>
#include <future>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <thread>

#include "util.hpp"
#include "raspi_camera.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;
using std::this_thread::sleep_for;

bool running = false;

void thread_routine(VideoCapture& camera, Mat& frame) {
    while (running) {
        camera.read(frame);

        // check if we succeeded
        if (frame.empty())
        {
            cerr << "ERROR! blank frame grabbed\n";
            log("Camera", "ERROR! blank frame grabbed", true);
        }
    }
}

RaspiCamera::RaspiCamera()
{
    int deviceID = 0;        // 0 = open default camera
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

void RaspiCamera::start() {
    running = true;
    sleep_for(seconds(2));
    m_thread = new std::thread(thread_routine, std::ref(camera), std::ref(m_latest_frame));
}

void RaspiCamera::end() {
    running = false;
    m_thread->join();
    delete m_thread;
}

Mat RaspiCamera::update_current_image()
{
    while (m_latest_frame.empty()) {
        //
    }
    return m_latest_frame;
}
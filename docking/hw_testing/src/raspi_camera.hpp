#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "util.hpp"
#include <raspicam/raspicam_cv.h>
#include <opencv2/core.hpp>
#include <thread>

class RaspiCamera
{

public:
    RaspiCamera();
    cv::Mat update_current_image();
    void start();
    void end();

private:
    // raspicam::RaspiCam_Cv camera;
    cv::VideoCapture camera;
    cv::Mat m_latest_frame;
    std::thread* m_thread;
};

#endif // CAMERA_SIMULATOR_H_
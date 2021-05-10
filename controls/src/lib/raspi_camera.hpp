#ifndef RASPI_CAMERA_H_
#define RASPI_CAMERA_H_

#include "util.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

class RaspiCamera
{

public:
    RaspiCamera(Target target);
    cv::Mat update_current_image(float unused1, float unused2, float unused3, float unused4, int unused5 = 0);

private:
    Target m_target;
    // raspicam::RaspiCam_Cv camera;
    cv::VideoCapture camera;
};

#endif // RASPI_CAMERA_H_
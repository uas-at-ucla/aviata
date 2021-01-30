#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "util.hpp"
#include <raspicam/raspicam_cv.h>
#include <opencv2/core.hpp>

class RaspiCamera {

public:
    RaspiCamera();
    ~RaspiCamera();
    cv::Mat update_current_image();

private:
    // raspicam::RaspiCam_Cv camera;
};


#endif // CAMERA_SIMULATOR_H_
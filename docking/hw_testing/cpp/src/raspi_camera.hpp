#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "util.hpp"
#include <opencv2/core.hpp>

using namespace cv;

class RaspiCamera {

public:
    RaspiCamera();
    ~RaspiCamera();
    Mat update_current_image();

private:
};


#endif // CAMERA_SIMULATOR_H_
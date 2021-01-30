#include "raspi_camera.hpp"
#include "util.hpp"

#include <string>
#include <iostream>

#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <raspicam/raspicam_cv.h>

#include "util.hpp"

using namespace cv;
using namespace std;

RaspiCamera::RaspiCamera(Target target)  {

    raspicam::RaspiCam_Cv camera;
    log("Camera", "Connecting...");
    if ( !Camera.open() ) {
        log("Camera", "Connect failed", true);
        return;
    }
    log("Camera", "Connected successfully");

    camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640 );
    camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480 );
    camera.set ( cv::CAP_PROP_BRIGHTNESS, 50 );
    camera.set ( cv::CAP_PROP_CONTRAST , 50 );
    camera.set ( cv::CAP_PROP_SATURATION, 50 );
    camera.set ( cv::CAP_PROP_GAIN, 50 );
    camera.set ( cv::CAP_PROP_FPS, 50 );
    // camera.set ( cv::CAP_PROP_FORMAT, CV_8UC1 ); // this sets grayscale
    // camera.set ( cv::CAP_PROP_EXPOSURE, ??  ); // this sets shutter speed
}

Mat RaspiCamera::update_current_image() {
    cv::Mat image;
    Camera.grab();
    Camera.retrieve(image);
    return image;
}
#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "util.hpp"
#include <opencv2/core.hpp>

using namespace cv;

// Measured in meters, East and North are positive
#define RELATIVE_LAT 0.0
#define RELATIVE_LON 0.0
#define RELATIVE_ALT 22.5

// Measured degrees counterclockwise from north
#define RELATIVE_YAW 45.0

// Camera information, measured in degrees
#define CAMERA_FOV_VERTICAL 48.8
#define CAMERA_FOV_HORIZONTAL 62.2

// Target information, measured in centimeters
#define TARGET_SIZE 234
#define PERIPHERAL_TARGET_SIZE 4.50
#define DRONE_RADIUS 1.1135 // measured in meters, 1m boom + half the central tag side length (possibly slightly off for corners but good enough)

#define DISPLAY_SCALE 1750

class CameraSimulator {

public:
    CameraSimulator(Target target);
    void update_target_location(Target target);
    Mat update_current_image(float absLon, float absLat, float absAlt, float absYaw, int target_id=0);

private:
    std::string m_log_tag;
    Mat m_april_tag;
    float m_scale_constant;
    float m_peripheral_scale_constant;
    int m_display_width;
    int m_display_height;
    Target m_target;

    float get_view_scale_constant(float target_size);
    void rotate_image(Mat& image, double angle);
};


#endif // CAMERA_SIMULATOR_H_
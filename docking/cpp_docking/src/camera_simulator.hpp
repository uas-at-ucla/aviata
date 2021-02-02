#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "util.hpp"
#include <opencv2/core.hpp>

using namespace cv;

class CameraSimulator
{

public:
    CameraSimulator(Target target);
    void update_target_location(Target target);
    Mat update_current_image(float absLon, float absLat, float absAlt, float absYaw, int target_id = 0);

private:
    std::string m_log_tag;
    Mat m_april_tag;
    float m_scale_constant;
    float m_peripheral_scale_constant;
    int m_display_width;
    int m_display_height;
    Target m_target;

    float get_view_scale_constant(float target_size);
    void rotate_image(Mat &image, double angle);
};

#endif // CAMERA_SIMULATOR_H_
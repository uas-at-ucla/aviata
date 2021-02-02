#ifndef IMAGE_ANALYZER_H_
#define IMAGE_ANALYZER_H_

#include <opencv2/core.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
using namespace cv;

class ImageAnalyzer
{
public:
    ImageAnalyzer();
    ~ImageAnalyzer();
    float *processImage(Mat img, int ind, float yaw, std::string &tags); //returns x_err, y_err, alt_err, rot_err packaged as float*
private:
    apriltag_detector_t *m_tagDetector;
    apriltag_family_t *tf;
};

#endif //IMAGE_ANALYZER_H
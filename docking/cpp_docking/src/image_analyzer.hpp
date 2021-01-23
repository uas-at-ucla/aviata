#ifndef IMAGE_ANALYZER_H_
#define IMAGE_ANALYZER_H_

#include "../apriltags/AprilTags/TagDetection.h"
#include "../apriltags/AprilTags/TagDetector.h"

#include <opencv2/core.hpp>
using namespace cv;

class ImageAnalyzer{
public:
    ImageAnalyzer();
    ~ImageAnalyzer();
    float* processImage(Mat img, int ind, float yaw, string& tags);
private: 
    AprilTags::TagDetector* m_tagDetector;
};

#endif
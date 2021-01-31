#ifndef IMAGE_ANALYZER_H_
#define IMAGE_ANALYZER_H_

#include <opencv2/core.hpp>
using namespace cv;

class apriltag_detector_t;
class apriltag_family_t;
class ImageAnalyzer{
public:
    ImageAnalyzer();
    ~ImageAnalyzer();
    float* processImage(Mat img, int ind, float yaw, std::string& tags);
private: 
    apriltag_detector_t* m_tagDetector;
    apriltag_family_t* tf;
};

#endif
#ifndef IMAGE_ANALYZER_H_
#define IMAGE_ANALYZER_H_

#include "util.hpp"

#include <opencv2/core.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
using namespace cv;

class ImageAnalyzer
{
public:
    ImageAnalyzer();
    ~ImageAnalyzer();
    bool processImage(Mat img, int ind, std::string &tags, Errors &errs);

private:
    apriltag_detector_t *m_tagDetector;
    apriltag_family_t *tf;
};

#endif //IMAGE_ANALYZER_H
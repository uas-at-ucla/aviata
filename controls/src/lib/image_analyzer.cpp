#include "image_analyzer.hpp"
#include "util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag16h5.h>

#include <string>
#include <math.h>
#include <vector>
#include <algorithm>

ImageAnalyzer::ImageAnalyzer()
{
    //tf = tag36h11_create();
    tf = tag16h5_create();
    m_tagDetector = apriltag_detector_create();
    m_tagDetector->nthreads = 4;
    m_tagDetector->quad_decimate = 1; 
    apriltag_detector_add_family(m_tagDetector, tf);
}
ImageAnalyzer::~ImageAnalyzer()
{
    apriltag_detector_destroy(m_tagDetector);
    //tag36h11_destroy(tf);
    tag16h5_destroy(tf);
}

/**
 * Determines drone's offset from target
 * 
 * @param img image to analyze
 * @param ind index of the target we want to dock to
 * @param tags reference parameter which is set to a string of indices representing detected tags
 * @param errs reference parameter which is set to the errors detected by this analyzer (4 element array, x y z yaw errors in that order)
 * @return true if target was detected, false otherwise (and errs remains unchanged if so)
 * 
 * */
bool ImageAnalyzer::processImage(Mat img, int ind, std::string &tags, Errors &errs)
{
    Point2f image_center(img.cols / 2, img.rows / 2);

    cvtColor(img, img, COLOR_BGR2GRAY);
    image_u8_t im = {.width = img.cols, .height = img.rows, .stride = img.cols, .buf = img.data}; //Converts CV2 image to C-friendly image format
    zarray_t *dets = apriltag_detector_detect(m_tagDetector, &im);                                //Detects apriltags

    //Prints detected tags
    std::string tagsDetected = "";
    for (int i = 0; i < zarray_size(dets); i++)
    {
        apriltag_detection_t *det;
        zarray_get(dets, i, &det);
        if (det->decision_margin >= MARGIN)
        {
            tagsDetected += std::to_string(det->id) + "(" + std::to_string(det->decision_margin) + ") ";
        }
    }
    tags = tagsDetected;

    //Iterates over detected tag to find target tag
    for (int i = 0; i < zarray_size(dets); i++)
    {
        apriltag_detection_t *det;
        zarray_get(dets, i, &det);

        if (det->id == ind && det->decision_margin >= MARGIN)
        { //Target tag found

            // 1. Calculate altitude error

            //Gathers location information on detection
            double *center = det->c;
            double rect[4][2];
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    rect[i][j] = (det->p)[i][j];
                }
            }

            //Calculates average side length
            std::vector<float> sideList;
            for (int i = 0; i < 4; i++)
            {
                for (int j = i + 1; j < 4; j++)
                {
                    sideList.push_back((float)sqrt(pow((rect[i][0] - rect[j][0]), 2) + pow((rect[i][1] - rect[j][1]), 2)));
                }
            }
            std::sort(sideList.begin(), sideList.end());
            float sideAvg = 0;
            for (int i = 0; i < 4; i++)
            {
                sideAvg += sideList[i];
            }
            sideAvg /= 4.0;

            //Calculates pixel density of image
            float tag_pixel_ratio;
            if (ind == 0)
            {
                tag_pixel_ratio = ((float)CENTRAL_TAG_SIZE) / sideAvg;
            }
            else
            {
                tag_pixel_ratio = ((float)PERIPHERAL_TAG_SIZE) / sideAvg;
            }

            errs.alt = 0.50 / tan((CAMERA_FOV_HORIZONTAL * 0.50) * M_PI / 180.0) * img.cols * tag_pixel_ratio; //Calculates alt_err

            // 2. Calculate rotation error

            float y3 = rect[3][1]; //Chooses points for calculating diagonal angle
            float y0 = rect[0][1];
            float x3 = rect[3][0];
            float x0 = rect[0][0];

            //Calculates diagonal angle
            float incline_angle;
            if (x3 - x0 != 0)
            {
                incline_angle = 180.0 / M_PI * atan2(y3 - y0, x3 - x0);
                incline_angle = incline_angle * -1.0 + 90.0;
                if (incline_angle > 0)
                {
                    incline_angle -= 180;
                }
                else
                {
                    incline_angle += 180;
                }
            }
            else if (y3 > y0)
            {
                incline_angle = -180;
            }
            else
            {
                incline_angle = 0;
            }
            errs.yaw = incline_angle * -1; //Finds rotational error from diagonal angle


            // 3. Calculate x/y error
            
            // errs.x = (center[0] - image_center.x) * tag_pixel_ratio * -1; // * -1 since camera rotated 180 from drone
            // errs.y = (image_center.y - center[1]) * tag_pixel_ratio * -1; // same ^
            errs.x = (center[0] - image_center.x) * tag_pixel_ratio; // simulated camera is not rotated relative to drone
            errs.y = (image_center.y - center[1]) * tag_pixel_ratio;
            errs.tag_pixel_ratio = tag_pixel_ratio;

            //Cleanup
            apriltag_detection_destroy(det);
            zarray_destroy(dets);
            return true;
        }
        apriltag_detection_destroy(det);
    }
    zarray_destroy(dets);
    return false;
}
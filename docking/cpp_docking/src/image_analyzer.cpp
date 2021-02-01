#include "image_analyzer.hpp"
#include "util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <string>
#include <math.h>
#include <vector>
#include <algorithm>

ImageAnalyzer::ImageAnalyzer(){
    tf=tag36h11_create();
    m_tagDetector=apriltag_detector_create();
    apriltag_detector_add_family(m_tagDetector,tf);

}
ImageAnalyzer::~ImageAnalyzer(){
    apriltag_detector_destroy(m_tagDetector);
    tag36h11_destroy(tf);
}

float* ImageAnalyzer::processImage(Mat img, int ind, float yaw, std::string& tags){
    float* errs=new float[4];

    int width=img.cols;
    int height=img.rows;
    Point2f image_center(img.cols / 2, img.rows / 2);

    //Rotates target image and converts to greyscale
    Mat rot_mat = getRotationMatrix2D(image_center, -1.0*yaw, 1.0);
    warpAffine(img, img, rot_mat, Size(img.cols, img.rows), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
    cvtColor(img,img,COLOR_BGR2GRAY);

    image_u8_t im={.width=img.cols,.height=img.rows, .stride=img.cols, .buf=img.data}; //Converts CV2 image to C-friendly image format

    zarray_t* dets=apriltag_detector_detect(m_tagDetector,&im); //Detects apriltags

    //Prints detected tags
    std::string tagsDetected="";
    for(int i=0;i<zarray_size(dets);i++){
        apriltag_detection_t *det;
        zarray_get(dets,i,&det);
        if(det->decision_margin>=MARGIN){
            tagsDetected+=std::to_string(det->id)+" ";
        }
    }
    tags=tagsDetected;

    //Iterates over detected tag to find target tag
    for(int i=0;i<zarray_size(dets);i++){
        apriltag_detection_t *det;
        zarray_get(dets,i,&det);

        if(det->id==ind&&det->decision_margin>=MARGIN){ //Target tag found

            //Gathers location information on detection
            double* center=det->c;
            double rect[4][2];
            for(int i=0;i<4;i++){
                for(int j=0;j<2;j++){
                    rect[i][j]=(det->p)[i][j];
                }
            }

            //Calculates average side length
            std::vector<float> sideList;
            for(int i=0;i<4;i++){
                for(int j=i+1;j<4;j++){
                    sideList.push_back((float)sqrt(pow((rect[i][0]-rect[j][0]),2)+pow((rect[i][1]-rect[j][1]),2)));
                }
            }
            std::sort(sideList.begin(),sideList.end());
            float sideAvg=0;
            for(int i=0;i<4;i++){
                sideAvg+=sideList[i];
            }
            sideAvg/=4.0;

            //Calculates pixel density of image
            float tag_pixel_ratio;
            if(ind==0){
                tag_pixel_ratio=((float)CENTRAL_TAG_SIZE)/sideAvg;
            }
            else{
                tag_pixel_ratio=((float)PERIPHERAL_TAG_SIZE)/sideAvg;
            }

            errs[2]=0.50/tan((CAMERA_FOV_HORIZONTAL*0.50)*M_PI/180.0)*width*tag_pixel_ratio; //Calculates alt_err

            float y3=rect[3][1]; //Chooses points for calculating diagonal angle
            float y0=rect[0][1];
            float x3=rect[3][0];
            float x0=rect[0][0];

            //Calculates diagonal angle
            float incline_angle;
            if(x3-x0!=0){
                incline_angle=180.0/M_PI *atan2(y3-y0,x3-x0);
                incline_angle=incline_angle*-1.0+90.0;
                if(incline_angle>0){
                    incline_angle-=180;
                }
                else{
                    incline_angle+=180;
                }
            }
            else if(y3>y0){
                incline_angle=-180;
            }
            else{
                incline_angle=0;
            }
            errs[3]=incline_angle*-1; //Finds rotational error from diagonal angle

            //Finds the offset of the image location, finds x/y err
            float x_offset=center[0]-image_center.x;
            float y_offset=image_center.y-center[1];
            errs[0]=x_offset*tag_pixel_ratio;
            errs[1]=y_offset*tag_pixel_ratio;

            //Cleanup
            apriltag_detection_destroy(det);
            zarray_destroy(dets);
            return errs;
        }
        apriltag_detection_destroy(det);
    }
    zarray_destroy(dets);
    return nullptr; //Tag not detected, returns nullptr
}
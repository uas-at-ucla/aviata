#include "raspi_camera.hpp"
#include "util.hpp"

// #include <string>
// #include <iostream>

// #include <ctime>
// #include <cstdlib>
// #include <fstream>
// #include <sstream>
#include <future>
#include <raspicam/raspicam_cv.h>

#include "util.hpp"
#include "raspi_camera.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;
using std::this_thread::sleep_for;

RaspiCamera::RaspiCamera() {
    // if ( !camera.open() ) {
    //     log("Camera", "Connect failed", true);
    //     return;
    // }
    // log("Camera", "Connected successfully");

    // camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640 );
    // camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480 );
    // camera.set ( cv::CAP_PROP_BRIGHTNESS, 50 );
    // camera.set ( cv::CAP_PROP_CONTRAST , 50 );
    // camera.set ( cv::CAP_PROP_SATURATION, 50 );
    // camera.set ( cv::CAP_PROP_GAIN, 50 );
    // camera.set ( cv::CAP_PROP_FPS, 0 );
    // camera.set ( cv::CAP_PROP_FORMAT, CV_8UC1 ); // this sets grayscale
    // camera.set ( cv::CAP_PROP_EXPOSURE, ??  ); // this sets shutter speed
}

RaspiCamera::~RaspiCamera() {
    // camera.release();
}

Mat RaspiCamera::update_current_image() {
    // sleep_for(seconds(3));
    // cv::Mat image;
    // camera.grab();
    // camera.retrieve(image);
    // cv::imshow("memes", image);
    // cv::waitKey(0);

    raspicam::RaspiCam_Cv Camera;


    Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640 );
    Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480 );
    Camera.set ( cv::CAP_PROP_BRIGHTNESS, 50 );
    Camera.set ( cv::CAP_PROP_CONTRAST , 50 );
    Camera.set ( cv::CAP_PROP_SATURATION, 50 );
    Camera.set ( cv::CAP_PROP_GAIN, 50 );
    Camera.set ( cv::CAP_PROP_FPS,50 );
    Camera.set ( cv::CAP_PROP_FORMAT, CV_8UC1 ); // this sets grayscale
    if ( !Camera.open() ) {
        log("Camera", "Connect failed", true);
    }
    log("Camera", "Connected successfully");

    cv::Mat image;
    for (int i = 0; i < 5; i++) {
    cout<<"Capturing"<<endl;

        Camera.grab();
        Camera.retrieve ( image );
        cv::imshow("test", image);
        cv::waitKey(0);
        // cv::imwrite("test.png", image);

    }
    Camera.release();
    return image;
}


// int main ( int argc,char **argv ) {

//     cv::Mat image;
//     int nCount=getParamVal ( "-nframes",argc,argv, 100 );
//     cout<<"Capturing"<<endl;

//     double time_=cv::getTickCount();

//     for ( int i=0; i<nCount || nCount==0; i++ ) {
//         Camera.grab();
//         Camera.retrieve ( image );
//         if ( !doTestSpeedOnly ) {
//             if ( i%5==0 ) 	  cout<<"\r capturing ..."<<i<<"/"<<nCount<<std::flush;
//             if ( i%30==0 && i!=0 )
//                 cv::imwrite ("image"+std::to_string(i)+".jpg",image );
//         }
//     }
//     if ( !doTestSpeedOnly )  cout<<endl<<"Images saved in imagexx.jpg"<<endl;
//     double secondsElapsed= double ( cv::getTickCount()-time_ ) /double ( cv::getTickFrequency() ); //time in second
//     cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<< ( float ) ( ( float ) ( nCount ) /secondsElapsed ) <<endl;
//     Camera.release();


#include <iostream>
#include <memory>
#include <thread>
#include <opencv_project/depth_computations.hpp>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ZED Libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

using namespace sl::zed;
using namespace std;
using namespace cv;

void depth_()
{

}


// This function deals with all the depth realted computations using ZED and openCV
void depth_reception()
{
    // Input from keyboard
    char keyboard = ' ';
    int quality = sl::zed::MODE::PERFORMANCE;
    int gpu_id = -1;
    int zed_id = 0;
    int frame_rate = 30;
    int resolution = sl::zed::HD1080;

    std::unique_ptr<sl::zed::Camera> zed;
    sl::zed::InitParams param;

    param.unit = UNIT::METER;
    param.coordinate = COORDINATE_SYSTEM::IMAGE;
    param.mode = static_cast<sl::zed::MODE> (quality);
    param.verbose = true;
    param.device = gpu_id;
    param.disableSelfCalib = false;
    cout << "Done Init Vars" << endl;

    // Initialize ZED color stream in HD and depth in Performance mode
    zed.reset(new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode> (resolution), frame_rate, zed_id));

    ERRCODE err = ERRCODE::ZED_NOT_AVAILABLE;
    while (err != SUCCESS) {
        err = zed->init(param);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    cout << "Camera Init Done." << endl;

    // Quit if an error occurred
    if (err != sl::zed::SUCCESS)
    {
        std::cout << "Unable to init the ZED:" << errcode2str(err) << std::endl;
    }
    else
    {
        std::cout << "Successfully initialized the ZED Camera" << std::endl;
    }

    // Initializing




    // Initialize color image and depth
    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
    cv::Mat image(height, width, CV_8UC4,1);
    cv::Mat depth(height, width, CV_8UC4,1);

    // Create OpenCV windows
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

    // Settings for windows
    cv::Size displaySize(720, 404);
    cv::Mat imageDisplay(displaySize, CV_8UC4);
    cv::Mat depthDisplay(displaySize, CV_8UC4);

    // Loop until 'q' is pressed
    while (keyboard != 'q')
    {
        // Grab frame and compute depth in FILL sensing mode
        if (!zed->grab(sl::zed::SENSING_MODE::FILL))
        {
             // Retrieve left color image
             sl::zed::Mat left = zed->retrieveImage(sl::zed::SIDE::LEFT);
             memcpy(image.data,left.data,width*height*4*sizeof(uchar));

            // Retrieve depth map
             sl::zed::Mat depthmap = zed->normalizeMeasure(sl::zed::MEASURE::DEPTH);
             memcpy(depth.data,depthmap.data,width*height*4*sizeof(uchar));

             // Display image in OpenCV window
             cv::resize(image, imageDisplay, displaySize);
             cv::imshow("Image", imageDisplay);

             // Display depth map in OpenCV window
             cv::resize(depth, depthDisplay, displaySize);
             cv::imshow("Depth", depthDisplay);
        }

        keyboard = cv::waitKey(30);

    }
}



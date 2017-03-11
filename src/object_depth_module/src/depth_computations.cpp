#include <iostream>
#include <memory>
#include <thread>
#include <object_depth_module/depth_computations.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

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

// This function deals with all the depth realted computations using ZED and openCV
void depth_reception(bool is_live, string svo_path, int depth_clamp)
{
    int svo_position = 0;

    // Initializing while loop timer parameter
    boost::asio::io_service service_io;
    boost::asio::deadline_timer timer(service_io, boost::posix_time::seconds(2));

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

    // If not live and not 0, limit the max depth value
    if(!is_live && depth_clamp != 0) {
        zed->setDepthClampValue(depth_clamp);
    }

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

             cout << "Closest distance: " << zed->getClosestDepthValue() << endl;
             cout << "Confidence Threshold: " << zed->getConfidenceThreshold() << endl;

             if(!is_live) {
                 svo_position = zed->getSVOPosition();
                 svo_position += 10;
                 zed->setSVOPosition(svo_position);
             }
             timer.wait();
        }
        keyboard = cv::waitKey(30);
    }
}



#ifndef OBJECT_DEPTH_INTERFACE_HPP
#define OBJECT_DEPTH_INTERFACE_HPP

// System libraries
#include <iostream>

// ZED libraries
#include <zed/Camera.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class object_depth_interface
{
public:
    object_depth_interface(bool live_flag, string svo_file_path, int depth_requested);

    // ZED Camera related functions
    int initialize_ZED_camera();
    int initialize_camera_reception();      // Function to initialize the camera
    int receive_images();

    //Depth related functions
    int depth_reception();

    //Object related functions
    int object_detection_initialize();
    int object_detect();
//    int object_detection();
    int object_detection(int argc, char *argv[]);


private:
    std::unique_ptr<sl::zed::Camera> zed;
    bool is_live;
    string svo_path;

    // Image Storage
    int image_width;
    int image_height;
    cv::Size displaySize;
    cv::Mat left_image;
    cv::Mat right_image;

    // Depth related data
    int depth_clamp;

    // Object detection
    cv::Mat HSV;                            //matrix storage for HSV image
    cv::Mat threshold;                      //matrix storage for binary threshold image
};

#endif // OBJECT_DEPTH_INTERFACE_HPP

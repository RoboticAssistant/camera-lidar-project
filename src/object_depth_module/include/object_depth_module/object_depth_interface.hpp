#ifndef OBJECT_DEPTH_INTERFACE_HPP
#define OBJECT_DEPTH_INTERFACE_HPP

// System libraries
#include <iostream>

// ZED libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
using namespace sl::zed;

class object_depth_interface
{
public:
    object_depth_interface(bool live_flag, string svo_file_path, int depth_requested,
                           int confidence_requested);

    // ZED Camera related functions
    int initialize_ZED_camera();
    int initialize_camera_reception();      // Function to initialize the camera
    ERRCODE receive_images();

    //Depth related functions
    int depth_reception();

    // Face detection related function
    int face_detection_engine();

    //Object related functions
    int object_detection_initialize();
    int object_detect();

    // Object and depth interface
    // Returns the
    int depth_object_interface();
    int object_detect_from_disparity();

private:
    std::unique_ptr<sl::zed::Camera> zed;
    bool is_live;
    string svo_path;
    int svo_position;

    // Image Storage
    int image_width;
    int image_height;
    cv::Size displaySize;
    cv::Mat left_image;
    cv::Mat right_image;

    // Depth related data
    int depth_clamp;
    cv::Mat depth_map;

    // Disparity
    int confidence;
    cv::Mat disparity_image;

    // Face related data
    int left_faces_detected;
    int right_faces_detected;

    // Object detection
    cv::Mat HSV;                            //matrix storage for HSV image
    cv::Mat threshold;                      //matrix storage for binary threshold image

    // Object depth interface
    float *objects_distances;
};

int face_detector_initializer();
#endif // OBJECT_DEPTH_INTERFACE_HPP

#include <iostream>
#include <memory>
#include <thread>
#include <stdio.h>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ROS libraries
#include <ros/ros.h>

// ZED Libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

// CUDA Libraries
#include <cuda.h>
#include <cuda_runtime_api.h>

//// Internal Libraries
#include <object_depth_module/object_depth_interface.hpp>
#include <publisher_subscriber/pub_sub.hpp>

using namespace sl::zed;
using namespace std;
using namespace cv;

/// If not using any of these options, just keep them either "" or 0
// Usage: <exe> [is_live]<true/false> [svo_file_name(Can be "" if no file)] [depth_clamp] [confidence]

Pub_Sub face_information_publisher;

int set_up_communication() {

    ros::NodeHandle node_handle;

    // Publisher 1: From Decision to Motor: Forward
    msg_details face_information_details;
    ros::Subscriber sub;
    face_information_details.is_talker = true;
    face_information_details.is_listener = false;
    face_information_details.pub_repeat = false;
    face_information_details.frequency = 1;
    face_information_details.message_topic = "face_information";
    face_information_publisher.init(node_handle, face_information_details, sub);

    // To write publishers for other important data here


    return 0;
}

int main(int argc, char *argv[])
{
    bool is_live = false;
    string svo_file_path, svo_file_name(argv[2]), input_status(argv[1]);
    char keyboard = ' ';

    ros::init(argc, argv, "camera_communication_module");
    set_up_communication();

    if(argc != 5) {
        cout << "Error in Usage:::Usage: <exe> [is_live]<true/false> [svo_file_name(Can be "" if no file)] [depth_clamp] [confidence]" << endl;
        return -1;
    }
    int depth_clamp = atoi(argv[3]);
    int confidence = atoi(argv[4]);

    int size_of_file_name = (sizeof(svo_file_name)/sizeof(char));
    if(!strcmp(input_status.c_str(), "true")) {
        is_live = true;
    }
    else {
        is_live = false;
    }

    if(!is_live) {
        cout << "From Stored Camera Feed" << endl;
        svo_file_path = "/media/ubuntu/7A1A38B11A386C6F/Video_Recordings/";
        svo_file_path.append(svo_file_name.c_str());
    }
    else {
        cout << "From Live Camera Feed" << endl;
    }

//     Handling initial camera object and it's initialization
    object_depth_interface camera_processing(is_live, svo_file_path, depth_clamp, confidence);
    camera_processing.initialize_ZED_camera();
    camera_processing.initialize_camera_reception();

//     Initializing face detector Initialization
    if(!face_detector_initializer()) {
        cout << "face_detection_module: Face detector Initialized Successfully" << endl;
    }

    ros::Rate loop_rate(1);
    ERRCODE error;

    while(ros::ok())
    {
        error = camera_processing.receive_images();
        if(!error) {
            camera_processing.face_detection_engine();
//            camera_processing.object_detect();
            camera_processing.object_detect_from_disparity();
            camera_processing.depth_object_interface();
        }
        else {
            cout << "object_depth_module_node: Error: " << error << endl;
        }
        loop_rate.sleep();
        cv::waitKey(10);
    }
    return 0;
}


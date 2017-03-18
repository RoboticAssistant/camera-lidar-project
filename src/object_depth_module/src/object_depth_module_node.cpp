#include <iostream>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ZED Libraries
#include <zed/Camera.hpp>

// CUDA Libraries
#include <cuda.h>
#include <cuda_runtime_api.h>

//// Internal Libraries
#include <object_depth_module/depth_computations.hpp>
#include <object_depth_module/object_depth_interface.hpp>

using namespace sl::zed;
using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    bool is_live = false;
    string svo_file_path, svo_file_name(argv[2]), input_status(argv[1]);
    char keyboard = ' ';

    // Initializing while loop timer parameter
    boost::asio::io_service service_io;
    boost::asio::deadline_timer timer(service_io, boost::posix_time::seconds(2));

    if(argc != 4) {
        cout << "Error in Usage:::Usage: <exe> [is_live]<true/false> <svo_file_name> <depth_clamp>" << endl;
        return -1;
    }
    int depth_clamp = atoi(argv[3]);

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

    // Handling initial camera object and it's initialization
    object_depth_interface depth_display(is_live, svo_file_path, depth_clamp);
    depth_display.initialize_ZED_camera();
    depth_display.initialize_camera_reception();

    // Initializing data for object detection
    depth_display.object_detection_initialize();

    while (keyboard != 'q')
    {
        // Depth Calculations
        if(!depth_display.receive_images()) {
              depth_display.depth_reception();
//            depth_display.object_detect();
//            depth_display.object_detection(argc, argv);
        }
        else {
            cout << "object_depth_module_node: image reception" << endl;
        }

        timer.wait();
        keyboard = cv::waitKey(30);
    }

    waitKey(0);
    return 0;
}


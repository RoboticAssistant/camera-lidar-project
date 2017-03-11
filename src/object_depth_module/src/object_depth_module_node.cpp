#include <iostream>
#include <memory>
#include <thread>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ZED Libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

// CUDA Libraries
#include <cuda.h>
#include <cuda_runtime_api.h>

//// Internal Libraries
#include <object_depth_module/depth_computations.hpp>


using namespace sl::zed;
using namespace std;
using namespace cv;

std::unique_ptr<sl::zed::Camera> zed;

int initialize_ZED_camera()
{
    // Initialization of the parameters
    char keyboard = ' ';
    int quality = sl::zed::MODE::PERFORMANCE;
    int gpu_id = -1;
    int zed_id = 0;
    int frame_rate = 30;
    int resolution = sl::zed::HD720;

    // Initialization of camera object and it's requirements
    sl::zed::InitParams param;

    // Filling of camera object parameter
    param.unit = UNIT::METER;                                   // METER looks Good
    param.coordinate = COORDINATE_SYSTEM::IMAGE;                // IMAGE looks Good
    param.mode = static_cast<sl::zed::MODE> (quality);          // Good
    param.verbose = true;                                       // Good
    param.device = gpu_id;                                      // Good
    param.disableSelfCalib = false;                             // Good
    cout << "===================" << "Done Variable Initialization" << "===================" << endl;                           // Good

    cout << svo_path << endl;
    if(is_live) {
        // Initialize ZED color stream in HD and depth in Performance mode
        zed.reset(new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode> (resolution), frame_rate, zed_id));
        cout << "Creating live camera object" << endl;
    }
    else {
        zed.reset(new sl::zed::Camera(svo_path));
        cout << "Creating stored camera object" << endl;
    }

    ERRCODE err = ERRCODE::ZED_NOT_AVAILABLE;
    while (err != SUCCESS) {
        err = zed->init(param);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    cout << "=======================" << "ZED Camera Initialized" << "=======================" << endl;                           // Good

    // Quit if an error occurred
    if (err != sl::zed::SUCCESS) {
        std::cout << "Unable to initialized the ZED:" << errcode2str(err) << std::endl;
    }
    else {
        std::cout << "Successfully initialized the ZED Camera" << std::endl;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    bool is_live = false;
    string svo_file_path, svo_file_name(argv[2]), input_status(argv[1]);

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

    // Depth Calculations
    depth_reception(is_live, svo_file_path, depth_clamp);

    waitKey(0);
    return 0;
}


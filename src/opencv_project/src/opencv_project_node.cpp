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
#include <opencv_project/convolution_2d.hpp>
#include <opencv_project/depth_computations.hpp>


using namespace sl::zed;
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{	
    cv::Mat inputImage, grayImage, binaryImage;
    string kernelFileName;


    // Checking for the input argument image path
    if(argc != 3)
    {
        cout << "Error in Usage\nUsage: <exe> <image_name> <kernel_file_name>" << endl;
        return -1;
    }

    inputImage = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    // Checking the image data. If there is any data-OK. If not, error.
    if(!inputImage.data)
    {
        cout << "Could not open this image. Please try with any other Image";
        return -1;
    }

    // Showing the input image. *This is in order to keep an order of all the images
//    namedWindow("Input Window", WINDOW_AUTOSIZE);
//    imshow("Input Window", inputImage);

    // Converting that image into a grayscale image.
    cvtColor(inputImage, grayImage, COLOR_RGB2GRAY);

    // Showing the grayscale image. *This is in order to keep an order of all the images
//    namedWindow("Grayscale Window", WINDOW_AUTOSIZE);
//    imshow("Grayscale Window", grayImage);

    // Coverting a grayscale image into a binary image.
    threshold(grayImage, binaryImage, 100, 255, CV_THRESH_BINARY);

    // Showing the binary image.
//    namedWindow("Binary Image", WINDOW_AUTOSIZE);
//    imshow("Binary Image", binaryImage);

    // Perform the convolution of this gray image.
    // Get the convolution kernel
    kernelFileName = argv[2];
    //perform_convolution(binaryImage, kernelFileName);

    depth_reception();

    // Input from keyboard
    char keyboard = ' ';
    int quality = sl::zed::MODE::PERFORMANCE;
    int gpu_id = -1;
    int zed_id = 0;
    int frame_rate = 30;
    sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA;

//    std::unique_ptr<sl::zed::Camera> zed;
    sl::zed::InitParams param;

//    param.unit = UNIT::METER;
//    param.coordinate = COORDINATE_SYSTEM::IMAGE;
//    param.mode = static_cast<sl::zed::MODE> (quality);
    param.verbose = true;
    param.device = gpu_id;
      param.disableSelfCalib = true;

    cout << "Done Init Vars" << endl;

    // Initialize ZED color stream in HD and depth in Performance mode
    sl::zed::Camera *zed = new sl::zed::Camera(resolution, frame_rate, zed_id);
    cout << "Camera Init Done." << endl;

    ERRCODE err = ERRCODE::ZED_NOT_AVAILABLE;
    while (err != SUCCESS) {
        err = zed->init(param);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    // Quit if an error occurred
    if (err != sl::zed::SUCCESS)
    {
        std::cout << "Unable to init the ZED:" << errcode2str(err) << std::endl;
    }
    else
    {
        std::cout << "Success init the ZED:" << std::endl;
    }

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
    while (keyboard != 'q') {

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


    waitKey(0);
	return 0;
}

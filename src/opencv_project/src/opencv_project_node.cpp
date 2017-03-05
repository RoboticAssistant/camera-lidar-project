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


//    cv::Mat inputImage, grayImage, binaryImage;
//    string kernelFileName;


//    // Checking for the input argument image path
//    if(argc != 4)
//    {
//        cout << "Error in Usage\nUsage: <exe> <image_name> <kernel_file_name>" << endl;
//        return -1;
//    }

//    inputImage = imread(argv[1], CV_LOAD_IMAGE_COLOR);

//    // Checking the image data. If there is any data-OK. If not, error.
//    if(!inputImage.data)
//    {
//        cout << "Could not open this image. Please try with any other Image";
//        return -1;
//    }

//    // Showing the input image. *This is in order to keep an order of all the images
//    namedWindow("Input Window", WINDOW_AUTOSIZE);
//    imshow("Input Window", inputImage);

//    // Converting that image into a grayscale image.
//    cvtColor(inputImage, grayImage, COLOR_RGB2GRAY);

//    // Showing the grayscale image. *This is in order to keep an order of all the images
//    namedWindow("Grayscale Window", WINDOW_AUTOSIZE);
//    imshow("Grayscale Window", grayImage);

//    // Coverting a grayscale image into a binary image.
//    threshold(grayImage, binaryImage, 100, 255, CV_THRESH_BINARY);

//    // Showing the binary image.
//    namedWindow("Binary Image", WINDOW_AUTOSIZE);
//    imshow("Binary Image", binaryImage);

//    // Perform the convolution of this gray image.
//    // Get the convolution kernel
//    kernelFileName = argv[2];
//    perform_convolution(binaryImage, kernelFileName, argv[3]);

    waitKey(0);
	return 0;
}

#include <iostream>
#include <memory>
#include <thread>
#include <face_detect_module/face_detection_engine.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

// ZED Libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

using namespace sl::zed;
using namespace std;
using namespace cv;

// Copy this file from opencv/data/haarscascades to target folder
string face_cascade_name = "c:/haarcascade_frontalface_alt.xml";
string window_name = "Capture - Face detection";
int filenumber; // Number of file to be saved
string filename;

// Function detectAndDisplay
void detectAndDisplay(cv::CascadeClassifier face_cascade, cv::Mat frame)
{
    std::vector<Rect> faces;
    cv::Mat frame_gray;
    cv::Mat crop;
    cv::Mat res;
    cv::Mat gray;
    string text;
    stringstream sstm;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;
    cv::Rect roi_c;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)

    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);
        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, CV_BGR2GRAY); // Convert cropped image to Grayscale

        // Form a filename
        filename = "";
        stringstream ssfn;
        ssfn << filenumber << ".png";
        filename = ssfn.str();
        filenumber++;

        imwrite(filename, gray);

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
        rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
    }

    // Show image
    sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
    text = sstm.str();

    putText(frame, text, cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
    imshow("original", frame);

    if (!crop.empty())
    {
        imshow("detected", crop);
    }
    else
        destroyWindow("detected");
}

int face_detector()
{
    // Global variables
    cv::CascadeClassifier face_cascade(face_cascade_name);

    // Load the cascade
    if (!face_cascade.load(face_cascade_name)){
        printf("--(!)Error loading\n");
        return (-1);
    }

    // Read the image file
    cv::Mat frame = imread("yourImageName.bmp");

    // Apply the classifier to the frame
    if (!frame.empty()){
        detectAndDisplay(face_cascade, frame);
    }
    else{
        printf(" --(!) No captured frame -- Break!");
    }

    return 0;
}

void face_detection_engine(bool is_live, string svo_path)
{
    int svo_position = 0;

    // Initializing while loop timer parameter
    boost::asio::io_service service_io;
    boost::asio::deadline_timer timer(service_io, boost::posix_time::seconds(2));

    // Initializatio of the parameters
    char keyboard = ' ';
    int quality = sl::zed::MODE::PERFORMANCE;
    int gpu_id = -1;
    int zed_id = 0;
    int frame_rate = 30;
    int resolution = sl::zed::HD720;

    // Initialization of camera object and it's requirements
    std::unique_ptr<sl::zed::Camera> zed;
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

    // Initialize color image and depth
    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
    cv::Mat image_left(height, width, CV_8UC4,1);
    cv::Mat image_right(height, width, CV_8UC4,1);

    // Create OpenCV windows
    cv::namedWindow("Left Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right Depth", cv::WINDOW_AUTOSIZE);

    // Settings for windows
    cv::Size displaySize(720, 404);
    cv::Mat left_image_display(displaySize, CV_8UC4);
    cv::Mat right_image_display(displaySize, CV_8UC4);

    face_detector();
    // Loop until 'q' is pressed
    while (keyboard != 'q')
    {
        // Grab frame and compute depth in FILL sensing mode
        if (!zed->grab(sl::zed::SENSING_MODE::FILL))
        {
             // Retrieve left color image
             sl::zed::Mat left = zed->retrieveImage(sl::zed::SIDE::LEFT);
             memcpy(image_left.data, left.data,width*height*4*sizeof(uchar));

             sl::zed::Mat right = zed->retrieveImage(sl::zed::SIDE::RIGHT);
             memcpy(image_right.data, right.data,width*height*4*sizeof(uchar));

             // Display image in OpenCV window
             cv::resize(image_left, left_image_display, displaySize);
             cv::imshow("Left Image", left_image_display);

             cv::resize(image_right, right_image_display, displaySize);
             cv::imshow("Right Image", left_image_display);

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

#include <iostream>
#include <memory>
#include <thread>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/opencv_modules.hpp>

// ZED Libraries
#include <zed/Camera.hpp>

// CUDA Libraries
#include <cuda.h>
#include <cuda_runtime_api.h>

#include "object_depth_module/object_depth_interface.hpp"
#include "object_depth_module/object_detection.hpp"
#include <publisher_subscriber/pub_sub.hpp>

using namespace std;
using namespace sl::zed;
using namespace cv;

extern int H_VAL_MIN;
extern int H_VAL_MAX;
extern int S_VAL_MIN;
extern int S_VAL_MAX;
extern int V_VAL_MIN;
extern int V_VAL_MAX;

const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

cv::Mat contour_display_mat;
cv::vector<Vec4i> hierarchy;
vector<vector<Point> > contours;

extern Pub_Sub face_information_publisher;

int object_depth_interface::initialize_ZED_camera()
{
    cout << "initialize_ZED_camera(): Initialized ZED Camera." << endl;
    // Initialization of the parameters
    int quality = sl::zed::MODE::QUALITY;
    int gpu_id = -1;
    int zed_id = 0;
    int frame_rate = 30;
    int resolution = sl::zed::HD720;

    // Initialization of camera object and it's requirements
    sl::zed::InitParams param;

    // Filling of camera object parameter
    param.unit = UNIT::MILLIMETER;                              // METER looks Good
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

object_depth_interface::object_depth_interface(bool live_flag, string svo_file_path, int depth_requested, int confidence_requested)
    : displaySize(720, 404)
{
    is_live = live_flag;
    svo_path = svo_file_path;
    depth_clamp = depth_requested;
    confidence = confidence_requested;
    svo_position = 0;
}

int object_depth_interface::object_detection_initialize()
{
    cout << "object_detection_initialize(): Initial object detection module setup done." << endl;
//    createTrackbars();
}

int object_depth_interface::initialize_camera_reception()
{
    cout << "initialize_camera_reception(): Initial camera setup done." << endl;
    // Initialize color image and depth
    image_width = zed->getImageSize().width;
    image_height = zed->getImageSize().height;

    left_image.create(image_height, image_width, CV_8UC4);
    right_image.create(image_height, image_width, CV_8UC4);
    depth_map.create(image_height, image_width, CV_8UC4);

    // If not live and not 0, limit the max depth value
    if(is_live && (depth_clamp != 0 || confidence != 0)) {
        zed->setDepthClampValue(depth_clamp);
        cout << "\ninitialize_camera_reception(): Limiting Depth to " << depth_clamp << endl;

        zed->setConfidenceThreshold(confidence);
        cout << "\ninitialize_camera_reception(): Setting confidence to " << confidence << endl;
    }
}

ERRCODE object_depth_interface::receive_images()
{
    ERRCODE err;
    err = zed->grab(sl::zed::SENSING_MODE::FILL);

    if(!is_live) {
     svo_position = zed->getSVOPosition();
     svo_position += 1;
     zed->setSVOPosition(svo_position);
    }

    // Grab frame and compute depth in FILL sensing mode
    if (!err) {
        cout << "\nreceive_images(): Receiving left Image." << endl;
        // Retrieve left color image
        sl::zed::Mat left = zed->retrieveImage(sl::zed::SIDE::LEFT);
        left_image = slMat2cvMat(left);
        // Display image in OpenCV window
        cv::resize(left_image, left_image, displaySize);

        cout << "receive_images(): Receiving right Image." << endl;
        // Retrieve left color image
        sl::zed::Mat right = zed->retrieveImage(sl::zed::SIDE::RIGHT);
        right_image = slMat2cvMat(right);
        // Display image in OpenCV window
        cv::resize(right_image, right_image, displaySize);

        cout << "receive_images(): Receiving Depth Map." << endl;
        sl::zed::Mat depth = zed->normalizeMeasure(sl::zed::MEASURE::DEPTH);
        depth_map = slMat2cvMat(depth);
        // Display image in OpenCV window
        cv::resize(depth_map, depth_map, displaySize);

        cout << "receive_images(): Receiving Disparity." << endl;
        sl::zed::Mat disparity = zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY);
        disparity_image = slMat2cvMat(disparity);
        // Display image in OpenCV window
        cv::resize(disparity_image, disparity_image, displaySize);
    }

    return err;
}

int object_depth_interface::object_detect()
{
    imshow("Depth", depth_map);
    cout << "object_detect(): Detecting object." << endl;

    //some boolean variables for different functionality within this
    //program
    static bool trackObjects = true;
    static bool useMorphOps = true;

    //x and y values for the location of the object
    static int x=0, y=0;
    //create slider bars for HSV filtering

    //convert frame from BGR to HSV colorspace
    cvtColor(left_image, HSV, COLOR_BGR2HSV);
    //filter HSV image between values and store filtered image to
    //threshold matrix
    inRange(HSV, Scalar(H_VAL_MIN, S_VAL_MIN, V_VAL_MIN), Scalar(H_VAL_MAX, S_VAL_MAX, V_VAL_MAX), threshold);
    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if(useMorphOps)
        morphOps(threshold);
    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
//    if(trackObjects)
//        trackFilteredObject(x, y, threshold, left_image);

    //show frames
//    imshow(windowName1, HSV);

//    // Apply the contour and edge detection on the threshold
//    contour_display_mat = cv::Mat::zeros(image_height, image_width, CV_8UC1);
//    int threshold_val = 100;

//    /// Detect edges using canny
//    Canny(threshold, threshold, threshold_val, threshold_val*2, 3 );
//    imshow(windowName2, threshold);

//    findContours(threshold, contours, hierarchy,
//                  CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

//    for(int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
//    {
//        Scalar color(rand()&255, rand()&255, rand()&255);
//        drawContours(contour_display_mat, contours, idx, color, CV_FILLED, 8, hierarchy);
//    }

//    namedWindow( "Components", 1 );
//    imshow( "Components", contour_display_mat);

    return 0;
}

// Copy this file from opencv/data/haarscascades to target folder
cv::CascadeClassifier face_cascade;
string face_cascade_name = "/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
string window_name = "Capture - Face detection";
int filenumber; // Number of file to be saved
string filename;
int faces_detected = 0;

// Function detectAndDisplay. Returns faces detected
int detectAndDisplay(cv::Mat frame, bool is_left)
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

    faces_detected = faces.size();

    for (ic = 0; ic < faces_detected; ic++) // Iterate through all current elements (detected faces)

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

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
        rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
    }

    text = "Face Detector";

    putText(frame, text, cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);

    if(is_left) {
        imshow("Left: Face Detector", frame);
    }
    else {
        imshow("Right: Face Detector", frame);
    }

    return faces_detected;
}

int face_detector_initializer()
{
    // Load the cascade
    if (!face_cascade.load(face_cascade_name)){
        printf("--(!)Error loading\n");
        return (-1);
    }
    return 0;
}

int object_depth_interface::face_detection_engine()
{
    static bool face_detected = false;
    string buffer_send;

    if (!left_image.empty()){
     left_faces_detected = detectAndDisplay(left_image, true);
    }
    else{
     printf(" --(!) No captured frame -- Break!");
    }

    if (!right_image.empty()){
     right_faces_detected = detectAndDisplay(right_image, false);
    }
    else{
     printf(" --(!) No captured frame -- Break!");
    }

    if(left_faces_detected == right_faces_detected) {
        face_detected = true;
        faces_detected = left_faces_detected;
    }
    else {
        faces_detected = 0;
    }

    if(faces_detected > 0) {
        buffer_send = to_string(faces_detected);
        face_information_publisher.publisher_data(buffer_send);
    }

    face_detected = false;
}

// This function gets the distance of the centroid of all white pixels.
// This is the approximate distance of the objects around
int object_depth_interface::depth_object_interface() {


    return 0;
}

/// This section plays with the disparity and finds different blobs in the image
/// Algo:
/// 1) Convert the disparity to binary: Think of a threshold
/// 2) Apply erosion and dilation to get rid of noise blobs
/// 3) Apply the simple blob detector
/// 4) Get all the blobs. find the midpoint of these blobs
/// 5) Find the distance using the depth map
/// 6) Write a publisher: Sending the notification about following information
///     a) Number of objects
///     b) Distance of those object
int object_depth_interface::object_detect_from_disparity() {
//    // Showing the depth map
//    imshow("Depth Map", depth_map);

    // Showing the disparity
    imshow("Disparity", disparity_image);

    // Converting this image to a binary image
    cv::threshold(disparity_image, disparity_image, 100, 255, CV_THRESH_BINARY);
    // Showing the binarized image for disparity
    imshow("Disparity Binary Image", disparity_image);

    morphOps(disparity_image);
    imshow("Processed Image", disparity_image);

    return 0;
}

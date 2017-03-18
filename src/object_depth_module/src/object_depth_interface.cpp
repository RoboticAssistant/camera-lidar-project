#include <iostream>
#include <memory>
#include <thread>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/opencv_modules.hpp>

// Qt stuff
#include <qt5/QtCore/QTime>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QGraphicsRectItem>
#include <qt5/QtGui/QPen>
#include <qt5/QtGui/QColor>

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif

#undef RENDER_MSERS
#define RENDER_MSERS 0

// ZED Libraries
#include <zed/Camera.hpp>

// CUDA Libraries
#include <cuda.h>
#include <cuda_runtime_api.h>

#include "object_depth_module/object_depth_interface.hpp"
#include "object_depth_module/object_detection.hpp"
#include "find_object_2d/ObjectsStamped.h"


using namespace std;
using namespace sl::zed;
using namespace cv;
using namespace find_object_2d;

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

int object_depth_interface::initialize_ZED_camera()
{
    cout << "initialize_ZED_camera(): Initialized ZED Camera." << endl;
    // Initialization of the parameters
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

object_depth_interface::object_depth_interface(bool live_flag, string svo_file_path, int depth_requested)
    : displaySize(720, 404)
{
    is_live = live_flag;
    svo_path = svo_file_path;
    depth_clamp = depth_requested;
}

// This function deals with all the depth realted computations using ZED and openCV
int object_depth_interface::depth_reception()
{
    int svo_position = 0;

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

    ERRCODE err;
    err = zed->grab(sl::zed::SENSING_MODE::FILL);
    // Grab frame and compute depth in FILL sensing mode
    if (!err) {
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
    }
    else
    {
        cout << err << endl;
    }

    return 0;
}

int object_depth_interface::object_detection_initialize()
{
    cout << "object_detection_initialize(): Initial object detection module setup done." << endl;
    createTrackbars();
}

int object_depth_interface::initialize_camera_reception()
{
    cout << "initialize_camera_reception(): Initial camera setup done." << endl;
    // Initialize color image and depth
    image_width = zed->getImageSize().width;
    image_height = zed->getImageSize().height;

    left_image.create(image_height, image_width, CV_8UC4);
    right_image.create(image_height, image_width, CV_8UC4);
}

int object_depth_interface::receive_images()
{
    ERRCODE err;
    err = zed->grab(sl::zed::SENSING_MODE::FILL);

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

        return 0;
    }
    else {
        return -1;
    }
}

int object_depth_interface::object_detect()
{
    cout << "object_detect(): Detecting object." << endl;
    int svo_position = 0;

    //some boolean variables for different functionality within this
    //program
    static bool trackObjects = true;
    static bool useMorphOps = true;

    //x and y values for the location of the object
    static int x=0, y=0;
    //create slider bars for HSV filtering

     if(!is_live) {
         svo_position = zed->getSVOPosition();
         svo_position += 10;
         zed->setSVOPosition(svo_position);
     }


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
    if(trackObjects)
        trackFilteredObject(x, y, threshold, left_image);

    //show frames
    imshow(windowName2, threshold);
    imshow(windowName, left_image);
    imshow(windowName1, HSV);

    return 0;
}

//int object_depth_interface::object_detection(int argc, char *argv[])
//{
//    QTime time;

//    // GUI stuff
//    QApplication app(argc, argv);

//    //Load as grayscale
//    cv::Mat objectImg = left_image;
//    cv::Mat sceneImg = right_image;

//    if(!objectImg.empty() && !sceneImg.empty())
//    {
//        cout << "Loading images" << endl;
//        std::vector<cv::KeyPoint> objectKeypoints;
//        std::vector<cv::KeyPoint> sceneKeypoints;
//        cv::Mat objectDescriptors;
//        cv::Mat sceneDescriptors;

//        ////////////////////////////
//        // EXTRACT KEYPOINTS
//        ////////////////////////////
//        cv::Ptr<cv::FeatureDetector> detector;
//        // The detector can be any of (see OpenCV features2d.hpp):
//        detector = cv::Ptr<cv::FeatureDetector>(new cv::StarFeatureDetector());

//        detector->detect(objectImg, objectKeypoints);
//        cout << "Object: " << (int)objectKeypoints.size() << " keypoints detected" << endl;

//        detector->detect(sceneImg, sceneKeypoints);
//        cout << "Scene: " << (int)sceneKeypoints.size() << " keypoints detected" << endl;

//        ////////////////////////////
//        // EXTRACT DESCRIPTORS
//        ////////////////////////////
//        cv::Ptr<cv::DescriptorExtractor> extractor;
//        extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::BriefDescriptorExtractor());
//        // extractor = cv::Ptr(new cv::ORB());
//        // extractor = cv::Ptr(new cv::BRISK());
//        // extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::FREAK());
//        extractor->compute(objectImg, objectKeypoints, objectDescriptors);
//        cout << "Object: " << objectDescriptors.rows << " descriptors extracted" << endl;

//        extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
//        cout << "Scene: " << sceneDescriptors.rows << " descriptors extracted" << endl;

//        ////////////////////////////
//        // NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
//        ////////////////////////////
//        cv::Mat results;
//        cv::Mat dists;
//        std::vector<std::vector<cv::DMatch> > matches;
//        int k=2; // find the 2 nearest neighbors
//        bool useBFMatcher = false; // SET TO TRUE TO USE BRUTE FORCE MATCHER
//        if(objectDescriptors.type()==CV_8U)
//        {
//            // Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
//            cout << "Binary descriptors detected...\n" << endl;
//            if(useBFMatcher)
//            {
//                cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
//                matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
//            }
//            else
//            {
//                // Create Flann LSH index
//                cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
//                cout << "Time creating FLANN LSH index" << endl;

//                // search (nearest neighbor)
//                flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
//            }
//        }
//        else
//        {
//            // assume it is CV_32F
//            cout << "Float descriptors detected...\n" << endl;
//            if(useBFMatcher)
//            {
//                cv::BFMatcher matcher(cv::NORM_L2);
//                matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
//            }
//            else
//            {
//                // Create Flann KDTree index
//                cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

//                // search (nearest neighbor)
//                flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
//            }
//        }

//        // Conversion to CV_32F if needed
//        if(dists.type() == CV_32S)
//        {
//            cv::Mat temp;
//            dists.convertTo(temp, CV_32F);
//            dists = temp;
//        }


//        ////////////////////////////
//        // PROCESS NEAREST NEIGHBOR RESULTS
//        ////////////////////////////
        // Set gui data
//        ObjWidget objWidget(0, objectKeypoints, QMultiMap<int,int>(), cvtCvMat2QImage(objectImg));
//        ObjWidget sceneWidget(0, sceneKeypoints, QMultiMap<int,int>(), cvtCvMat2QImage(sceneImg));

//        // Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
//        float nndrRatio = 0.8f;
//        std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
//        std::vector<int> indexes_1, indexes_2; // Used for homography
//        std::vector<uchar> outlier_mask;  // Used for homography
//        // Check if this descriptor matches with those of the objects
//        if(!useBFMatcher)
//        {
//            for(int i=0; i<objectDescriptors.rows; ++i)
//            {
//                // Apply NNDR
//                //printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
//                if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
//                   dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
//                {
//                    mpts_1.push_back(objectKeypoints.at(i).pt);
//                    indexes_1.push_back(i);

//                    mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
//                    indexes_2.push_back(results.at<int>(i,0));
//                }
//            }
//        }
//        else
//        {
//            for(unsigned int i=0; i<matches.size(); ++i)
//            {
//                // Apply NNDR
//                //printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
//                if(matches.at(i).size() == 2 &&
//                   matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
//                {
//                    mpts_1.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
//                    indexes_1.push_back(matches.at(i).at(0).queryIdx);

//                    mpts_2.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
//                    indexes_2.push_back(matches.at(i).at(0).trainIdx);
//                }
//            }
//        }

//        // FIND HOMOGRAPHY
//        unsigned int minInliers = 8;
//        if(mpts_1.size() >= minInliers)
//        {
//            cv::Mat H = findHomography(mpts_1,
//                    mpts_2,
//                    cv::RANSAC,
//                    1.0,
//                    outlier_mask);
//            printf("Time finding homography\n");
//            int inliers=0, outliers=0;
//            for(unsigned int k=0; k<mpts_1.size();++k)
//            {
//                if(outlier_mask.at(k))
//                {
//                    ++inliers;
//                }
//                else
//                {
//                    ++outliers;
//                }
//            }
//            QTransform hTransform(
//            H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
//            H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
//            H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

//            // GUI : Change color and add homography rectangle
//            QColor color(Qt::green);
//            int alpha = 130;
//            color.setAlpha(alpha);
//            for(unsigned int k=0; k<mpts_1.size();++k)
//            {
//                if(outlier_mask.at(k))
//                {
//                    objWidget.setKptColor(indexes_1.at(k), color);
//                    sceneWidget.setKptColor(indexes_2.at(k), color);
//                }
//                else
//                {
//                    objWidget.setKptColor(indexes_1.at(k), QColor(255,0,0,alpha));
//                    sceneWidget.setKptColor(indexes_2.at(k), QColor(255,0,0,alpha));
//                }
//            }
//            QPen rectPen(color);
//            rectPen.setWidth(4);
//            QGraphicsRectItem * rectItem = new QGraphicsRectItem(objWidget.pixmap().rect());
//            rectItem->setPen(rectPen);
//            rectItem->setTransform(hTransform);
//            sceneWidget.addRect(rectItem);
//            printf("Inliers=%d Outliers=%d\n", inliers, outliers);
//        }
//        else
//        {
//            printf("Not enough matches (%d) for homography...\n", (int)mpts_1.size());
//        }

//        // Wait for gui
//        objWidget.setGraphicsViewMode(false);
//        objWidget.setWindowTitle("Object");
//        if(objWidget.pixmap().width() <= 800)
//        {
//            objWidget.setMinimumSize(objWidget.pixmap().width(), objWidget.pixmap().height());
//        }
//        else
//        {
//            objWidget.setMinimumSize(800, 600);
//            objWidget.setAutoScale(false);
//        }

//        sceneWidget.setGraphicsViewMode(false);
//        sceneWidget.setWindowTitle("Scene");
//        if(sceneWidget.pixmap().width() <= 800)
//        {
//            sceneWidget.setMinimumSize(sceneWidget.pixmap().width(), sceneWidget.pixmap().height());
//        }
//        else
//        {
//            sceneWidget.setMinimumSize(800, 600);
//            sceneWidget.setAutoScale(false);
//        }

//        sceneWidget.show();
//        objWidget.show();

//        int r = app.exec();
//        printf("Closing...\n");

//        return r;
//    }
//    else
//    {
//        printf("Images are not valid!\n");
//    }

//    return 1;
//}

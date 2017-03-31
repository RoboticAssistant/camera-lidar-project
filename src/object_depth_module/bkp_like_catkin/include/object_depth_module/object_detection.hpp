#ifndef OBJECT_DETECTION_HPP
#define OBJECT_DETECTION_HPP

#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace cv;


void createTrackbars();
void morphOps(cv::Mat &thresh);
void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);

#endif // OBJECT_DETECTION_HPP


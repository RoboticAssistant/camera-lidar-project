#include <sstream>
#include <string>
#include <iostream>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_depth_module/object_detection.hpp>

using namespace std;
using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_VAL_MIN = 0;
int H_VAL_MAX = 256;
int S_VAL_MIN = 0;
int S_VAL_MAX = 256;
int V_VAL_MIN = 0;
int V_VAL_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

void on_trackbar( int, void* )
{//This function gets called whenever a
    // trackbar position is changed

}

string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void createTrackbars(){
    //create window for trackbars

    namedWindow(trackbarWindowName,0);
    //create memory to store trackbar name on window
    char TrackbarName[50] = "Temp Name";
    sprintf( TrackbarName, "H_VAL_MIN", H_VAL_MIN);
    sprintf( TrackbarName, "H_VAL_MAX", H_VAL_MAX);
    sprintf( TrackbarName, "S_VAL_MIN", S_VAL_MIN);
    sprintf( TrackbarName, "S_VAL_MAX", S_VAL_MAX);
    sprintf( TrackbarName, "V_VAL_MIN", V_VAL_MIN);
    sprintf( TrackbarName, "V_VAL_MAX", V_VAL_MAX);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar( "H_VAL_MIN", trackbarWindowName, &H_VAL_MIN, H_VAL_MAX, on_trackbar );
    createTrackbar( "H_VAL_MAX", trackbarWindowName, &H_VAL_MAX, H_VAL_MAX, on_trackbar );
    createTrackbar( "S_VAL_MIN", trackbarWindowName, &S_VAL_MIN, S_VAL_MAX, on_trackbar );
    createTrackbar( "S_VAL_MAX", trackbarWindowName, &S_VAL_MAX, S_VAL_MAX, on_trackbar );
    createTrackbar( "V_VAL_MIN", trackbarWindowName, &V_VAL_MIN, V_VAL_MAX, on_trackbar );
    createTrackbar( "V_VAL_MAX", trackbarWindowName, &V_VAL_MAX, V_VAL_MAX, on_trackbar );
}

void drawObject(int x, int y,Mat &frame) {

    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

    putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

void morphOps(Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(6,6));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(5,5));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea){
                    x = moment.m10/area;
                    y = moment.m01/area;
                    objectFound = true;
                    refArea = area;
                }else objectFound = false;
            }
            //let user know you found an object
            if(objectFound ==true){
                putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                //draw object location on screen
                drawObject(x,y,cameraFeed);
            }
        }
        else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}

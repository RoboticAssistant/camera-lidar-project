#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <stdio.h>
#include <opencv_project/convolution_2d.hpp>

using namespace std;
using namespace cv;

// This function does the convolution of the an image with a kernel.
// the kernel is specified with the help of a file name. The file consists of the kernel.
int perform_convolution(Mat srcConvImage, string kernelFileName)
{
    int kernelWindow[3][3];
    int nRows, nColumns, pixelSum;
    int rowCount, colCount;
    Mat dstConvImage;

    // Take the filename and get the kernel from the file
    FILE *filePointer;
    filePointer = fopen(kernelFileName.c_str(), "r");
//    cout << "Starting with Convolution with kernel" << filePointer;

    // Define a kernel
    kernelWindow[0][0] = 1; kernelWindow[0][1] = 0; kernelWindow[0][2] = -1;
    kernelWindow[1][0] = 2; kernelWindow[1][1] = 0; kernelWindow[1][2] = -2;
    kernelWindow[2][0] = 1; kernelWindow[2][1] = 0; kernelWindow[2][2] = -1;

    // Setting up the destination image first. Never do this -> dstConvImage = srcConvImage.
    // The reason for not doing this is that this would be a reference equal and so the changes on dstConvImage
    // would mean changes on srcConvImage. Instead use the Mat constructor.
    dstConvImage = Mat(500, 1000, srcConvImage.type());

    // Peform the convolution using the given kernel window
    for(rowCount = 0; rowCount < dstConvImage.rows; rowCount++)
    {
        for(colCount = 0; colCount < dstConvImage.cols; colCount++)
        {
            // continue of this is a corner.
            if(rowCount < 0  || rowCount == dstConvImage.rows-1
                    || colCount < 0 || colCount == dstConvImage.cols-1)
            continue;

            pixelSum = kernelWindow[1][1]*(int)srcConvImage.at<uchar>(rowCount,colCount) + kernelWindow[0][0]*(int)srcConvImage.at<uchar>(rowCount-1,colCount-1) +
                    kernelWindow[0][1]*(int)srcConvImage.at<uchar>(rowCount-1,colCount) + kernelWindow[0][2]*(int)srcConvImage.at<uchar>(rowCount-1,colCount+1) +
                    kernelWindow[1][0]*(int)srcConvImage.at<uchar>(rowCount,colCount-1) + kernelWindow[1][2]*(int)srcConvImage.at<uchar>(rowCount,colCount+1) +
                    kernelWindow[2][0]*(int)srcConvImage.at<uchar>(rowCount+1,colCount-1) + kernelWindow[2][1]*(int)srcConvImage.at<uchar>(rowCount+1,colCount) +
                    kernelWindow[2][2]*(int)srcConvImage.at<uchar>(rowCount+1,colCount+1);

            dstConvImage.at<uchar>(rowCount,colCount) = pixelSum;
        }
    }

    namedWindow("2D Convoluted Image", WINDOW_AUTOSIZE);
    imshow("2D Convoluted Image", dstConvImage);

    // Returning success
    return 0;
}

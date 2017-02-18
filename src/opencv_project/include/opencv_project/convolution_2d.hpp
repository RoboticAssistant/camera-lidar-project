#ifndef CONVOLUTION_2D_HPP
#define CONVOLUTION_2D_HPP

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int perform_convolution(Mat inputConvImage, string kernelFileName);

#endif // CONVOLUTION_2D_HPP


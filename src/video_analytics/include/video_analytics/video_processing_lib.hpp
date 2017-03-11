#ifndef VIDEO_PROCESSING_LIB_HPP
#define VIDEO_PROCESSING_LIB_HPP

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void calculate_mean(int ***features, float **mean_result, int number_of_groups, int number_of_samples, int feature_vector_dimension);
int calculate_covariance(int ***features, int number_of_groups, int number_of_samples, int feature_vector_dimension);

int perform_convolution(Mat srcConvImage, string kernelFileName, string kernelPattern);

#endif // VIDEO_PROCESSING_LIB_HPP


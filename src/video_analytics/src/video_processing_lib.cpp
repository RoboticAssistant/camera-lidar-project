#include <iostream>
#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <stdio.h>
#include <video_analytics/video_processing_lib.hpp>
#include <math.h>

#define PIZZA_PI 3.14159265

using namespace std;
using namespace cv;


// Crude way of calculating the mean
void calculate_mean(int ***features, float **mean_result, int number_of_groups, int number_of_samples, int feature_vector_dimension)
{
    // Initializing the mean_result to 0
    for(int counter_x = 0; counter_x < number_of_groups; counter_x++)
    {
        for(int counter_y = 0; counter_y < feature_vector_dimension; counter_y++)
        {
            mean_result[counter_x][counter_y] = 0;
        }
    }

    for(int i = 0; i < number_of_groups; i++)
    {
        for(int j = 0; j < feature_vector_dimension; j++)
        {
            for(int k = 0; k < number_of_samples; k++)
            {
                mean_result[i][j] = mean_result[i][j] + features[i][k][j];
            }
            mean_result[i][j] = (mean_result[i][j])/number_of_samples;
        }
    }

    // Display the mean
    for(int i = 0; i < number_of_groups; i++)
    {
        cout << "Mean for Group " << i+1 << " features." << endl;
        cout << "[";
        for(int j = 0; j < feature_vector_dimension; j++)
        {
            cout << mean_result[i][j] << " ";
        }
        cout << "]";
        cout << endl;
    }
}

// int m_one_size[] -> m_one_size[0] = number of rows, m_one_size[1] = number of columns
int matrix_multiplication(float *mat_one, int m_one_size[], float **mat_two, int m_two_size[], float **mat_result, int *r_size)
{
    int sum = 0;
    if(m_one_size[1] != m_two_size[0]) {
        return -1;
    }

    // Filling the result size
    r_size[0] = m_one_size[0]; r_size[1] = m_two_size[1];
    mat_result = (float **)malloc(r_size[0] * sizeof(float*));

    for(int i = 0; i < r_size[0]; i++) {
        mat_result[i] = (float*)malloc(r_size[1] * sizeof(float));
    }

    for (int c = 0; c < m_one_size[0]; c++) {
        for (int d = 0; d < m_two_size[1]; d++) {
            for (int k = 0; k < m_two_size[0]; k++) {
                //sum = sum + (*(mat_one + k)) * (*(mat_two + k)));
            }

            mat_result[c][d] = sum;
            sum = 0;
        }
    }

    return 0;
}

int mat_inverse(float **mat, int *mat_size, float *mat_inverse, int *mat_inverse_size)
{
    mat_inverse_size[0] = mat_size[1];
    mat_inverse_size[1] = mat_size[0];

    for(int i = 0; i < mat_inverse_size[0]; i++)
    {
        for(int j = 0; j < mat_inverse_size[1]; j++)
        {
            *(mat_inverse + j) = *(*(mat + j)+ i);           // fix this later. Now just for 1D matrix
        }
    }
}

// This array prints the input double array
int print_array(float **print_array, int arr_size_r, int arr_size_c)
{
    for(int i = 0; i < arr_size_r; i++) {
        for(int j = 0; j < arr_size_c; j++) {
            cout << *(*(print_array+i)+j) << " ";
        }
        cout << endl;
    }
}

// This calculates the covariance and returns the int data
int calculate_covariance(int ***features, int number_of_groups, int number_of_samples, int feature_vector_dimension)
{
    float **mean_result, *mean_result_invert, **mean_multiply;
    int mat_size[] = {1, 3};
    int mat_inverse_size[] = {3, 1};
    int mean_mul_size[2];

    mean_result = (float **)malloc(number_of_groups * sizeof(float *));
    for(int counter = 0; counter < number_of_groups; counter++)
    {
        mean_result[counter] = (float *)malloc(feature_vector_dimension * sizeof(float));
    }

    calculate_mean(features, mean_result, number_of_groups, number_of_samples, feature_vector_dimension);

    // Memory for matrix invert
    mean_result_invert = (float *)malloc(feature_vector_dimension * sizeof(float));
    mat_inverse((float **)(mean_result + 0), mat_size, mean_result_invert, mat_inverse_size);

	//Now calculating - This calculation is for covariance
    if(matrix_multiplication(mean_result_invert, mat_inverse_size,
                             (float **)(mean_result+0), mat_size,
                             mean_multiply, mean_mul_size) != 0) {
        cout << "Matrix multiply unsuccessfull" << endl;
    }

//    print_array(mean_multiply, mean_mul_size[0], mean_mul_size[1]);

    return 0;
}


// This function does the convolution of the an image with a kernel.
// the kernel is specified with the help of a file name. The file consists of the kernel.
int perform_convolution(Mat srcConvImage, string kernelFileName, string kernelPattern)
{
    int kernelWindow[3][3];
    int nRows, nColumns, pixelSum;
    int rowCount, colCount;
    Mat dstConvImage;

    // Take the filename and get the kernel from the file
    FILE *filePointer;
    filePointer = fopen(kernelFileName.c_str(), "r");
//    cout << "Starting with Convolution with kernel" << filePointer;


    if(!kernelPattern.compare("edge_detector"))
    {
//        kernelWindow = (int **) malloc(9*sizeof(int));
        cout << "Applying edge detector kernel" << endl;
        kernelWindow[0][0] = 1; kernelWindow[0][1] = 0; kernelWindow[0][2] = -1;
        kernelWindow[1][0] = 1; kernelWindow[1][1] = 0; kernelWindow[1][2] = -1;
        kernelWindow[2][0] = 1; kernelWindow[2][1] = 0; kernelWindow[2][2] = -1;
    }
    else if(!kernelPattern.compare("gaussian"))
    {
//        kernelWindow = (int **) malloc(25*sizeof(int));
        cout << "Applying Gaussian kernel" << endl;
        // Define a kernel
        kernelWindow[0][0] = 1; kernelWindow[0][1] = 0; kernelWindow[0][2] = -1;
        kernelWindow[1][0] = 1; kernelWindow[1][1] = 0; kernelWindow[1][2] = -1;
        kernelWindow[2][0] = 1; kernelWindow[2][1] = 0; kernelWindow[2][2] = -1;
        kernelWindow[3][0] = 1; kernelWindow[3][1] = 0; kernelWindow[3][2] = -1;
        kernelWindow[4][0] = 1; kernelWindow[4][1] = 0; kernelWindow[4][2] = -1;
    }
    else if(!kernelPattern.compare("log"))
    {
//        kernelWindow = (int **) malloc(9*sizeof(int));
        cout << "Applying LoG kernel" << endl;
        // Define a kernel
        kernelWindow[0][0] = 0; kernelWindow[0][1] = -1; kernelWindow[0][2] = 0;
        kernelWindow[1][0] = -1; kernelWindow[1][1] = 4; kernelWindow[1][2] = -1;
        kernelWindow[2][0] = 0; kernelWindow[2][1] = -1; kernelWindow[2][2] = 0;
    }


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

            if(!kernelPattern.compare("gaussian"))
            {
                pixelSum = kernelWindow[2][2]*(int)srcConvImage.at<uchar>(rowCount,colCount) + kernelWindow[1][1]*(int)srcConvImage.at<uchar>(rowCount-1,colCount-1) +
                        kernelWindow[0][2]*(int)srcConvImage.at<uchar>(rowCount-1,colCount) + kernelWindow[0][3]*(int)srcConvImage.at<uchar>(rowCount-1,colCount+1) +
                        kernelWindow[2][1]*(int)srcConvImage.at<uchar>(rowCount,colCount-1) + kernelWindow[2][3]*(int)srcConvImage.at<uchar>(rowCount,colCount+1) +
                        kernelWindow[3][1]*(int)srcConvImage.at<uchar>(rowCount+1,colCount-1) + kernelWindow[3][2]*(int)srcConvImage.at<uchar>(rowCount+1,colCount) +
                        kernelWindow[3][3]*(int)srcConvImage.at<uchar>(rowCount+1,colCount+1) + kernelWindow[0][0]*(int)srcConvImage.at<uchar>(rowCount-2,colCount-2) +
                        kernelWindow[0][1]*(int)srcConvImage.at<uchar>(rowCount-2,colCount-1) + kernelWindow[0][2]*(int)srcConvImage.at<uchar>(rowCount-2,colCount) +
                        kernelWindow[0][3]*(int)srcConvImage.at<uchar>(rowCount-2,colCount+1) + kernelWindow[0][4]*(int)srcConvImage.at<uchar>(rowCount-2,colCount+2) +
                        kernelWindow[1][0]*(int)srcConvImage.at<uchar>(rowCount-1,colCount-2) + kernelWindow[2][0]*(int)srcConvImage.at<uchar>(rowCount,colCount-2) +
                        kernelWindow[3][0]*(int)srcConvImage.at<uchar>(rowCount+1,colCount-2) + kernelWindow[1][4]*(int)srcConvImage.at<uchar>(rowCount-1,colCount+2) +
                        kernelWindow[2][4]*(int)srcConvImage.at<uchar>(rowCount,colCount+2) + kernelWindow[3][4]*(int)srcConvImage.at<uchar>(rowCount+1,colCount+2) +
                        kernelWindow[4][0]*(int)srcConvImage.at<uchar>(rowCount+2,colCount-2) + kernelWindow[4][1]*(int)srcConvImage.at<uchar>(rowCount+2,colCount-1) +
                        kernelWindow[4][2]*(int)srcConvImage.at<uchar>(rowCount+2,colCount) + kernelWindow[4][3]*(int)srcConvImage.at<uchar>(rowCount+2,colCount+1) +
                        kernelWindow[4][4]*(int)srcConvImage.at<uchar>(rowCount+2,colCount+2);
            }
            else
            {
                pixelSum = kernelWindow[1][1]*(int)srcConvImage.at<uchar>(rowCount,colCount) + kernelWindow[0][0]*(int)srcConvImage.at<uchar>(rowCount-1,colCount-1) +
                        kernelWindow[0][1]*(int)srcConvImage.at<uchar>(rowCount-1,colCount) + kernelWindow[0][2]*(int)srcConvImage.at<uchar>(rowCount-1,colCount+1) +
                        kernelWindow[1][0]*(int)srcConvImage.at<uchar>(rowCount,colCount-1) + kernelWindow[1][2]*(int)srcConvImage.at<uchar>(rowCount,colCount+1) +
                        kernelWindow[2][0]*(int)srcConvImage.at<uchar>(rowCount+1,colCount-1) + kernelWindow[2][1]*(int)srcConvImage.at<uchar>(rowCount+1,colCount) +
                        kernelWindow[2][2]*(int)srcConvImage.at<uchar>(rowCount+1,colCount+1);
            }

            dstConvImage.at<uchar>(rowCount,colCount) = pixelSum;
        }
    }

    namedWindow("2D Convoluted Image", WINDOW_AUTOSIZE);
    imshow("2D Convoluted Image", dstConvImage);

    // Returning success
    return 0;
}

// Function used for getting the perimeter of the image.
// to use opencv internal functions
// Return the perimeter and area of the image.
int find_contour_features(Mat &input_image, int &perimeter, double &area, vector<vector<Point> > &contours)
{
	Mat contour_display_mat = Mat::zeros(input_image.rows, input_image.cols, CV_8UC1);
	int threshold = 100;

	/// Detect edges using canny
	Canny( input_image, input_image, threshold, threshold*2, 3 );
	imshow("Canny Output", input_image);

	// Finding different contours
	vector<Vec4i> hierarchy;

	findContours(input_image, contours, hierarchy,
				  CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	for(int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
	{
		Scalar color(rand()&255, rand()&255, rand()&255);
		drawContours(contour_display_mat, contours, idx, color, CV_FILLED, 8, hierarchy);
	}

	for(int counter = 0; counter < contours.size(); counter++)
	{
		perimeter = perimeter + arcLength(contours[counter], true);
		area  = area + contourArea(contours[counter]);
	}

	namedWindow( "Components", 1 );
	imshow( "Components", contour_display_mat);

	return 0;
}

// This function is for getting the x bar and y bar features of the image
int get_x_y_bar(Mat &binaryImage, double &x_bar, double &y_bar, int area)
{
	int row_counter, col_counter;

	for(row_counter = 0; row_counter < binaryImage.rows; row_counter++) {
		for(col_counter = 0; col_counter < binaryImage.cols; col_counter++) {
			x_bar = x_bar + (row_counter * (int)binaryImage.at<uchar>(row_counter, col_counter));
			y_bar = y_bar + (col_counter * (int)binaryImage.at<uchar>(row_counter, col_counter));
		}
	}
	x_bar = x_bar/area;
	y_bar = y_bar/area;

	return 0;
}

// Get the elongation axis and the theta value of the image
int get_theta(Mat &binaryImage, double x_bar, double y_bar, double &theta)
{
	double a = 0, b = 0, c = 0;
	int rows = binaryImage.rows, row_counter = 0, cols = binaryImage.cols, col_counter = 0;
	double angle = 0;

	// Calculate a, b, c
	for(row_counter = 0; row_counter < rows; row_counter++) {
		for(col_counter = 0; col_counter < cols; col_counter++) {
			a = a + ((row_counter - x_bar) *
					 (int)binaryImage.at<uchar>(row_counter, col_counter));
			b = b + (2 * (row_counter - x_bar) *
					 (col_counter - y_bar) *
					 (int)binaryImage.at<uchar>(row_counter, col_counter));
			c = c + ((col_counter - y_bar) *
					 (int)binaryImage.at<uchar>(row_counter, col_counter));
		}
	}

	// Calculate value of theta using a, b, c
	angle = (atan((b/(a-c))))/2;
	theta = angle * 180/PIZZA_PI;
}

int get_moments(Mat &binary_image, vector<double> &raw_moments, vector<double> &hu_moments)
{
	int number_of_moments = 7;
	cv::Moments mom = cv::moments(binary_image, true);

	//Pushing in raw centralized moments
	raw_moments.push_back(mom.mu20);
	raw_moments.push_back(mom.mu11);
	raw_moments.push_back(mom.mu02);
	raw_moments.push_back(mom.mu30);
	raw_moments.push_back(mom.mu21);
	raw_moments.push_back(mom.mu12);
	raw_moments.push_back(mom.mu03);

	cv::HuMoments(mom, hu_moments);
}

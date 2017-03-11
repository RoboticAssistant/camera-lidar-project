#include <iostream>
#include <video_analytics/video_processing_lib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    Mat input_image, grayImage, binaryImage;
    input_image = imread("/home/ubuntu/Documents/CMPE297/stop1.jpg");
    string activity = argv[1];
    bool is_feature = false, is_convolution = false;
    string kernel_file;
    string kernel_pattern;
    int feature_vector_dimension, number_of_samples, number_of_groups;
    float **mean_result;

    // Complex description of the feature array
    int ***X;

    if(!strcmp(activity.c_str(), "feature"))
    {
        is_feature = true;
        feature_vector_dimension = 3;
        number_of_samples = 4;
        number_of_groups = 2;
    }
    else if(!strcmp(activity.c_str(), "convolution"))
    {
        is_convolution = true;
        // Checking for the input argument image path
        if(argc != 4)
        {
            cout << "Error in Usage\nUsage: <exe> <activity>[feature, convolution] <image_name> <kernel_file_name>" << endl;
            return -1;
        }
        kernel_file = argv[2];
        kernel_pattern = argv[3];

    }
    else
    {
        cout << "Error in Usage\nUsage: <exe> <activity>[feature, convolution]" << endl;
    }

    if(is_convolution)
    {
        // Checking the image data. If there is any data-OK. If not, error.
        if(!input_image.data)
        {
            cout << "Could not open this image. Please try with any other Image";
            return -1;
        }

        // Showing the input image. *This is in order to keep an order of all the images
        namedWindow("Input Window", WINDOW_AUTOSIZE);
        imshow("Input Window", input_image);

        // Converting that image into a grayscale image.
        cvtColor(input_image, grayImage, COLOR_RGB2GRAY);

        // Showing the grayscale image. *This is in order to keep an order of all the images
        namedWindow("Grayscale Window", WINDOW_AUTOSIZE);
        imshow("Grayscale Window", grayImage);

        // Coverting a grayscale image into a binary image.
        threshold(grayImage, binaryImage, 100, 255, CV_THRESH_BINARY);

        // Showing the binary image.
        namedWindow("Binary Image", WINDOW_AUTOSIZE);
        imshow("Binary Image", binaryImage);

        perform_convolution(binaryImage, kernel_file, kernel_pattern);
    }
    else if(is_feature)
    {
        // Allocating memory for the groups (i will be always number of groups
        /// TODO: Handle memory leak later
        X = (int ***)malloc(number_of_groups * sizeof(int **));

        mean_result = (float **)malloc(number_of_groups * sizeof(float *));

        for(int counter = 0; counter < number_of_groups; counter++)
        {
            mean_result[counter] = (float *)malloc(feature_vector_dimension * sizeof(float));
        }

        // Allocating memory for the number of samples (j will always be number of samples)
        /// TODO: Handle memory leak later
        for(int j = 0; j < number_of_groups; j++)
        {
            X[j] = (int **)malloc(number_of_samples * sizeof(int *));

            //Allocating memory for each array[i] with the number feature vector dimension
            // (k should always be feature vector dimension
            for(int k = 0; k < number_of_samples; k++)
            {
                X[j][k] = (int *)malloc(feature_vector_dimension * sizeof(int));
            }
        }

        /// TODO: Remove this after the feature vectors are filled
        // Temporary: Filling up group 1
        X[0][0][0] = 0; X[0][0][1] = 0; X[0][0][2] = 0;
        X[0][1][0] = 1; X[0][1][1] = 0; X[0][1][2] = 0;
        X[0][2][0] = 1; X[0][2][1] = 0; X[0][2][2] = 1;
        X[0][3][0] = 1; X[0][3][1] = 1; X[0][3][2] = 0;

        // Temporary: Filling up group 2
        X[1][0][0] = 0; X[1][0][1] = 0; X[1][0][2] = 1;
        X[1][1][0] = 0; X[1][1][1] = 1; X[1][1][2] = 0;
        X[1][2][0] = 0; X[1][2][1] = 1; X[1][2][2] = 1;
        X[1][3][0] = 1; X[1][3][1] = 1; X[1][3][2] = 1;

        // Calculation of mean ahead of this
        calculate_mean(X, mean_result, number_of_groups, number_of_samples, feature_vector_dimension);

        // Calculation of covariance
        calculate_covariance(X, number_of_groups, number_of_samples, feature_vector_dimension);
    }

    // Freeing everything
    free(X);
    X = NULL;
    free(mean_result);
    mean_result = NULL;

    waitKey(0);
    return 0;
}

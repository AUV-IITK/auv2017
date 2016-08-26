// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <sstream>
std_msgs::Float32MultiArray array;
cv::Mat image_input;
int main(int argc, char* argv[])
{
    int height, width, step;  // parameters of the image we are working on
    int t1min = 0, t1max = 255, t2min = 0 , t2max = 255, t3min = 0, t3max = 255;  // other variables used
    ros::init(argc, argv, "image");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("parameters", 1000);
    ros::Rate loop_rate(10);
    cv::Mat circles;
    // Load the previous values of the threshold if they exist
// *****  UNCOMMENT BELOW PART TO TAKE FEED FROM CAMERA AND COMMENT OUT PART AFTER THAT***********\\
//    int camno = (**(argv+1)-'0');
//    VideoCapture cap(camno);
//   image_input = imread(argv[0], CV_LOAD_IMAGE_COLOR);
    image_input = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
//**************************************************************************\\
// VideoCapture cap(argv[1]);
//***************************************************************************\\
// Create a window in which the captured images will be presented
    cvNamedWindow("After Color Filtering", CV_WINDOW_NORMAL);
    cvNamedWindow("F1", CV_WINDOW_NORMAL);
    cvNamedWindow("RealPic", CV_WINDOW_NORMAL);
    cvNamedWindow("F2", CV_WINDOW_NORMAL);
    cvNamedWindow("F3", CV_WINDOW_NORMAL);
    /// Create Trackbars
    char TrackbarName1[50] = "t1min";
    char TrackbarName2[50] = "t1max";
    char TrackbarName3[50] = "t2min";
    char TrackbarName4[50] = "t2max";
    char TrackbarName5[50] = "t3min";
    char TrackbarName6[50] = "t3max";
    cvCreateTrackbar(TrackbarName1, "F1", &t1min, 260 , NULL);
    cvCreateTrackbar(TrackbarName2, "F1", &t1max, 260,  NULL);
    cvCreateTrackbar(TrackbarName3, "F2", &t2min, 260 , NULL);
    cvCreateTrackbar(TrackbarName4, "F2", &t2max, 260,  NULL);
    cvCreateTrackbar(TrackbarName5, "F3", &t3min, 260 , NULL);
    cvCreateTrackbar(TrackbarName6, "F3", &t3max, 260,  NULL);
    // Load threshold from the slider bars in these 2 parameters
    // capture size -
    CvSize size = cvSize(width, height);
    cv::Mat  hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;
    while (ros::ok())
    {
        // Get one frame
//        if( !cap.isOpened() )
//        {
//          fprintf( stderr, "ERROR: frame is null...\n" );
//          getchar();
//          break;
//        }
        cv::Mat frame;
        image_input.copyTo(frame);
        if (frame.empty())
        {
            printf("No Frame");
            continue;
        }
        // get the image data
        height    = frame.rows;
        width     = frame.cols;
        step      = frame.step;
        cv::imshow("RealPic", frame);
        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvtColor(frame, hsv_frame, CV_BGR2HSV);
        cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
        cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
        // Filter out colors which are out of range.
        cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);
        // Split image into its 3 one dimensional images
        cv::Mat thresholded_hsv[3];
        cv::split(hsv_frame, thresholded_hsv);
        // Filter out colors which are out of range.
        cv::inRange(thresholded_hsv[0], cv::Scalar(t1min, 0, 0, 0) , cv::Scalar(t1max, 0, 0, 0) , thresholded_hsv[0]);
        cv::inRange(thresholded_hsv[1], cv::Scalar(t2min, 0, 0, 0) , cv::Scalar(t2max, 0, 0, 0) , thresholded_hsv[1]);
        cv::inRange(thresholded_hsv[2], cv::Scalar(t3min, 0, 0, 0) , cv::Scalar(t3max, 0, 0, 0) , thresholded_hsv[2]);
        cv::imshow("F1", thresholded_hsv[0]);  // individual filters
        cv::imshow("F2", thresholded_hsv[1]);
        cv::imshow("F3", thresholded_hsv[2]);
        // Memory for hough circles
        cv::imshow("After Color Filtering", thresholded);  // The stream after color filtering
        array.data.push_back(t1min);
        array.data.push_back(t1max);
        array.data.push_back(t2min);
        array.data.push_back(t2max);
        array.data.push_back(t3min);
        array.data.push_back(t3max);
        pub.publish(array);
        ros::spinOnce();
        // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        // remove higher bits using AND operator
        if ((cvWaitKey(10) & 255) == 27 ) break;
    }
    return 0;
  }

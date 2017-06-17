// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <calibrate/calibrateConfig.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int t1min, t1max, t2min, t2max, t3min, t3max;  // Default Params

cv::Mat frame;
cv::Mat newframe;
int count = 0, count_avg = 0, x = -1;

void callback(calibrate::calibrateConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("calibrate_Reconfigure Request:New params : %d %d %d %d %d %d", t1min, t1max, t2min, t2max, t3min, t3max);
}

int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on

  ros::init(argc, argv, "calibrate");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/calibrate", 1000);
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<calibrate::calibrateConfig> server;
  dynamic_reconfigure::Server<calibrate::calibrateConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  n.getParam("calibrate_detection/t1max", t1max);
  n.getParam("calibrate_detection/t1min", t1min);
  n.getParam("calibrate_detection/t2max", t2max);
  n.getParam("calibrate_detection/t2min", t2min);
  n.getParam("calibrate_detection/t3max", t3max);
  n.getParam("calibrate_detection/t3min", t3min);

  calibrate::calibrateConfig config;
  config.t1min_param = t1min;
  config.t1max_param = t1max;
  config.t2min_param = t2min;
  config.t2max_param = t2max;
  config.t3min_param = t3min;
  config.t3max_param = t3max;
  callback(config, 0);

  cvNamedWindow("Contours", CV_WINDOW_NORMAL);
  cvNamedWindow("RealPic", CV_WINDOW_NORMAL);
  cvNamedWindow("After Color Filtering", CV_WINDOW_NORMAL);

  cvNamedWindow("F2", CV_WINDOW_NORMAL);
  cvNamedWindow("F3", CV_WINDOW_NORMAL);
  cvNamedWindow("F1", CV_WINDOW_NORMAL);

  if (argc != 2)
    printf("Please write the name of the file you want to load\n");
  cv::VideoCapture cap(argv[1]);

  CvSize size = cvSize(width, height);
  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;  // image converted to HSV plane

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    if (x != 32)
      cap >> frame;

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
    cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
    // Filter out colors which are out of range.
    cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);
    // Split image into its 3 one dimensional images
    cv::Mat thresholded_hsv[3];
    cv::split(hsv_frame, thresholded_hsv);

    // Filter out colors which are out of range.
    cv::inRange(thresholded_hsv[0], cv::Scalar(t1min, 0, 0, 0), cv::Scalar(t1max, 0, 0, 0), thresholded_hsv[0]);
    cv::inRange(thresholded_hsv[1], cv::Scalar(t2min, 0, 0, 0), cv::Scalar(t2max, 0, 0, 0), thresholded_hsv[1]);
    cv::inRange(thresholded_hsv[2], cv::Scalar(t3min, 0, 0, 0), cv::Scalar(t3max, 0, 0, 0), thresholded_hsv[2]);
    cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    cv::imshow("After Color Filtering", thresholded);  // The stream after color filtering

    cv::imshow("F1", thresholded_hsv[0]);  // individual filters
    cv::imshow("F2", thresholded_hsv[1]);
    cv::imshow("F3", thresholded_hsv[2]);

    std::vector<std::vector<cv::Point> > contours;
    cv::Mat thresholded_Mat = thresholded;
    findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
    double largest_area = 0, largest_contour_index = 0;

    for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
    {
      double a = contourArea(contours[i], false);  //  Find the area of contour
      if (a > largest_area)
      {
        largest_area = a;
        largest_contour_index = i;  // Store the index of largest contour
      }
    }
    cv::Scalar color(255, 255, 255);
    std::vector<cv::Vec4i> hierarchy;

    cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
    drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);
    cv::imshow("Contours", Drawing);
    cv::imshow("RealPic", frame);

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if ((cvWaitKey(10) & 255) == 32)
    {
      if (x == 32)
        x = -1;
      else
        x = 32;
    }
    if (x == 32)
      ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());
    ros::spinOnce();
  }
  return 0;
}

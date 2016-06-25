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
#include <image_transport/image_transport.h>
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "std_msgs/Float64MultiArray.h"

// bool IP = true;
bool IP = false;

void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

cv::Mat frame;
cv::Mat newframe;
int count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    count++;
    // imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;

    ///////////////////////////// DO NOT REMOVE THIS, IT COULD BE INGERIOUS TO HEALTH /////////////////////
    newframe.copyTo(frame);
    cv::imshow("newframe", newframe);
    ////////////////////////// FATAL ///////////////////////////////////////////////////
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on
  int i, j, k, t1min = 0, t1max = 9, t2min = 104, t2max = 260, t3min = 185, t3max = 260;  // other variables used

  cv::VideoWriter output_cap(argv[2], CV_FOURCC('D', 'I', 'V', 'X'), 25, cv::Size(640, 480));
  ros::init(argc, argv, "buoy_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("balls", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("/balls_off", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("camera/image", 1, imageCallback);

  char TrackbarName1[50] = "t1min";
  char TrackbarName2[50] = "t1max";
  char TrackbarName3[50] = "t2min";
  char TrackbarName4[50] = "t2max";
  char TrackbarName5[50] = "t3min";
  char TrackbarName6[50] = "t3max";

  cvCreateTrackbar(TrackbarName1, "F1", &t1min, 260, NULL);
  cvCreateTrackbar(TrackbarName2, "F1", &t1max, 260, NULL);

  cvCreateTrackbar(TrackbarName3, "F2", &t2min, 260, NULL);
  cvCreateTrackbar(TrackbarName4, "F2", &t2max, 260, NULL);

  cvCreateTrackbar(TrackbarName5, "F3", &t3min, 260, NULL);
  cvCreateTrackbar(TrackbarName6, "F3", &t3max, 260, NULL);

  // capture size -
  CvSize size = cvSize(width, height);

  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;  // image converted to HSV plane

  std_msgs::Float64MultiArray array;
  while (1)
  {
    if (!IP)
    {
      loop_rate.sleep();

      if (frame.empty())
      {
        std::cout << "empty frame \n";
        ros::spinOnce();
        continue;
      }

      // cv::imshow("frame",frame);

      // get the image data
      height = frame.rows;
      width = frame.cols;
      step = frame.step;
      // frame = cvQueryFrame( capture );

      // cv::imshow("RealPic", frame);
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
      // cv::imshow("F1", thresholded_hsv[0]);  // individual filters
      // cv::imshow("F2", thresholded_hsv[1]);
      // cv::imshow("F3", thresholded_hsv[2]);

      // cv::imshow("After Color Filtering", thresholded);  // The stream after color filtering
      // hough detector works better with some smoothing of the image
      cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);

      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      cv::findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      // cv::imshow("Contours", thresholded_Mat);  // The stream after color filterin
      if (contours.empty())
      {
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        // cv::imshow( "circle", circles ); // Original stream with detected ball overlay
        pub.publish(array);
        ros::spinOnce();
        // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        // remove higher bits using AND operator
        if ((cvWaitKey(10) & 255) == 27)
          break;
        continue;
      }

      for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
      {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
          largest_area = a;
          largest_contour_index = i;  // Store the index of largest contour
        }
      }
      // Convex HULL
      std::vector<std::vector<cv::Point> > hull(contours.size());
      convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      output_cap.write(frame);

      std::vector<cv::Point2f> center(1);
      std::vector<float> radius(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);

      cv::Mat circles = frame;
      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;
      float distance;
      float *p;                                     // array to publish
      distance = pow(radius[0] / 7526.5, -.92678);  // function found using experiment

      circle(circles, center[0], radius[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);         // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);            // center of screen
      array.data.push_back(radius[0]);                                        // publish radius
      array.data.push_back((320 - center[0].x));
      array.data.push_back((240 - center[0].y));
      array.data.push_back(distance);
      if (((320 - center[0].x > -5) && (320 - center[0].x < 5)) &&
          ((240 - center[0].y > -5) && (240 - center[0].y) < 5))
        array.data.push_back(1);
      else
        array.data.push_back(0);  // telling we are in line of center of ball

      pub.publish(array);
      // imshow("F1", thresholded1);  // individual filters
      // imshow("F2", thresholded2);
      // imshow("F3", thresholded3);
      // cv::imshow("circle", circles);  // Original stream with detected ball overlay

      ros::spinOnce();
      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
        break;
    }
    else
    {
      std::cout << "waiting\n";
      ros::spinOnce();
    }
  }
  output_cap.release();
  return 0;
}

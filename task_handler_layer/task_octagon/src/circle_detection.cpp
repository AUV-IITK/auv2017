// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <task_octagon/octagonConfig.h>
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
#include <string>
#include "std_msgs/Float64MultiArray.h"

bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;  // Default Params

void callback(task_octagon::octagonConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Reconfigure Request : New parameters : %d %d %d %d %d %d", t1min, t1max, t2min, t2max, t3min, t3max);
}

cv::Mat frame;
cv::Mat newframe;
int count = 0, count_avg = 0;

float mod(float x, float y)
{
  if (x - y > 0) return x;
  else return y;
}
void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    count++;
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    cvNamedWindow("newframe", CV_WINDOW_NORMAL);
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
  std::string Video_Name = "Random_Video";
  if (argc >= 2)
    flag = true;
  if (argc == 3)
  {
    video = true;
    std::string avi = ".avi";
    Video_Name = (argv[2]) + avi;
  }

  cv::VideoWriter output_cap(Video_Name, CV_FOURCC('D', 'I', 'V', 'X'), 9, cv::Size(640, 480));

  ros::init(argc, argv, "circle_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/octagon", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("circle_detection_switch", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);
  int t1minParam, t1maxParam, t2minParam, t2maxParam, t3minParam, t3maxParam;

  n.getParam("circle_detection/t1maxParam", t1maxParam);
  n.getParam("circle_detection/t1minParam", t1minParam);
  n.getParam("circle_detection/t2maxParam", t2maxParam);
  n.getParam("circle_detection/t2minParam", t2minParam);
  n.getParam("circle_detection/t3maxParam", t3maxParam);
  n.getParam("circle_detection/t3minParam", t3minParam);

  t1min = t1minParam;
  t1max = t1maxParam;
  t2min = t2minParam;
  t2max = t2maxParam;
  t3min = t3minParam;
  t3max = t3maxParam;

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

  cvNamedWindow("Contours", CV_WINDOW_NORMAL);
  cvNamedWindow("circle", CV_WINDOW_NORMAL);
  cvNamedWindow("After Color Filtering", CV_WINDOW_NORMAL);

  dynamic_reconfigure::Server<task_octagon::octagonConfig> server;
  dynamic_reconfigure::Server<task_octagon::octagonConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  if (flag)
  {
    cvNamedWindow("F1", CV_WINDOW_NORMAL);
    cvNamedWindow("F2", CV_WINDOW_NORMAL);
    cvNamedWindow("F3", CV_WINDOW_NORMAL);
  }

  // capture size -
  CvSize size = cvSize(width, height);

  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;  // image converted to HSV plane
  float r[5];

  for (int i; i++; i < 5)
    r[i] = 0;

  while (1)
  {
    printf("137\n");
    std_msgs::Float64MultiArray array;
printf("139\n");
    if (frame.empty())
    {
      std::cout << "empty frame \n";
      ros::spinOnce();
      continue;
    }

    if (video)
      output_cap.write(frame);

    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;
printf("154\n" );
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
    printf("169\n");
    cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    cv::imshow("After Color Filtering", thresholded);  // The stream after color filtering

    if (flag)
    {
      cv::imshow("F1", thresholded_hsv[0]);              // individual filters
      cv::imshow("F2", thresholded_hsv[1]);
      cv::imshow("F3", thresholded_hsv[2]);
    }

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if ((!IP))
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat;
      thresholded.copyTo(thresholded_Mat);
      cv::findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
printf("190\n");
      if (contours.empty())
      {
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);

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

      std::vector<cv::Point2f> center(1);
      std::vector<float> radius(1);
      std::vector<cv::Point2f> center_ideal(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);

      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;
printf("228\n");
printf("245\n");
      cv::Mat circles = frame;
      circle(circles, center[0], radius[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);         // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);            // center of screen

     
      array.data.push_back(r[0]);                                        // publish radius
      array.data.push_back((320 - center_ideal[0].x));
      array.data.push_back(-(240 - center_ideal[0].y));

      cv::imshow("circle", circles);            // Original stream with detected ball overlay
      cv::imshow("Contours", thresholded_Mat);  // The stream after color filtering
      pub.publish(array);
printf("259\n");
      ros::spinOnce();
printf("262\n");      
      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
      {  
        printf("266\n");
        break;
      }  
      printf("270\n");
      printf("271\n");
      printf("272\n");
    }
    else
    {
      printf("271\n");
      std::cout << "waiting\n";
      ros::spinOnce();
    }
    printf("278\n");
  }
  printf("280\n");
  output_cap.release();
  return 0;
}

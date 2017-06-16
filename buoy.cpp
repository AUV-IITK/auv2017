// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <task_buoy/buoyConfig.h>
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
#include <algorithm>

bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;  // Default Params

cv::Mat dst_array, frame, src;
int count = 0, count_avg = 0, x = -1;
///////////////////////// dynamic-reconfigure//////////////////////////////////////////////
void callback(task_buoy::buoyConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d", t1min, t1max, t2min, t2max, t3min, t3max);
}
/////////////////////////////// lineDetected-listeners///////////////////////////////////////
void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
}
////////////////////////////// clor-balance//////////////////////////////////////////////////
void SimplestCB(cv::Mat& in, cv::Mat& out, float percent)
{
    assert(in.channels() == 3);
    assert(percent > 0 && percent < 100);
    float half_percent = percent / 200.0f;
    std::vector<cv::Mat> tmpsplit; split(in, tmpsplit);
    for ( int i = 0; i < 3; i++ )
    {
        // find the low and high precentile values (based on the input percentile)
        cv::Mat flat;
        tmpsplit[i].reshape(1, 1).copyTo(flat);
        cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        int lowval = flat.at<uchar>(cvFloor((static_cast<float>(flat.cols)) * half_percent));
        int highval = flat.at<uchar>(cvCeil((static_cast<float>(flat.cols)) * (1.0 - half_percent)));
        // saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval, tmpsplit[i] > highval);
        // scale the channel
        cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
    }
    cv::merge(tmpsplit, out);
}
/////////////////////////////////// contrast-streching ///////////////////////////////////////
int computeOutput(int x, int r1, int s1, int r2, int s2)
{
    float result;
    if ( 0 <= x && x <= r1 ) {
        result = s1/r1 * x;
    }
    else if (r1 < x && x <= r2) {
        result = ((s2 - s1)/(r2 - r1)) * (x - r1) + s1;
    }
    else if ( r2 < x && x <= 255 ) {
        result = ((255 - s2)/(255 - r2)) * (x - r2) + s2;
    }
    return static_cast<int>(result);
}
//////////////////////////////////// imageCallback ///////////////////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (x == 32)
    return;

  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame.copyTo(src);
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("%s:couldn't convert from '%s' to 'bgr8' .", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}
///////////////////////////////////// histogram-equalizatiion //////////////////////////////////////////////
void equalizeHistogram(cv::Mat &src, cv::Mat &dst)
{
  cv::Mat frame1, frame2, frame3, frame4;
  frame1 = src;
  cv::imshow("frame1", frame1);

  cv::cvtColor(frame1, frame1, CV_BGR2HSV);

  cv::Mat frame_array[3];
  cv::Mat dst_array;
  cv::split(frame1, frame_array);

  equalizeHist(frame_array[0], frame_array[0]);
  equalizeHist(frame_array[1], frame_array[1]);
  equalizeHist(frame_array[2], frame_array[2]);

  cv::merge(frame_array, 3, dst_array);
  dst = dst_array;
  cv::imshow("dst_array", dst_array);
  cv::imshow("final_image", dst_array);
  cv::waitKey(0);
  return;
}
/////////////////////////////////////////// Erosion /////////////////////////////////////////////////////////////
int erosion_elem = 0;
int erosion_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

void Erosion(int, void*)
{
  int erosion_type;
  if (erosion_elem == 0) { erosion_type = cv::MORPH_RECT; }
  else if (erosion_elem == 1) { erosion_type = cv::MORPH_CROSS; }
  else if (erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = getStructuringElement(erosion_type, cv::Size(2*erosion_size + 1, 2*erosion_size+1),
    cv::Point(erosion_size, erosion_size));
  /// Apply the erosion operation

  erode(dst_array, dst_array, element);
  imshow("Erosion Demo", dst_array);
}
/////////// apply-eroion //////////////////////////////////////////////////////////////////////////////
void apply_erosion(cv::Mat &src)
{
  cv::namedWindow("Erosion Demo", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo", &erosion_elem, max_elem, Erosion);
  cv::createTrackbar("Kernel size:\n 2n +1", "Erosion Demo", &erosion_size, max_kernel_size, Erosion);
  Erosion(0, 0);
}

int main(int argc, char **argv)
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
  //////////// subscribing to the image published by the image publisher /////////////////////////////
  ros::init(argc, argv, "task_buoy");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("buoy_detection_switch", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);
  
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  ////////////////////// dynamic reconfigure ////////////////////////////////////////////////////////
  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
  cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);

  /*n.getParam("buoy_detection/t1max", t1max);
  n.getParam("buoy_detection/t1min", t1min);
  n.getParam("buoy_detection/t2max", t2max);
  n.getParam("buoy_detection/t2min", t2min);
  n.getParam("buoy_detection/t3max", t3max);
  n.getParam("buoy_detection/t3min", t3min);

  task_buoy::buoyConfig config;
  config.t1min_param = t1min;
  config.t1max_param = t1max;
  config.t2min_param = t2min;
  config.t2max_param = t2max;
  config.t3min_param = t3min;
  config.t3max_param = t3max;
  callback(config, 0);*/

  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;
  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    if(src.empty())
    {
        ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
        ros::spinOnce();
        continue;
    }
    height=src.rows;
    width=src.cols;
    step=src.step;    

    if (!IP)
    {
    ///////////////////////////// color balance, histogram equalization, erosion ////////////////////////////
      SimplestCB(frame, dst_array, 1);
      equalizeHistogram(dst_array, dst_array);
      apply_erosion(dst_array);
      /////////////// applying thresholding,reducing the noise,filters ,contour_detectors///////////////////
      cv::inRange(dst_array, hsv_min, hsv_max, thresholded);
      cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
      cv::imshow("BuoyDetection:AfterColorFiltering", thresholded);  // The stream after color filtering
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      if (contours.empty())
      {
        int x_cord = 320 - center_ideal[0].x;
        int y_cord = -240 + center_ideal[0].y;
        if (x_cord < -270)
        {
          array.data.push_back(-2);  // top
          array.data.push_back(-2);
          array.data.push_back(-2);
          array.data.push_back(-2);
          pub.publish(array);
        }
        else if (x_cord > 270)
        {
          array.data.push_back(-1);  // left_side
          array.data.push_back(-1);
          array.data.push_back(-1);
          array.data.push_back(-1);
          pub.publish(array);
        }
        else if (y_cord > 200)
        {
          array.data.push_back(-3);  // bottom
          array.data.push_back(-3);
          array.data.push_back(-3);
          array.data.push_back(-3);
          pub.publish(array);
        }
        else if (y_cord < -200)
        {
          array.data.push_back(-4);  // right_side
          array.data.push_back(-4);
          array.data.push_back(-4);
          array.data.push_back(-4);
          pub.publish(array);
        }
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
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);
      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

       float r_avg = (r[0] + r[1] + r[2] + r[3] + r[4]) / 5;
      if ((radius[0] < (r_avg + 10)) && (count_avg >= 5))
      {
        r[4] = r[3];
        r[3] = r[2];
        r[2] = r[1];
        r[1] = r[0];
        r[0] = radius[0];
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = center[0];
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        r[count_avg] = radius[0];
        center_ideal[count_avg] = center[0];
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      int net_x_cord = 320 - center_ideal[0].x + r[0];
      int net_y_cord = -240 + center_ideal[0].y + r[0];
      if (net_x_cord < -310)
      {
        array.data.push_back(-2);  // top
        array.data.push_back(-2);
        array.data.push_back(-2);
        array.data.push_back(-2);
        pub.publish(array);
      }
      else if (net_x_cord > 310)
      {
        array.data.push_back(-1);  // left_side
        array.data.push_back(-1);
        array.data.push_back(-1);
        array.data.push_back(-1);
        pub.publish(array);
        ros::spinOnce();
      }
      else if (net_y_cord > 230)
      {
        array.data.push_back(-3);  // bottom
        array.data.push_back(-3);
        array.data.push_back(-3);
        array.data.push_back(-3);
        pub.publish(array);
      }
      else if (net_y_cord < -230)
      {
        array.data.push_back(-4);  // right_side
        array.data.push_back(-4);
        array.data.push_back(-4);
        array.data.push_back(-4);
        pub.publish(array);
      }
      else if (r[0] > 110)
      {
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        pub.publish(array);
      }
      else
      {
        float distance;
        distance = pow(radius[0] / 7526.5, -.92678);  // function found using experiment
        array.data.push_back(r[0]);                   // publish radius
        array.data.push_back((320 - center_ideal[0].x));
        array.data.push_back(-(240 - center_ideal[0].y));
        array.data.push_back(distance);
        pub.publish(array);
      }
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
      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
        break;
    }
    else
    {
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
  }
  return 0;
}

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
#include <dynamic_reconfigure/server.h>
#include <task_gate/gateConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int w = -2, x = -2, y = -2, z = -2;
bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;

cv::Mat frame;
cv::Mat newframe;
int count = 0, count_avg = 0, p = -1;

void callback(task_gate::gateConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Gate_Reconfigure Request : New parameters : %d %d %d %d %d %d ", t1min, t1max, t2min, t2max, t3min, t3max);
}

void gateListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (p == 32)
    return;
  try
  {
    count++;
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    ///////////////////////////// DO NOT REMOVE THIS, IT COULD BE INGERIOUS TO HEALTH /////////////////////
    newframe.copyTo(frame);
    ////////////////////////// FATAL ///////////////////////////////////////////////////
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void balance_white(cv::Mat mat) {
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }


  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}


int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on
  // std::string Video_Name = "Random_Video";
  // if (argc >= 2)
  //   flag = true;
  // if (argc == 3)
  // {
  //   video = true;
  //   std::string avi = ".avi";
  //   Video_Name = (argv[2]) + avi;
  // }

  // cv::VideoWriter output_cap(Video_Name, CV_FOURCC('D', 'I', 'V', 'X'), 9, cv::Size(640, 480));

  ros::init(argc, argv, "gate_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/gate", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("gate_detection_switch", 1000, &gateListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

  dynamic_reconfigure::Server<task_gate::gateConfig> server;
  dynamic_reconfigure::Server<task_gate::gateConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // n.getParam("gate_detection/t1max", t1max);
  // n.getParam("gate_detection/t1min", t1min);
  // n.getParam("gate_detection/t2max", t2max);
  // n.getParam("gate_detection/t2min", t2min);
  // n.getParam("gate_detection/t3max", t3max);
  // n.getParam("gate_detection/t3min", t3min);

  // task_gate::gateConfig config;
  // config.t1min_param = t1min;
  // config.t1max_param = t1max;
  // config.t2min_param = t2min;
  // config.t2max_param = t2max;
  // config.t3min_param = t3min;
  // config.t3max_param = t3max;
  // callback(config, 0);

  cvNamedWindow("GateDetection:AfterThresholding", CV_WINDOW_NORMAL);
  cvNamedWindow("GateDetection:AfterEnhancing", CV_WINDOW_NORMAL);
  cvNamedWindow("GateDetection:Gate",CV_WINDOW_NORMAL);

  // capture size -
  CvSize size = cvSize(width, height);

  // Initialize different images that are going to be used in the program
  // image converted to HSV plane
  // asking for the minimum distance where bwe fire torpedo

  cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
  cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
  cv::Mat lab_image, image_clahe, dst1, balanced_image1, dstx, thresholded, dst;
  // cv::Mat balanced_image;
  std::vector<cv::Mat> lab_planes(3);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    // Get one frame
    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    // if (video)
    //   output_cap.write(frame);
    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    // cv::Mat white_balance;
    // balanceWhite(frame, white_balance, WHITE_BALANCE_SIMPLE, 0, 255, 0, 255);
    frame.copyTo(balanced_image);
    balance_white(balanced_image);
    // fastNlMeansDenoisingColored(balanced_image, balanced_image, 3, 7, 21);
    bilateralFilter(balanced_image, dst1, 4, 8, 8);
        
    cv::cvtColor(frame, lab_image, CV_BGR2Lab);

    // Extract the L channel
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
      
    // fastNlMeansDenoisingColored(image_clahe, image_clahe, 3, 7, 21);
        
    for (int i=0; i < 10; i++)
    {
      bilateralFilter(image_clahe, dstx, 6, 8, 8);
      bilateralFilter(dstx, image_clahe, 6, 8, 8);
    }
    // balance_white(dst2);
    image_clahe.copyTo(balanced_image1);
    balance_white(balanced_image1);
    
    for (int i=0; i < 3; i++)
    {
      bilateralFilter(dst1, dstx, 6, 8, 8);
      bilateralFilter(dstx, dst1, 6, 8, 8);
    }

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    // cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
    // Filter out colors which are out of range.
    cv::inRange(balanced_image1, cv::Scalar(0, 0, 80), cv::Scalar(100, 50, 260), thresholded);
    // Split image into its 3 one dimensional images
    // cv::Mat thresholded_hsv[3];
    // cv::split(hsv_frame, thresholded_hsv);

    // Filter out colors which are out of range.
    // cv::inRange(thresholded_hsv[0], cv::Scalar(t1min, 0, 0, 0), cv::Scalar(t1max, 0, 0, 0), thresholded_hsv[0]);
    // cv::inRange(thresholded_hsv[1], cv::Scalar(t2min, 0, 0, 0), cv::Scalar(t2max, 0, 0, 0), thresholded_hsv[1]);
    // cv::inRange(thresholded_hsv[2], cv::Scalar(t3min, 0, 0, 0), cv::Scalar(t3max, 0, 0, 0), thresholded_hsv[2]);
    // cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    // The stream after color filtering

    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    cv::imshow("GateDetection:AfterEnhancing", balanced_image1);
    cv::imshow("GateDetection:AfterThresholding", thresholded);

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if (!IP)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
      double largest_area = 0, largest_contour_index = 0;

      if (contours.empty())
      {
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
      // std::vector<std::vector<cv::Point> > hull(contours.size());
      // convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      // cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
      // std::vector<cv::Vec4i> hierarchy;
      // cv::Scalar color(255, 255, 255);

      cv::Rect boundRect;

      boundRect = boundingRect(cv::Mat(contours[largest_contour_index]));

      rectangle(Drawing, boundRect.tl(), boundRect.br(), color, 2, 8, 0);

      cv::Point center;
      center.x = ((boundRect.br()).x + (boundRect.tl()).x) / 2;
      center.y = ((boundRect.tl()).y + (boundRect.br()).y) / 2;
      int side_x = (boundRect.br()).x - (boundRect.tl()).x;
      int side_y = -((boundRect.tl()).y - (boundRect.br()).y);
      // drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);

      cv::Mat frame_mat = frame;
      cv::Point2f screen_center;
      screen_center.x = 320;  // size of my screen
      screen_center.y = 240;

      circle(frame_mat, center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      rectangle(frame_mat, boundRect.tl(), boundRect.br(), color, 2, 8, 0);
      circle(frame_mat, screen_center, 4, cv::Scalar(150, 150, 150), -1, 8, 0);  // center of screen

      cv::imshow("GateDetection:Gate", frame_mat);

      w = (boundRect.br()).x;
      x = (boundRect.br()).y;
      y = (boundRect.tl()).y;
      z = (boundRect.tl()).x;
      if ((side_y < 70) && (z == 1))
      {
        array.data.push_back(-2);
        array.data.push_back(-2);  //  hits left
      }
      else if ((side_y < 70) && (x == frame.cols - 1))
      {
        array.data.push_back(-4);
        array.data.push_back(-4);  //  hits left
      }
      else if (w == (frame.rows) - 1 || x == (frame.cols) - 1 || y == 1 || z == 1)
      {
        if (y == 1)
        {
          array.data.push_back(-1);
          array.data.push_back(-1);  //  hits top
        }
        else if (w == frame.rows - 1)
        {
          array.data.push_back(-3);
          array.data.push_back(-3);  //  hits bottom
        }
        else if (z == 1)
        {
          array.data.push_back(-2);
          array.data.push_back(-2);  //  hits left
        }
        else if (x == frame.cols - 1)
        {
          array.data.push_back(-4);
          array.data.push_back(-4);  //  hits right
        }
      }
      else
      {
        array.data.push_back((frame.cols / 2.0 - center.x));
        array.data.push_back(-(frame.rows / 2.0 - center.y));
      }
      pub.publish(array);
      ros::spinOnce();

      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
        break;
    }
    else
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      if ((cvWaitKey(10) & 255) == 32)
      {
        if (p == 32)
          p = -1;
        else
          p = 32;
      }
      if (p == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());
      ros::spinOnce();
    }
  }
  // output_cap.release();
  return 0;
}

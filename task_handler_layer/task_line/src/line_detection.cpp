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
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>


int percentage = 20;  // used for how much percent of the screen should be orange
                      // before deciding that a line is below. Used in
                      // dynamic_reconfig
// callback for change the percent of orange before saying there is a line below
bool IP = false;
bool flag = false;
bool video = false;

cv::Mat frame;
cv::Mat newframe;
int count = 0;
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


// callback for off switch.
int detect(cv::Mat image)
{

  cv::Size size(640, 480);  // the dst image size,e.g.100x100
  cv::Mat resizeimage;      // dst image
  cv::Mat bgr_image;
  resize(image, resizeimage, size);  // resize image
  cv::waitKey(20);
  // detect red color here
  medianBlur(resizeimage, bgr_image, 3);  // blur to reduce noise
  // Convert input image to HSV
  cv::Mat hsv_image;
  cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
  // keep only red color
  cv::Mat red_hue_image;
  inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(179, 255, 255), red_hue_image);
  GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);  // gaussian blur to remove false positives
  int nonzero = countNonZero(red_hue_image);
  int nonzeropercentage = nonzero / 3072;
  if (nonzero > (3072 * percentage))  // return 1 if a major portion of image
                                      // has red color, Note : here the size of
                                      // image is 640X480 = 307200.
    return 1;
 
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");
  ros::NodeHandle n;
  ros::Publisher robot_pub = n.advertise<std_msgs::Bool>("linedetected", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("line_detection_switch", 1000, &lineDetectedListener);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  ros::Rate loop_rate(12);  // this rate should be same as the rate of camera
                            // input. and in the case of other sensors , this
                            // rate should be same as there rate of data
                            // generation

  while (ros::ok())
  {
  	if (!frame.data)  // Check for invalid input
    {
      std::cout << "Could not open or find the image" << std::endl;
      ros::spinOnce();
      continue;
      // TODO(shikherverma) : for now I am resetting the video but later we need to handle this
      // camera not available error properly
    }
    if(!IP)
    {  
      int alert = detect(frame);
      if (alert == 1)
      {
        std_msgs::Bool msg;
        msg.data = true;
        robot_pub.publish(msg);
        ROS_INFO("found line");
      }
      else if (alert == 0)
      {
        std_msgs::Bool msg;
        msg.data = false;
        robot_pub.publish(msg);
        ROS_INFO("no line");
      }
      else
      {
        return 0;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>  // for converting the command line parameter to integer
#include <string>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stdio.h>

/*! \file
* \brief super short description
*
* Long decription
*/

std::string exec(const char *cmd)
{
  char buffer[128];
  std::string result = "";
  FILE* pipe = popen(cmd, "r");
  if (!pipe)
    throw std::runtime_error("popen() failed!");
  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  return result;
}

bool checkIfFrontisZero()
{
  std::string productID = "7BE06A00";
  const char *command = "udevadm info -a -p  $(udevadm info -q path -n /dev/video0)";
  std::string video0 = exec(command);
  if (video0.find(productID))
    return true;
  return false;
}

/*! member description */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  std::string bottom_topic_name, front_topic_name, node_name;
  int front_camera_number, bottom_camera_number;
  front_topic_name = "/varun/sensors/front_camera/image_raw";
  bottom_topic_name = "/varun/sensors/bottom_camera/image_raw";
  image_transport::ImageTransport it(nh);
  image_transport::Publisher front_pub = it.advertise(front_topic_name, 1);
  image_transport::Publisher bottom_pub = it.advertise(bottom_topic_name, 1);

  if (checkIfFrontisZero())
  {
    front_camera_number = 0;
    bottom_camera_number = 1;
  }
  else
  {
    front_camera_number = 1;
    bottom_camera_number = 0;
  }

  cv::VideoCapture front_cap(front_camera_number);
  cv::VideoCapture bottom_cap(bottom_camera_number);
  cv::Mat front_frame, bottom_frame;
  sensor_msgs::ImagePtr front_msg, bottom_msg;
  int loopRate = 10;
  ros::Rate loop_rate(loopRate);
  while (nh.ok())
  {
    front_cap >> front_frame;
    bottom_cap >> bottom_frame;
    // Check if grabbed frame is actually full with some content
    if (!front_frame.empty())
    {
      front_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", front_frame).toImageMsg();
      front_pub.publish(front_msg);
    }
    // Check if grabbed frame is actually full with some content
    if (!bottom_frame.empty())
    {
      bottom_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bottom_frame).toImageMsg();
      bottom_pub.publish(bottom_msg);
    }
    ros::spinOnce();
  }
}

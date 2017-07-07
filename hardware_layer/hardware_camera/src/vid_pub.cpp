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
#include <dynamic_reconfigure/server.h>
#include <hardware_camera/cameraConfig.h>

/*! \file
* \brief super short description
*
* Long decription
*/

double rotation_error;
bool flag;

std::string exec(const char *cmd)
{
  char buffer[128];
  std::string result = "";
  FILE *pipe = popen(cmd, "r");
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
  std::string serialNumber = "243FA8C0";
  const char *command = "udevadm info -a -p  $(udevadm info -q path -n /dev/video0)";
  std::string video0 = exec(command);
  if (video0.find(serialNumber))
    return true;
  return false;
}

cv::Mat rotate(cv::Mat src, double angle)
{
  // get rotation matrix for rotating the image around its center
  cv::Point2f center(src.cols / 2.0, src.rows / 2.0);
  cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
  // determine bounding rectangle
  cv::Rect bbox = cv::RotatedRect(center, src.size(), angle).boundingRect();
  // adjust transformation matrix
  rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
  rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;
  cv::Mat dst;
  cv::warpAffine(src, dst, rot, bbox.size());
  return dst;
}

// dynamic reconfig
void callback(hardware_camera::cameraConfig &config, double level)
{
  ROS_INFO("%s Vide_pub: Reconfigure Request: angle= %f flag= %d", ros::this_node::getName().c_str(), config.angle,
           config.flag);
  rotation_error = config.angle;
  flag = config.flag;
}

/*! member description */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<hardware_camera::cameraConfig> server;
  dynamic_reconfigure::Server<hardware_camera::cameraConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // get launch file constants
  double error_angle;
  int flag_integer;
  nh.getParam("cameras/error_angle", error_angle);
  nh.getParam("cameras/flag", flag_integer);

  // set launch file constants
  hardware_camera::cameraConfig config;
  config.angle = error_angle;
  config.flag = (flag_integer == 1) ? true : false;
  callback(config, 0);

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
    if (flag)
    {
      front_cap >> bottom_frame;
      bottom_cap >> front_frame;
    }
    else
    {
      front_cap >> front_frame;
      bottom_cap >> bottom_frame;
    }
    // Check if grabbed frame is actually full with some content
    if (!front_frame.empty())
    {
      cv::Mat final_front_frame = rotate(front_frame, rotation_error);
      front_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_front_frame).toImageMsg();
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

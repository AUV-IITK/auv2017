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
#define D 10
#include <sstream>


cv::Mat frame;
cv::Mat newframe;

bool IP = false;


void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
  std::cout << "hi callback";
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    // count++;
    // std::cout<<"imageCallback"<<count<<std::endl;
    // imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    newframe=cv_bridge::toCvShare(msg, "bgr8")->image;
    // printf("Shit %d\n",count);
    // cv::imshow("newframe",newframe);
      newframe.copyTo(frame);

  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "buoy_detection");
  ros::NodeHandle n;
  // ROS_INFO("asa");
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("/balls_off", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);
  
  image_transport::ImageTransport it(n); 

  image_transport::Subscriber sub1 = it.subscribe("camera/image", 1, imageCallback);
  std::cout<<"in while"<<std::endl;
  // *********************  UNCOMMENT BELOW PART TO TAKE FEED FROM CAMERA AND COMMENT OUT PART AFTER THAT************\\


  //*****************************************************************************
  while (1)
  {
      
      if (frame.empty())
      {
        std::cout << "empty frame \n";
        ros::spinOnce();
        continue;
      }
      cv::imshow("show",frame);
      cv::imshow("show2",frame); 


      ros::spinOnce();

      if ((cvWaitKey(10) & 255) == 27)
      break;

 
  }

 return 0;
}
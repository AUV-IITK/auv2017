#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <linedetection/orangeConfig.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

#include <ctime>

using namespace cv;
using namespace std;

Mat image;
VideoCapture inputVideo("test1.avi");

int firstflag = 0; // used to check if the function detect() is being run for the first time
int percentage = 20; //used for how much percent of the screen should be orange before deciding that a line is below. Used in dynamic_reconfig
//callback for change the percent of orange before saying there is a line below
void callback(linedetection::orangeConfig &config, int level) {
	percentage = config.int_param;
	ROS_INFO("Reconfigure Request. New percentage : %d", percentage);
}

int detect()
{
	if(firstflag==0)//if detect is being run for the first time then create a named window
	{
		namedWindow( "Display", WINDOW_AUTOSIZE );// Create a window for display.
		firstflag++;
	}
	inputVideo.read(image);
	if(! image.data )                           // Check for invalid input
    {
        // cout <<  "Could not open or find the image" << std::endl ;
        // return -1;
        //TODO : for now I am resetting the video but later we need to handle this camera not available error properly
        VideoCapture inputVideo("test1.avi");
		inputVideo.read(image);
    }
    else
	{
		Size size(640,480);//the dst image size,e.g.100x100
		Mat resizeimage;//dst image
		Mat bgr_image;
		resize(image,resizeimage,size);//resize image
		waitKey(20);
		//detect red color here
		medianBlur(resizeimage, bgr_image, 3);//blur to reduce noise
		// Convert input image to HSV
		Mat hsv_image;
		cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
		//keep only red color
		Mat red_hue_image;
		inRange(hsv_image, Scalar(0, 100, 100), Scalar(179, 255, 255), red_hue_image);
		GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);//gaussian blur to remove false positives
		int nonzero = countNonZero(red_hue_image);
		int nonzeropercentage = nonzero/3072;
		imshow("Display",red_hue_image);
		if (nonzero > (3072*percentage)) //return 1 if a major portion of image has red color, Note : here the size of image is 640X480 = 307200.
			return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orangedetection");
  ros::NodeHandle n;
  ros::Publisher robot_pub = n.advertise<std_msgs::String>("robot", 1000);
  ros::Rate loop_rate(12);//this rate should be same as the rate of camera input. and in the case of other sensors , this rate should be same as there rate of data generation

  //setting callback for the orange percentage.
  dynamic_reconfigure::Server<linedetection::orangeConfig> server;
  dynamic_reconfigure::Server<linedetection::orangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
	int alert = detect();
	if(alert==1)
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "s" ;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		robot_pub.publish(msg);
	}
	else if(alert==0)
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "a" ;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		robot_pub.publish(msg);
	}
	else
	{
		return 0;
	}
	ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

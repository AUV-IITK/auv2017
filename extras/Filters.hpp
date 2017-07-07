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

void SimplestCB(cv::Mat& in, cv::Mat& out, float percent);

void equalizeHistogram(cv::Mat &src,cv::Mat &dst);

class Erode
{
public:
	int erosion_elem;
	int erosion_size;
	int const max_elem=2;
	int const max_kernel_size=21;

	Erode(int size,int elem);

	~Erode();

	void Erosion(int, void*);

	void apply_erosion(cv::Mat &src);

};

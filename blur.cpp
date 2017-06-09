#include <iostream>
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
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>


using namespace std;
using namespace cv;

int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;

void bandPassFilter(cv::Mat &src,cv::Mat &dst){

	cv::Mat img1,img2;
	GaussianBlur(src,img1,Size(3,3),0,0);
    cv::imshow("img1",img1);

    GaussianBlur(src,img2,Size(9,9),100,100);
    cv::imshow("img2",img2);

    cv::subtract(img1,img2,dst); 
    cv::Mat dst1;
    cv::subtract(img2,img1,dst1);
    cv::imshow("dst1",dst1);

}


int main(int argc,char **argv){

	cv::Mat src=imread(argv[1],1);
	cv::imshow("src",src);
	cv::Mat dst;
	bandPassFilter(src,dst);
	cv::imshow("final_image",dst);
	cv::waitKey(0);
	return 0;

}
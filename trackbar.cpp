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

cv::Mat src,dst,thresholded;

int t1min=0, t1max=255, t2min=0, t2max=255, t3min=0, t3max=255;
int t1_min,t1_max,t2_min,t2_max,t3_min,t3_max;

cv::Scalar hsv_min;
cv::Scalar hsv_max;

void callBack(int,void*){

	t1_min=t1min;
	t1_max=t1max;
	t2_min=t2min;
	t2_max=t2max;
	t3_min=t3min;
	t3_max=t3max;

	hsv_min = cv::Scalar(t1_min, t2_min,t3_min, 0);
    hsv_max = cv::Scalar(t1_max,t2_max,t3_max, 0);
    cv::inRange(src,hsv_min,hsv_max,thresholded);
}


int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;

void bandPassFilter(cv::Mat &src,cv::Mat &dst){

	cv::Mat img1,img2;
	GaussianBlur(src,img1,Size(3,3),0,0);
    //cv::imshow("img1",img1);

    GaussianBlur(src,img2,Size(9,9),100,100);
    //cv::imshow("img2",img2);

    cv::subtract(img1,img2,dst); 
    //cv::Mat dst1;
    //cv::subtract(img2,img1,dst1);
    //cv::imshow("dst1",dst1);

}


int main(int argc, char **argv){

	src=imread(argv[1],1);
	cv::namedWindow("src",CV_WINDOW_AUTOSIZE);
	cv::imshow("src",src);
	
	cv::namedWindow("threshold",CV_WINDOW_AUTOSIZE);	
	
	createTrackbar("t1min","threshold",&t1min,255,callBack);
	createTrackbar("t1max","threshold",&t1max,255,callBack);
	createTrackbar("t2min","threshold",&t2min,255,callBack);
	createTrackbar("t2max","threshold",&t2max,255,callBack);
	createTrackbar("t3min","threshold",&t3min,255,callBack);
	createTrackbar("t3max","threshold",&t3max,255,callBack);
	
    cv::imshow("threshold",thresholded);

    bandPassFilter(thresholded,dst);
	cv::imshow("final_image",dst);

    cv::waitKey(0);
    return 0;
}
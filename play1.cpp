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

void SimplestCB(Mat& in, Mat& out, float percent) {
    assert(in.channels() == 3);
    assert(percent > 0 && percent < 100);

    float half_percent = percent / 200.0f;

    vector<Mat> tmpsplit; split(in,tmpsplit);
    for(int i=0;i<3;i++) {
        //find the low and high precentile values (based on the input percentile)
        Mat flat; tmpsplit[i].reshape(1,1).copyTo(flat);
        cv::sort(flat,flat,CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        int lowval = flat.at<uchar>(cvFloor(((float)flat.cols) * half_percent));
        int highval = flat.at<uchar>(cvCeil(((float)flat.cols) * (1.0 - half_percent)));
        cout << lowval << " " << highval << endl;
        
        //saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval,tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval,tmpsplit[i] > highval);
        
        //scale the channel
        normalize(tmpsplit[i],tmpsplit[i],0,255,NORM_MINMAX);
    }
    merge(tmpsplit,out);
}

int computeOutput(int x, int r1, int s1, int r2, int s2)
{
    float result;
    if(0 <= x && x <= r1){
        result = s1/r1 * x;
    }else if(r1 < x && x <= r2){
        result = ((s2 - s1)/(r2 - r1)) * (x - r1) + s1;
    }else if(r2 < x && x <= 255){
        result = ((255 - s2)/(255 - r2)) * (x - r2) + s2;
    }
    return (int)result;
}


int main(int argc,char **argv){
  int t1min=126,t1max=200;
  int t2min=0,t2max=255;
  int t3min=0,t3max=255;
	int r1 = 70, s1 = 0, r2 = 140, s2 = 255;
  	cv::Mat hsv_frame, frame, thresholded, thresholded1, thresholded2;

    frame=cv::imread(argv[1],1);

    cvNamedWindow("frame1",CV_WINDOW_AUTOSIZE);
    cv::imshow("frame1",frame);
    cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
    cvNamedWindow("frame",CV_WINDOW_AUTOSIZE);
    cv::imshow("frame",hsv_frame);
    cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
    cvNamedWindow("hsv",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("thresholded_hsv",CV_WINDOW_AUTOSIZE);
	
    // Filter out colors which are out of range.
    cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);
    cv::imshow("hsv",thresholded);
    // Split image into its 3 one dimensional images
    //cv::Mat tmpsplit[3];
    cv::Mat thresholded_hsv[3];
    cv::split(hsv_frame, thresholded_hsv);
    //cv::split(frame,tmpsplit);
    // Filter out colors which are out of range.
    /*for(int i=0;i<3;i++){
    	cv::normalize(tmpsplit[i],tmpsplit[i],0,255,NORM_MINMAX);
    }*/
    cv::Mat tmp;
    SimplestCB(frame,tmp,1); //color balance 
    cv::Mat new_image;
    new_image=frame.clone();
    //cout << "*"<< endl;
    for(int y = 0; y < frame.rows; y++){
    	//cout << "*" << endl;
        for(int x = 0; x < frame.cols; x++){
            //cout << "**" << endl;
            for(int c = 0; c < 3; c++){
            	//cout << "***" << endl;
                int output = computeOutput(frame.at<Vec3b>(y,x)[c], r1, s1, r2, s2);
                //cout << output << endl;
                new_image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(output);
                //cout << "*****" << endl;
            }
        }
    }
    cv::Mat grey_image,dst;
    cv::cvtColor(frame,grey_image,CV_BGR2GRAY);
    cv::inpaint(frame,grey_image,dst,10,INPAINT_NS);
    //cout << "*"<< endl;
    cvNamedWindow("new_image",CV_WINDOW_AUTOSIZE);
 	cv::imshow("new_image",new_image);
 	cv::Mat new_image2;
 	cv::cvtColor(new_image,new_image2,CV_BGR2HSV);
 	cv::imshow("new_image2",new_image2);
 	cv::Mat thresholded3;
    cv::inRange(new_image2,hsv_min,hsv_max,thresholded3);
    cv::imshow("thresholded3",thresholded3);
    //cv::merge(tmpsplit,3,tmp);
    cvNamedWindow("tmp",CV_WINDOW_AUTOSIZE);
    cv::imshow("tmp",tmp);
    cv::inRange(thresholded_hsv[0], cv::Scalar(t1min, 0, 0, 0), cv::Scalar(t1max, 0, 0, 0), thresholded_hsv[0]);
    cv::inRange(thresholded_hsv[1], cv::Scalar(t2min, 0, 0, 0), cv::Scalar(t2max, 0, 0, 0), thresholded_hsv[1]);
    cv::inRange(thresholded_hsv[2], cv::Scalar(t3min, 0, 0, 0), cv::Scalar(t3max, 0, 0, 0), thresholded_hsv[2]);
    
    cv::merge(thresholded_hsv,3,thresholded1);
    //cout << "*****" << endl;

    cvNamedWindow("color_balance",CV_WINDOW_AUTOSIZE);
    cv::imshow("color_balance",thresholded1);
    
    cvNamedWindow("hsv_frame",CV_WINDOW_AUTOSIZE);
    cv::inRange(thresholded1,hsv_min,hsv_max,thresholded2);
    cv::imshow("hsv_frame",thresholded2);
    //cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    //cout << "hagaap" << endl;
    //cv::imshow("thresholded_hsv", thresholded);  // The stream after color filtering
	cv::waitKey(0);
    return 0;
}

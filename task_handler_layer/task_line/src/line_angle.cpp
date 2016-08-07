// Copyright 2016 AUV-IITK
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Header.h"

using cv::Mat;
using cv::split;
using cv::Size;
using cv::Scalar;
using cv::normalize;
using cv::Point;
using cv::VideoCapture;
using cv::NORM_MINMAX;
using cv::MORPH_ELLIPSE;
using cv::COLOR_BGR2HSV;
using cv::destroyAllWindows;
using cv::getStructuringElement;
using cv::Vec4i;
using cv::namedWindow;
using std::vector;
using std::endl;
using std::cout;

bool stop = false;
bool IP=true;
bool flag=false;
bool video = false;
Mat img, imgHSV, imgThresholded, imgSmooth, imgCanny, imgLines, frame;
int lineCount = 0;

// parameters in param file should be nearly the same as the commented values
// params for an orange strip
int LowH=0;   // 0
int HighH=88;  // 88

int LowS=0;   // 0
int HighS=251;  // 251

int LowV=0;   // 0
int HighV=255;  // 255

// params for smoothing image
int ksize=21;  // 21
int stype=4;  // 4  -> decides which type of filter has to be used

// params for bilateral filter in smoothcallback
int sigmaColor=150;  // 150
int sigmaSpace=10;  // 10

// params for hough line transform
int lineThresh=60;     // 60
int minLineLength=70;  // 70
int maxLineGap=10;     // 10
int houghThresh=15;    // 15

double rho = 0.1;
double finalAngle;
double minDeviation = 0.02;

int count=0;
void StretchContrast()
{
  Mat ch[3];
  split(img, ch);
  normalize(ch[1], ch[1], 255, 0, NORM_MINMAX);
  normalize(ch[2], ch[2], 255, 0, NORM_MINMAX);
  normalize(ch[0], ch[0], 255, 0, NORM_MINMAX);
  merge(ch, 3, img);
}

void lineListener(std_msgs::Bool msg)
{
  IP = msg.data;
}
// called when large no of line detected

double computeMean(vector<double> &newAngles)
{
  double sum = 0;
  for (size_t i = 0; i < newAngles.size(); i++)
  {
    sum = sum + newAngles[i];
  }
  return sum / newAngles.size();
}
// called when few lines are detected
// to remove errors due to any stray results
double computeMode(vector<double> &newAngles)
{
  double mode = newAngles[0];
  int freq = 1;
  int tempFreq;
  double diff;
  for (int i = 0; i < newAngles.size(); i++)
  {
    tempFreq = 1;

    for (int j = i + 1; j < newAngles.size(); j++)
    {
      diff = newAngles[j] - newAngles[i] > 0.0 ? newAngles[j] - newAngles[i] : newAngles[i] - newAngles[j];
      if (diff <= minDeviation)
      {
        tempFreq++;
        newAngles.erase(newAngles.begin() + j);
        j = j - 1;
      }
    }

    if (tempFreq >= freq)
    {
      mode = newAngles[i];
      freq = tempFreq;
    }
  }

  return mode;
}

// callback for smoothing image
// contains different filters "stype" decides which filter to use
void SmoothCallback(int, void *)
{ 
  if (ksize % 2 == 0)
    ksize = ksize + 1;  // kernel size can not be even
  switch (stype)
  {
    case 1:
      blur(frame, imgSmooth, Size(ksize, ksize));  // ksize=17
      break;
    case 2:
      GaussianBlur(frame, imgSmooth, Size(ksize, ksize), 0, 0);
      break;
    case 3:
      medianBlur(frame, imgSmooth, ksize);
      break;
    case 4:
      bilateralFilter(frame, imgSmooth, ksize, sigmaColor, sigmaSpace);
      break;
    default:
      cout << "Please enter a smoothing type " << endl;
  }
}

// contains canny edge detection and houghline transform

void callback(int, void *)
{
  cvtColor(imgSmooth, imgHSV, COLOR_BGR2HSV);  // Convert the captured frame from BGR to HSV
  inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), imgThresholded);  // Threshold the image

  // opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  // closing (removes small holes from the foreground)
  dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  Canny(imgThresholded, imgCanny, 50, 150, 3, true);  // canny edge detection

  vector<Vec4i> lines;

  HoughLinesP(imgCanny, lines, 1, CV_PI / 180, lineThresh, minLineLength, maxLineGap);

  frame.copyTo(imgLines);
  imgLines = Scalar(0, 0, 0);
  vector<double> angles(lines.size());

  lineCount = lines.size();

  cout << "No of lines : " << lines.size() << endl
       << "Angles(radian): ";

  for (size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, CV_AA);
    angles[i] = atan(static_cast<double>(l[2] - l[0]) / (l[1] - l[3]));
    cout << angles[i] << " ";
  }
  cout << endl;
  if (flag)
    imshow("LINES", imgLines + frame);

  // if num of lines are large than one or two stray lines won't affect the mean
  // much
  // but if they are small in number than mode has to be taken to save the error
  // due to those stray line

  if (lines.size() > 0 && lines.size() < 10)
    finalAngle = computeMode(angles);
  else if (lines.size() > 0)
    finalAngle = computeMean(angles);

  cout << "Final Angle: " << finalAngle << endl
       << endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    Mat newframe;
    count++;
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    namedWindow("newframe", CV_WINDOW_NORMAL);
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

int main(int argc, char **argv)
{

  std::string Video_Name = "Random_Video";
  if (argc >= 2)
    flag = true;
  if (argc == 3)
  {
    video = true;
    std::string avi = ".avi";
    Video_Name = (argv[2]) + avi;
  }

  cv::VideoWriter output_cap(Video_Name, CV_FOURCC('D', 'I', 'V', 'X'), 9, cv::Size(640, 480));
  if (flag)
    namedWindow("LINES", CV_WINDOW_NORMAL);

  ros::init(argc, argv, "line_angle");
  ros::NodeHandle n;
  ros::Publisher tracker_pub1 = n.advertise<std_msgs::Float64>("lineAngle", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("line_angle_switch", 1000, &lineListener);

  // ros::Rate loop_rate(loopRate);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  while (ros::ok())
  {
    std_msgs::Float64 msg;
    // loop_rate.sleep();
    /*
    msg.data never takes positive 90
    when the angle is 90 it will show -90
    -------------TO BE CORRECTED-------------
    */
    if(flag)
    {  
      cvCreateTrackbar("LowH", "LINES", &LowH, 260, NULL);
      cvCreateTrackbar("HighH", "LINES", &HighH, 260, NULL);
      cvCreateTrackbar("LowS", "LINES", &LowS, 260, NULL);
      cvCreateTrackbar("HighS", "LINES", &HighS, 260, NULL);
      cvCreateTrackbar("LowV", "LINES", &LowV, 260, NULL);
      cvCreateTrackbar("HighV", "LINES", &HighV, 260, NULL);
    }
    if (frame.empty())
    {
      std::cout << "empty frame \n";
      ros::spinOnce();
      continue;
    }
    msg.data = -finalAngle * (180 / 3.14);
    // cout<<lineCount<<endl;
    if(!IP)
    {
      if (lineCount > 0)
      {
        tracker_pub1.publish(msg);
        ROS_INFO("%lf", msg.data);
      }
      // bSuccess = cap.read(img);  // read a new frame from video
      // if (!bSuccess)  // if not success, break loop
      // {
      //   cout << "Cannot read a frame from video stream" << endl;
      //   break;
      // }
      //        StretchContrast();
      SmoothCallback(0, 0);  // for assigning imgSmooth initially
      callback(0, 0);        // for displaying the thresholded image initially
      if ((cvWaitKey(10) & 255) == 27)
          break;
      // if (cvWaitKey(10) == 27)
      // {
      //   break;
      // }
      ros::spinOnce();
       // loop_rate.sleep();
    }
    // while(!start){
    //  ros::spinOnce();
    //  loop_rate.sleep();
    //  ROS_INFO("Paused by taskHandler");
    // }
    ros::spinOnce();
    std::cout << "waiting \n";
    if ((cvWaitKey(10) & 255) == 27)
          break;
  }

  destroyAllWindows();
}

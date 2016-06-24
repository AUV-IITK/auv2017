// Copyright 2016 AUV-IITK
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <vector>

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

Mat img, imgHSV, imgThresholded, imgSmooth, imgCanny, imgLines;
int lineCount = 0;

// parameters in param file should be nearly the same as the commented values
// params for an orange strip
int LowH;   // 0
int HighH;  // 88

int LowS;   // 0
int HighS;  // 251

int LowV;   // 0
int HighV;  // 255

// params for smoothing image
int ksize;  // 21
int stype;  // 4  -> decides which type of filter has to be used

// params for bilateral filter in smoothcallback
int sigmaColor;  // 150
int sigmaSpace;  // 10

// params for hough line transform
int lineThresh;     // 60
int minLineLength;  // 70
int maxLineGap;     // 10
int houghThresh;    // 15

double rho = 0.1;
double finalAngle;
double minDeviation = 0.02;

bool vidCheck = false;

void StretchContrast()
{
  Mat ch[3];
  split(img, ch);
  normalize(ch[1], ch[1], 255, 0, NORM_MINMAX);
  normalize(ch[2], ch[2], 255, 0, NORM_MINMAX);
  normalize(ch[0], ch[0], 255, 0, NORM_MINMAX);
  merge(ch, 3, img);
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
      blur(img, imgSmooth, Size(ksize, ksize));  // ksize=17
      break;
    case 2:
      GaussianBlur(img, imgSmooth, Size(ksize, ksize), 0, 0);
      break;
    case 3:
      medianBlur(img, imgSmooth, ksize);
      break;
    case 4:
      bilateralFilter(img, imgSmooth, ksize, sigmaColor, sigmaSpace);
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

  img.copyTo(imgLines);
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
  if (vidCheck)
    imshow("LINES", imgLines + img);

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

void offCallback(std_msgs::Bool msg)
{
  stop = msg.data;
  if (stop)
  {
    ROS_INFO("Sucide Sucide !!");
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    cout << "please specify whether you want the display or not : " << endl
         << "USAGE: " << endl
         << "first argument: 1 for display, otherwise 0" << endl;
    cout << "second argument : 0 / 1 for live " << endl;
    return 0;
  }

  if (*argv[1] == '1')
    vidCheck = true;
  int index = 0;
  if (*argv[2] == '1')
    index = 1;
  // else if(*argv[2] == '2') index=2;
  // enter the path which contains params.txt

  FILE *fp2 = fopen("src/task_line_following/src/hsv.txt", "r");
  fscanf(fp2, "%d %d %d %d %d %d", &LowH, &HighH, &LowS, &HighS, &LowV, &HighV);
  fclose(fp2);

  FILE *fp = fopen("src/task_line_following/src/params.txt", "r");
  fscanf(fp, "%d %d %d %d\n%d %d %d\n%d", &ksize, &stype, &sigmaSpace, &sigmaColor, &lineThresh, &minLineLength,
         &maxLineGap, &houghThresh);
  fclose(fp);

  // VideoCapture cap(index);

  // VideoCapture cap("src/task_line_following/src/outputnorm.avi"); //path of the
  // video for checking the code
  // VideoCapture cap("src/task_line_following/src/TestingLine.mp4");
  VideoCapture cap("src/task_line_following/src/test1.avi");

  if (!cap.isOpened())  // if not success, exit program
  {
    cout << "Cannot get the stream" << endl;
    return -1;
  }

  if (vidCheck)
    namedWindow("LINES", CV_WINDOW_AUTOSIZE);

  ros::init(argc, argv, "line_angle");
  ros::NodeHandle node;
  ros::Publisher tracker_pub1 = node.advertise<std_msgs::Float64>("lineAngle", 1000);
  ros::Subscriber sub = node.subscribe<std_msgs::Bool>("lineoff", 1000, &offCallback);
  int loopRate = 10;
  ros::Rate loop_rate(loopRate);

  bool bSuccess;
  while (ros::ok())
  {
    std_msgs::Float64 msg;

    /*
    msg.data never takes positive 90
    when the angle is 90 it will show -90
    -------------TO BE CORRECTED-------------
    */
    msg.data = -finalAngle * (180 / 3.14);

    // cout<<lineCount<<endl;
    if (lineCount > 0)
    {
      tracker_pub1.publish(msg);

      ROS_INFO("%lf", msg.data);
    }

    bSuccess = cap.read(img);  // read a new frame from video

    if (!bSuccess)  // if not success, break loop
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }

    //        StretchContrast();

    SmoothCallback(0, 0);  // for assigning imgSmooth initially
    callback(0, 0);        // for displaying the thresholded image initially

    char key = cvWaitKey(30);
    if (key == 27)
    {
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
    // while(!start){
    //  ros::spinOnce();
    //  loop_rate.sleep();
    //  ROS_INFO("Paused by taskHandler");
    // }
    if (stop)
      return 0;
  }

  destroyAllWindows();
}

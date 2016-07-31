// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>

int main(int argc, char* argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on
  int i, j, k, t1min = 29, t1max = 128, t2min = 72, t2max = 153, t3min = 133, t3max = 239, Rmin = 0,
               Rmax = 0;  // other variables used
  cv::VideoWriter output_cap(argv[2], CV_FOURCC('D', 'I', 'V', 'X'), 25, cv::Size(640, 480));

  ros::init(argc, argv, "image");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("balls", 1000);
  ros::Rate loop_rate(10);

  CvMat* threshold_matrix = cvCreateMat(2, 3, CV_32FC1);

  CvFileStorage* temp = cvOpenFileStorage("threshold_matrix.xml", NULL, CV_STORAGE_READ);

  // Load the previous values of the threshold if they exist
  if (temp)
  {
    threshold_matrix = reinterpret_cast<CvMat*>(cvLoad("threshold_matrix.xml"));
    t1min = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 0, 0));
    t2min = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 0, 1));
    t3min = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 0, 2));
    t1max = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 1, 0));
    t2max = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 1, 1));
    t3max = static_cast<int>(CV_MAT_ELEM(*threshold_matrix, float, 1, 2));
  }

  // ****************  UNCOMMENT BELOW PART TO TAKE FEED FROM CAMERA AND COMMENT OUT PART AFTER THAT*****************\\

  int camno = (**(argv + 1) - '0');
  cv::VideoCapture cap(camno);

  //*****************************************************************************************************************\\

  // VideoCapture cap(argv[1]);

  //*****************************************************************************************************************\\
    // Create a window in which the captured images will be presented
  cvNamedWindow("Contours", CV_WINDOW_NORMAL);
  cvNamedWindow("F1", CV_WINDOW_NORMAL);
  cvNamedWindow("RealPic", CV_WINDOW_NORMAL);
  cvNamedWindow("F2", CV_WINDOW_NORMAL);
  cvNamedWindow("F3", CV_WINDOW_NORMAL);

  /// Create Trackbars
  char TrackbarName1[50] = "t1min";
  char TrackbarName2[50] = "t1max";
  char TrackbarName3[50] = "t2min";
  char TrackbarName4[50] = "t2max";
  char TrackbarName5[50] = "t3min";
  char TrackbarName6[50] = "t3max";

  cvCreateTrackbar(TrackbarName1, "F1", &t1min, 260, NULL);
  cvCreateTrackbar(TrackbarName2, "F1", &t1max, 260, NULL);

  cvCreateTrackbar(TrackbarName3, "F2", &t2min, 260, NULL);
  cvCreateTrackbar(TrackbarName4, "F2", &t2max, 260, NULL);

  cvCreateTrackbar(TrackbarName5, "F3", &t3min, 260, NULL);
  cvCreateTrackbar(TrackbarName6, "F3", &t3max, 260, NULL);

  // Load threshold from the slider bars in these 2 parameters

  // capture size -
  CvSize size = cvSize(width, height);

  // Initialize different images that are going to be used in the program
  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;  // image converted to HSV plane
  // asking for the minimum distance where bwe fire torpedo

  while (1)
  {
    std_msgs::Float32MultiArray array;
    // Get one frame

    if (!cap.isOpened())
    {
      fprintf(stderr, "ERROR: frame is null...\n");
      getchar();
      break;
    }

    cv::Mat frame;
    cap >> frame;

    if (frame.empty())
      continue;

    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;
    // frame = cvQueryFrame( capture );

    cv::imshow("RealPic", frame);

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);

    cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);

    // Filter out colors which are out of range.
    cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);
    // Split image into its 3 one dimensional images
    cv::Mat thresholded_hsv[3];
    cv::split(hsv_frame, thresholded_hsv);

    // Filter out colors which are out of range.
    cv::inRange(thresholded_hsv[0], cv::Scalar(t1min, 0, 0, 0), cv::Scalar(t1max, 0, 0, 0), thresholded_hsv[0]);
    cv::inRange(thresholded_hsv[1], cv::Scalar(t2min, 0, 0, 0), cv::Scalar(t2max, 0, 0, 0), thresholded_hsv[1]);
    cv::inRange(thresholded_hsv[2], cv::Scalar(t3min, 0, 0, 0), cv::Scalar(t3max, 0, 0, 0), thresholded_hsv[2]);

    cv::imshow("F1", thresholded_hsv[0]);  // individual filters
    cv::imshow("F2", thresholded_hsv[1]);
    cv::imshow("F3", thresholded_hsv[2]);

    // Memory for hough circles
    CvMemStorage* storage = cvCreateMemStorage(0);
    cv::imshow("After Color Filtering", thresholded);  // The stream after color filtering
    // hough detector works better with some smoothing of the image
    cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);

    // find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat thresholded_Mat = thresholded;
    findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
    double largest_area = 0, largest_contour_index = 0;
    cv::imshow("Contours", thresholded_Mat);  // The stream after color filterin

    if (contours.empty())
      continue;

    for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
    {
      double a = contourArea(contours[i], false);  //  Find the area of contour
      if (a > largest_area)
      {
        largest_area = a;
        largest_contour_index = i;  // Store the index of largest contour
      }
    }
    // Convex HULL
    std::vector<std::vector<cv::Point> > hull(contours.size());
    convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

    cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
    std::vector<cv::Vec4i> hierarchy;
    cv::Scalar color(255, 255, 255);

    std::vector<cv::Rect> boundRect(1);

    boundRect[0] = boundingRect(cv::Mat(contours[largest_contour_index]));

    rectangle(Drawing, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0);

    cv::Point center;
    center.x = ((boundRect[0].br()).x + (boundRect[0].tl()).x) / 2;
    center.y = ((boundRect[0].tl()).y + (boundRect[0].br()).y) / 2;

    float Ratio_Verical_constant =
        -(static_cast<float>((boundRect[0].br()).x) - (boundRect[0].tl()).x) / ((boundRect[0].tl()).y
            - (boundRect[0].br()).y);

    printf("\n");
    drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);
    output_cap.write(frame);

    cv::Mat frame_mat = frame;
    circle(frame_mat, center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
    array.data.push_back((320 - center.x));
    array.data.push_back((240 - center.y));
    pub.publish(array);

    cv::imshow("HSV", hsv_frame);  // Original stream in the HSV color space
    cv::imshow("Contours", Drawing);
    cv::imshow("RealPic", frame_mat);

    // cvShowImage( "filtered", thresholded );

    cvReleaseMemStorage(&storage);

    ros::spinOnce();
    // loop_rate.sleep();

    // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    // remove higher bits using AND operator
    if ((cvWaitKey(10) & 255) == 27)
      break;
  }

  output_cap.release();
  return 0;
}

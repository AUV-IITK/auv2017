#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;

using namespace cv;

/** @function main */
int main(int argc, char** argv)
{
  Mat src, src_gray;

  /// Read the image
  src = imread( argv[1], 1 );

  if( !src.data )
    { return -1; }

  /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  //imshow("src_gray",src_gray);

  vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
  //Mat m1 = Mat(src.rows,src.cols, CV_64F, cvScalar(0.));
  Mat m1 = Mat(src.rows,src.cols, CV_64F, double(100));
  cout << circles.size() << endl;
  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      int b,g,r;
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      b=src.at<cv::Vec3b>(cvRound(circles[i][1]),cvRound(circles[i][0]))[0];
      g=src.at<cv::Vec3b>(cvRound(circles[i][1]),cvRound(circles[i][0]))[1];
      r=src.at<cv::Vec3b>(cvRound(circles[i][1]),cvRound(circles[i][0]))[2];
      cout << b <<" " << g <<" "<< r << endl;
      // circle center  
      circle( m1, center, 3, Scalar(1,0,0), -1, 8 );
      circle( src, center, 3, Scalar(0,255,255), -1, 8, 0 );
      // circle outline
      circle( m1, center, radius, Scalar(100,100,0), 3, 8 );
      circle( src, center, radius, Scalar(255,255,255), 3, 8, 0 );
   }

  /// Show your results
  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", src );
  //namedWindow( "black window", CV_WINDOW_AUTOSIZE );
  imshow( "blank window", m1 );

  waitKey(0);
  return 0;
}
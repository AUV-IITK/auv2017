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


int erosion_elem = 0;
int erosion_size = 0;
int const max_elem=2;
int const max_kernel_size=21;


void Erosion( int, void* )
{
  
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( src, erosion_dst, element );
  imshow( "Erosion Demo", erosion_dst );

}


int main(int argc,char **argv){

	Mat frame1,frame2,frame3,frame4;

	frame1=cv::imread(argv[1],1);
	cv::imshow("frame1",frame1);

	//cv::cvtColor(frame1,frame1,CV_BGR2HSV);

	cv::Mat frame_array[3];
	cv::Mat dst_array;
	cv::split(frame1,frame_array);

	equalizeHist( frame_array[0], frame_array[0] );
	equalizeHist( frame_array[1], frame_array[1] );
	equalizeHist( frame_array[2], frame_array[2] );

	cv::merge(frame_array,3,dst_array);
	//dst=dst_array;
	cv::imshow("dst_array",dst_array);
	cv::imshow("final_image",dst_array);
	
	src=dst_array;
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",&erosion_elem, max_elem,Erosion );
	createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",&erosion_size, max_kernel_size,Erosion );
	Erosion(0,0);

	cv::waitKey(0);
	return 0;

}


#include <cv.h>
#include <highgui.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
      

    int height,width,step,channels;  //parameters of the image we are working on
    int i,j,k,t1min=29,t1max=128,t2min=72,t2max=153,t3min=133,t3max=239,Rmin=0,Rmax=0; // other variables used
    VideoWriter output_cap(argv[2],CV_FOURCC('D','I','V','X'),25,Size(640,480));

    ros::init(argc, argv, "image");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("balls", 1000);
    ros::Rate loop_rate(10);  
     
    CvMat* threshold_matrix  = cvCreateMat(2,3,CV_32FC1);
      
    CvFileStorage* temp = cvOpenFileStorage("threshold_matrix.xml",NULL,CV_STORAGE_READ);
    
    // Load the previous values of the threshold if they exist
    if(temp)
    {
        threshold_matrix = (CvMat*)cvLoad("threshold_matrix.xml");
        t1min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0) ;t2min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1) ;t3min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0) ;t2max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1) ;t3max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2) ;
    }
    
   //int camno=(**(argv+1)-'0');

   //CvCapture* capture = cvCaptureFromCAM( camno);
   CvCapture* capture =cvCreateFileCapture( argv[1]);

    IplImage* frame = cvQueryFrame( capture );
    
    // Create a window in which the captured images will be presented
    //cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "Circle", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F1", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "RealPic", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F2", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F3", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "EdgeDetection", CV_WINDOW_AUTOSIZE );
    
    /// Create Trackbars
     char TrackbarName1[50]="t1min";
     char TrackbarName2[50]="t1max";
     char TrackbarName3[50]="t2min";
     char TrackbarName4[50]="t2max";
     char TrackbarName5[50]="t3min";
     char TrackbarName6[50]="t3max";
     char TrackbarName7[50]="Rmin";
     char TrackbarName8[50]="Rmax";
 
      cvCreateTrackbar( TrackbarName1, "F1", &t1min, 260 , NULL );
      cvCreateTrackbar( TrackbarName2, "F1", &t1max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName3, "F2", &t2min, 260 , NULL );
      cvCreateTrackbar( TrackbarName4, "F2", &t2max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName5, "F3", &t3min, 260 , NULL );
      cvCreateTrackbar( TrackbarName6, "F3", &t3max, 260,  NULL  );

     // cvCreateTrackbar( TrackbarName7, "Camera", &Rmin, 260 , NULL );
      //cvCreateTrackbar( TrackbarName8, "Camera", &Rmax, 260,  NULL  );
    
   // Load threshold from the slider bars in these 2 parameters
    CvScalar hsv_min = cvScalar(t1min, t2min, t3min, 0);
    CvScalar hsv_max = cvScalar(t1max, t2max ,t3max, 0);
    
    // get the image data
    height    = frame->height;
    width     = frame->width;
    step      = frame->widthStep;
      
     // capture size - 
    CvSize size = cvSize(width,height);

    // Initialize different images that are going to be used in the program
    IplImage*  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
    IplImage*  thresholded1   = cvCreateImage(size, IPL_DEPTH_8U, 1); // Component image threshold
    IplImage*  thresholded2   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  thresholded3   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  filtered   = cvCreateImage(size, IPL_DEPTH_8U, 1);  //smoothed image
    //asking for the minimum distance where bwe fire torpedo
    int D;
    cin >>D;
    
    while( 1 )
    {   
        // Load threshold from the slider bars in these 2 parameters
        hsv_min = cvScalar(t1min, t2min, t3min, 0);
        hsv_max = cvScalar(t1max, t2max ,t3max, 0);
   
        std_msgs::Float32MultiArray array;
        // Get one frame
      
        frame = cvQueryFrame( capture );
     
        cvShowImage( "RealPic", frame );
        if( !frame )
        {
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }
        
        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
        
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

        // Memory for hough circles
        CvMemStorage* storage = cvCreateMemStorage(0);
        cvShowImage( "After Color Filtering", thresholded ); // The stream after color filtering
        // hough detector works better with some smoothing of the image
        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
        
        //find contours
        vector<vector<Point> > contours;
        Mat thresholded_Mat=thresholded;
        findContours(thresholded_Mat, contours,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
        double largest_area=0,largest_contour_index=0;
        imshow( "Contours", thresholded_Mat ); // The stream after color filterin 

        if(contours.empty()) continue;

        for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
        {
           double a=contourArea( contours[i],false);  //  Find the area of contour
           if(a>largest_area)
           {
             largest_area=a;
             largest_contour_index=i;                //Store the index of largest contour
           }
        }  
        //Convex HULL
        vector<vector<Point> >hull( contours.size() );
        convexHull( Mat(contours[largest_contour_index]), hull[largest_contour_index], false );
  
        output_cap.write(frame);
      
      
       
        vector<Point2f>center(1);
        vector<float>radius(1);
        minEnclosingCircle(contours[largest_contour_index],center[0],radius[0]);    
  
        Mat circles=frame;
        Point2f pt;
        pt.x=320; //size of my screen
        pt.y=240;
        float distance;
        float* p;  //array to publish
               
        distance=pow(radius[0]/7526.5,-.92678);  //function found using experiment
        
        circle(circles,center[0],radius[0],Scalar(0,250,0),1,8,0);  //minenclosing circle 
        circle(circles,center[0],4,Scalar(0,250,0),-1,8,0); //center is made on the screen
        circle(circles,pt,4,Scalar(150,150,150),-1,8,0);  //center of screen
        array.data.push_back(radius[0]); //publish radius
        array.data.push_back((320-center[0].x)); 
        array.data.push_back((240-center[0].y));
        array.data.push_back(distance);
        if(((320-center[0].x>-5)&&(320-center[0].x<5))&&((240-center[0].y>-5)&&(240-center[0].y)<5)) array.data.push_back(1);  //telling we are in line of center of ball
        if(distance<D) array.data.push_back(1);
        else array.data.push_back(0);
        pub.publish(array);
     
        imshow( "circle", circles ); // Original stream with detected ball overlay
         
        //cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
         
         cvShowImage( "F1", thresholded1 ); // individual filters
         cvShowImage( "F2", thresholded2 );
         cvShowImage( "F3", thresholded3 );
         
         
        //cvShowImage( "filtered", thresholded );
        
        cvReleaseMemStorage(&storage);
        
        ros::spinOnce();
        // loop_rate.sleep();

        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cvWaitKey(10) & 255) == 27 ) break;
        
    }
     
     

   output_cap.release(); 
   return 0;
  }
  
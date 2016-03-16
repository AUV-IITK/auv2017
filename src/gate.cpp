#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
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
    int i,j,k,t1min=0,t1max=197,t2min=102,t2max=260,t3min=56,t3max=231,Rmin=0,Rmax=0; // other variables used
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
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    //int camno=(**(argv+1)-'0');

    //CvCapture* capture = cvCaptureFromCAM( camno);
    CvCapture* capture =cvCreateFileCapture( argv[1]);

    IplImage* frame = cvQueryFrame( capture );
    

    
    // Create a window in which the captured images will be presented

    cvNamedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F1", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "RealPic", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F2", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F3", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
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
    
    while( 1 )
    {   
        // Load threshold from the slider bars in these 2 parameters
        hsv_min = cvScalar(t1min, t2min, t3min, 0);
        hsv_max = cvScalar(t1max, t2max ,t3max, 0);
   
        std_msgs::Float32MultiArray array;
        // Get one frame
      
        frame= cvQueryFrame( capture );
        
        //IplImage* new_img = cvCreateImage(cvSize(640,480),frame->depth, frame->nChannels);
        //cvResize(frame,new_img);

        height    = frame->height;
        width     = frame->width;
        step      = frame->widthStep;
        
        if( !frame )
        {
                fprintf( stderr, "There is no frame being read\n" );
                getchar();
                break;
        }
       
        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame,hsv_frame, CV_BGR2HSV);
        
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
        //Split image into its 3 one dimensional images
        cvSplit( hsv_frame,thresholded1, thresholded2, thresholded3, NULL );
      
        // Filter out colors which are out of range.
        cvInRangeS(thresholded1,cvScalar(t1min,0,0,0) ,cvScalar(t1max,0,0,0) ,thresholded1);
        cvInRangeS(thresholded2,cvScalar(t2min,0,0,0) ,cvScalar(t2max,0,0,0) ,thresholded2);
        cvInRangeS(thresholded3,cvScalar(t3min,0,0,0) ,cvScalar(t3max,0,0,0) ,thresholded3);
        
        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------
    
    
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

        Mat Drawing(thresholded_Mat.rows,thresholded_Mat.cols,CV_8UC1,Scalar::all(0));  
        vector<Vec4i> hierarchy;
        Scalar color(255,255,255);
        
        vector<Rect> boundRect(1);
        
        boundRect[0] = boundingRect( Mat(contours[largest_contour_index]) );
        
        rectangle( Drawing, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0 );

        Point center ;
        center.x = ((boundRect[0].br()).x +(boundRect[0].tl()).x)/2;
        center.y = ((boundRect[0].tl()).y +(boundRect[0].br()).y)/2;
        
        float Ratio_Verical_constant= -((float)(boundRect[0].br()).x -(boundRect[0].tl()).x)/((boundRect[0].tl()).y -(boundRect[0].br()).y);

        printf("\n");
        drawContours(Drawing,contours,largest_contour_index,color,2,8,hierarchy);
        output_cap.write(frame);
        
      
               
       
        /// Get the moments 
        /*vector<Moments> mu(contours.size() );
         mu[largest_contour_index] = moments( contours[largest_contour_index], false ); 

        ///  Get the mass centers:
        vector<Point2f> mc( contours.size() );
        mc[largest_contour_index] = Point2f( mu[largest_contour_index].m10/mu[largest_contour_index].m00 , mu[largest_contour_index].m01/mu[largest_contour_index].m00 );
  */      Mat frame_mat=frame;
    /*    float* p;
         printf("%f\n",mc[largest_contour_index].x);          
        circle(frame_mat,mc[largest_contour_index],5,Scalar(0,250,0),1,8,1);
        */   
        printf("%f\n",Ratio_Verical_constant);

        if((Ratio_Verical_constant>1.10)&&(Ratio_Verical_constant<1.12))
        {
        	printf("We are in Center\n");
        }
        
        if(Ratio_Verical_constant<1.1)
        {
        	printf("Move to Left\n");
        }	
        if(Ratio_Verical_constant>1.12)
        {
        	printf("Move To Right\n");
        }	


        circle(frame_mat,center,5,Scalar(0,250,0),-1,8,1);
        array.data.push_back((320-center.x));
        array.data.push_back((240-center.y));
        pub.publish(array);
        
        cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
        cvShowImage( "F1", thresholded1 ); // individual filters
        cvShowImage( "F2", thresholded2 );
        cvShowImage( "F3", thresholded3 );
        imshow( "Contours",Drawing );
        imshow( "RealPic", frame_mat );
         
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
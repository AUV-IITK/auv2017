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

int lineThresh=60;
int minLineLength=70;
int maxLineGap=10;


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
   
    //int camno=(**(argv+1)-'0'); //to get feed from webcam

    //CvCapture* capture = cvCaptureFromCAM( camno);
    CvCapture* capture =cvCreateFileCapture( argv[1]);

    IplImage* frame = cvQueryFrame( capture );
    Mat test=frame;
    if ( test.empty() ) {printf("ERROR CANT OPEN FILE"); return ( 1 );}

    
    // Create a window in which the captured images will be presented

 //   cvNamedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F1", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "RealPic", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F2", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F3", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
  // cvNamedWindow( "lines", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Canny", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Output",CV_WINDOW_AUTOSIZE);
    /// Create Trackbars
     char TrackbarName1[50]="t1min";
     char TrackbarName2[50]="t1max";
     char TrackbarName3[50]="t2min";
     char TrackbarName4[50]="t2max";
     char TrackbarName5[50]="t3min";
     char TrackbarName6[50]="t3max";
     char TrackbarName7[50]="lineThresh";
     char TrackbarName8[50]="minLineLength";
     char TrackbarName9[50]="maxLineGap";
 
      cvCreateTrackbar( TrackbarName1, "F1", &t1min, 260 , NULL );
      cvCreateTrackbar( TrackbarName2, "F1", &t1max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName3, "F2", &t2min, 260 , NULL );
      cvCreateTrackbar( TrackbarName4, "F2", &t2max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName5, "F3", &t3min, 260 , NULL );
      cvCreateTrackbar( TrackbarName6, "F3", &t3max, 260,  NULL  );

      cvCreateTrackbar( TrackbarName7, "Output", &lineThresh, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName8, "Output", &minLineLength, 260 , NULL );
      cvCreateTrackbar( TrackbarName9, "Output", &maxLineGap, 260,  NULL  );

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
        cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space

        
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
        //Split image into its 3 one dimensional images
        cvSplit( hsv_frame,thresholded1, thresholded2, thresholded3, NULL );
      
        // Filter out colors which are out of range.
        cvInRangeS(thresholded1,cvScalar(t1min,0,0,0) ,cvScalar(t1max,0,0,0) ,thresholded1);
        cvInRangeS(thresholded2,cvScalar(t2min,0,0,0) ,cvScalar(t2max,0,0,0) ,thresholded2);
        cvInRangeS(thresholded3,cvScalar(t3min,0,0,0) ,cvScalar(t3max,0,0,0) ,thresholded3);
        

        cvShowImage( "F1", thresholded1 ); // individual filters
        cvShowImage( "F2", thresholded2 );
        cvShowImage( "F3", thresholded3 );


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
        

        
        drawContours(Drawing,contours,largest_contour_index,color,2,8,hierarchy);
        imshow( "Contours",Drawing );
        output_cap.write(frame);
        
        Mat lineMat;  //one to hold the lines
        
        Canny(Drawing,lineMat, 50, 200, 3);
        imshow("Canny",lineMat);
        Mat output;
 //     HoughLinesP(lineMat,lines,1,CV_PI/180,80,30,10);  

        vector<Vec4i> lines;

        HoughLinesP(lineMat, lines, 1, CV_PI/180, lineThresh, minLineLength, maxLineGap ); 

        Mat imgLines;
        test.copyTo(imgLines);

        imgLines=Scalar(0,0,0);
        vector<double> angles(lines.size());
   
        int val=0; 

        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            line(imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
            Point p1, p2;
            p1=Point(l[0], l[1]);
            p2=Point(l[2], l[3]);
            //calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
            float angle = atan2(p1.y - p2.y, -p1.x + p2.x)*180/CV_PI;
            
            //printf("%f %d %d %d %d\n",angle,p1.y, p2.y,p2.x, p1.x);
            
            /*int AV=(-95<angle)&&(angle<-85); //center
            int BV=(95<angle)&&(angle<85); //center
            int CV=(-90<angle)&&(angle<-85);
            */
            
            //assuming we are not in extreme right or left
            if ((angle>75)||(angle<-75)) continue; 
            int AH=(-5<angle)&&(angle<5); //center
            int BH =(-5>angle); //Move right we are in left
            int CH =(5<angle);  //Move left we are in right
            if(AH) val=1;
            if(BH) val=2;
            if(CH) val=3;
        }
        switch(val)
        {
            case 0:
            printf("Lost in the sea :(\n");break;
            case 1:
            printf("YEAH! Move forward\n");break;
            case 2:
            printf("Take a left turn commander\n");break;
            case 3:
            printf("Take a right turn commander\n");break;
        }       

//        printf("shit 1\n");
        imshow("Output",imgLines+test);        
        Mat frame_mat=frame;
//        imshow( "RealPic", frame_mat );






//        circle(frame_mat,center,5,Scalar(0,250,0),-1,8,1);
//        array.data.push_back((320-center.x));
//        array.data.push_back((240-center.y));
//       pub.publish(array);
        
        
        ros::spinOnce();

        if( (cvWaitKey(10) & 255) == 27 ) break;
        
    }
     
     

   output_cap.release(); 
   return 0;
  }
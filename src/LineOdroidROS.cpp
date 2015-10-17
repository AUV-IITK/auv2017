#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fstream>

using namespace cv;
using namespace std;

Mat img,imgHSV,imgThresholded,imgSmooth,imgCanny,imgLines;
int lineCount=0;

//parameters in param file should be nearly the same as the commented values
//params for an orange strip
int LowH ; //0
int HighH ; //88

int LowS ; //0 
int HighS ; //251

int LowV ; //0
int HighV ; //255

//params for smoothing image
int ksize; //21
int stype; //4  -> decides which type of filter has to be used 

//params for bilateral filter in smoothcallback
int sigmaColor; //150
int sigmaSpace; //10

//params for hough line transform
int lineThresh; //60
int minLineLength; //70
int maxLineGap; //10
int houghThresh; //15

double rho=0.1;
double finalAngle;
double minDeviation=0.02;

void StretchContrast()
{
	Mat ch[3];
	split(img,ch);
	normalize(ch[1],ch[1],255,0,NORM_MINMAX);
	normalize(ch[2],ch[2],255,0,NORM_MINMAX);
	normalize(ch[0],ch[0],255,0,NORM_MINMAX);
	merge(ch,3,img);
}

//called when large no of line detected
double computeMean(vector<Vec2f>& newRhoAngle){
	double sum=0;
	for( size_t i = 0; i < newRhoAngle.size(); i++ ){
    	sum=sum+newRhoAngle[i][1];    		
    }
    return sum/newRhoAngle.size();
}

//called when few lines are detected 
//to remove errors due to any stray results
double computeMode(vector<Vec2f>& newRhoAngle){
	double mode=newRhoAngle[0][1];
	int freq=1;
	int tempFreq;
	double diff;
	for(int i=0;i<newRhoAngle.size();i++){
		tempFreq=1;

		for(int j=i+1;j<newRhoAngle.size();j++){
			diff=newRhoAngle[j][1]-newRhoAngle[i][1]>0.0? newRhoAngle[j][1]-newRhoAngle[i][1]:newRhoAngle[i][1]-newRhoAngle[j][1];
			if(diff<=minDeviation){
				tempFreq++;
				newRhoAngle.erase(newRhoAngle.begin()+j);
				j=j-1;
			}
		}

		if(tempFreq>=freq){
			mode=newRhoAngle[i][1];
			freq=tempFreq;
		}
	}

	return mode;
}

//callback for smoothing image
//contains different filters "stype" decides which filter to use
void SmoothCallback(int ,void *){
	if(ksize%2==0) ksize=ksize+1;     //kernel size can not be even
	switch(stype){
		case 1:
			blur( img, imgSmooth, Size( ksize, ksize ) ); //ksize=17 
			break;
		case 2:
			GaussianBlur( img, imgSmooth, Size( ksize, ksize ), 0, 0 );
			break;
		case 3:
			medianBlur( img, imgSmooth, ksize );
			break;
		case 4:
			bilateralFilter( img, imgSmooth, ksize, sigmaColor, sigmaSpace);
			break;
		default:
			cout<<"Please enter a smoothing type "<<endl;
	}

}

//contains canny edge detection and houghline transform
void callback(int ,void *){
	cvtColor(imgSmooth, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), imgThresholded); //Threshold the image
      
	//opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	Canny(imgThresholded,imgCanny,50,150,3,true);//canny edge detection

	vector<Vec4i> lines;

	HoughLinesP(imgCanny, lines, 1, CV_PI/180, lineThresh, minLineLength, maxLineGap );	

	img.copyTo(imgLines);
	imgLines=Scalar(0,0,0);

	for( size_t i = 0; i < lines.size(); i++ )
	{
  		Vec4i l = lines[i];
  		line( imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
	}

	cout<<"No of lines initially: "<<lines.size()<<endl;
	imshow("LINES",imgLines+img);

	cvtColor(imgLines, imgLines, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	inRange(imgLines, Scalar(59, 0, 0), Scalar(61, 255, 255), imgLines); //Threshold the image
	Canny(imgLines,imgLines,50,150,3,true);//canny edge detection
	vector<Vec2f> rhoAngle;
    HoughLines(imgLines, rhoAngle, rho, CV_PI/180, houghThresh, 0, 0 );
 
    cout<<"No of lines finally: "<<rhoAngle.size()<<" "<<", Angles(radian): ";	
    for( size_t i = 0; i < rhoAngle.size(); i++ ){
    	cout<<rhoAngle[i][1]<<" ";
    	rhoAngle[i][1]=rhoAngle[i][1]>1.57? -1*(3.14-rhoAngle[i][1]):rhoAngle[i][1];    		
    } 
    cout<<endl;
    lineCount=rhoAngle.size();
    //num of lines :-large ->mean is computed
    //num of lines :-small ->mode is computed -> removes error due to lines
    if(rhoAngle.size()>0 && rhoAngle.size()<10) finalAngle=computeMode(rhoAngle);
    else if(rhoAngle.size()>0) finalAngle=computeMean(rhoAngle);

    //if(finalAngle >1.57) finalAngle= -1*(3.14-finalAngle );

    cout<<"Final Angle: "<<finalAngle<<endl;

	//imshow("check",imgLines);
}

int main( int argc, char** argv ) {

	//enter the path which contains params.txt
	FILE* fp=fopen("/home/shibhansh/catkin_ws/src/linefollowing/src/params.txt","r"); 
	fscanf(fp,"%d %d %d %d %d %d\n%d %d %d %d\n%d %d %d\n%d",&LowH,&HighH,&LowS,&HighS,&LowV,&HighV,&ksize,&stype,&sigmaSpace,&sigmaColor,&lineThresh,&minLineLength,&maxLineGap,&houghThresh);
	fclose(fp);

    //VideoCapture cap(0); //capture the video from webcam
    VideoCapture cap("/home/shibhansh/catkin_ws/src/linefollowing/src/outputnorm.avi"); //path of the video for checking the code 

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot get the stream" << endl;
         return -1;
    }

    namedWindow("LINES",CV_WINDOW_AUTOSIZE);

    ros::init(argc, argv, "LineOdroidROS");
	ros::NodeHandle node;
 	ros::Publisher tracker_pub = node.advertise<std_msgs::Float64>("lineAngle", 1000);	

 	finalAngle=0.0;

    bool bSuccess;
    while(ros::ok()){

    	std_msgs::Float64 msg;
    	msg.data = finalAngle;

    	cout<<lineCount<<endl;

  		if(lineCount>0){
		tracker_pub.publish(msg);
		ROS_INFO("%lf", msg.data);
		}
		
		ros::spinOnce();

    	bSuccess = cap.read(img); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        StretchContrast();

        SmoothCallback(0,0); //for assigning imgSmooth initially
		callback(0,0); //for displaying the thresholded image initially		

    	char key=cvWaitKey(30);
		if(key==27){
			break;
		}
    }

    destroyAllWindows();
}

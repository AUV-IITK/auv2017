// Copyright 2016 AUV-IITK
#include  "Filters.hpp"

void Filters::SimplestCB(cv::Mat& in, cv::Mat& out, float percent)
{
    assert(in.channels() == 3);
    assert(percent > 0 && percent < 100);
    float half_percent = percent / 200.0f;
    std::vector<cv::Mat> tmpsplit; split(in, tmpsplit);
    for( int i = 0; i<3; i++ ) 
    {
        // find the low and high precentile values (based on the input percentile)
        cv::Mat flat; tmpsplit[i].reshape(1,1).copyTo(flat);
        cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        int lowval = flat.at<uchar>(cvFloor((std::static_case<float>flat.cols) * half_percent));
        int highval = flat.at<uchar>(cvCeil(((float)flat.cols) * (1.0 - half_percent)));
        // saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval, tmpsplit[i] > highval);
        // scale the channel
        cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
    }
    cv::merge(tmpsplit, out);
}
	
void Filters::equalizeHistogram(cv::Mat &src,cv::Mat &dst)
{
  	cv::Mat frame1,frame2,frame3,frame4;

	frame1=src;
	cv::imshow("frame1",frame1);

	cv::cvtColor(frame1,frame1,CV_BGR2HSV);

	cv::Mat frame_array[3];
	cv::Mat dst_array;
	cv::split(frame1,frame_array);

	equalizeHist( frame_array[0], frame_array[0] );
	equalizeHist( frame_array[1], frame_array[1] );
	equalizeHist( frame_array[2], frame_array[2] );

	cv::merge(frame_array,3,dst_array);
	dst=dst_array;
	cv::imshow("dst_array",dst_array);
	cv::imshow("final_image",dst_array);
	cv::waitKey(0);
	return ;
}

class Filters::Erode
{
public:
	int erosion_elem;
	int erosion_size;
	int const max_elem=2;
	int const max_kernel_size=21;


	Erode(int size,int elem){

			erosion_elem=elem;
			erosion_size=size;
	}

	~Erode(){};

	void Erosion( int, void* )
	{
  		int erosion_type;
  		if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
  		else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
  		else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

  		cv::Mat element =cv::getStructuringElement( erosion_type,cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),cv::Point( erosion_size, erosion_size ) );
  		/// Apply the erosion operation
  		erode( dst_array, dst_array, element );
  		imshow( "Erosion Demo", dst_array );
	}

	void apply_erosion(cv::Mat &src)
	{
  		cv::namedWindow("Erosion Demo",CV_WINDOW_AUTOSIZE);
		cv::createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",&erosion_elem, max_elem,Erosion );
		cv::createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",&erosion_size, max_kernel_size,Erosion);
		Erosion(0,0);
	}



};

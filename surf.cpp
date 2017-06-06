#include <stdio.h>
#include <opencv2/calib3d/claib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace std;
using namespace cv;

int main(int argc,char **argv)
{

	Mat image1;
	Mat image2;

	image1=cv::imread(argv[1],1);             //read the input images
	image2=cv::imread(argv[2],1);

	vector<cv::KeyPoint> kp1;                 
	vector<cv::KeyPoint> kp2;
	
	std::vector<Point2f> obj_corners(4);

	///////////get the obj_scene corners////////
	
	obj_corners[0]=(cvPoint(0,0));
	obj_corners[1]=(cvPoint(image1.cols,0));
	obj_corners[2]=(cvPoint(image1.cols,image1.rows));
	obj_corners[3]=(cvPoint(0,image1.rows));


    	Mat des1, des2;

	SurfFeatureDetector detector(500);            //get the keypoints
	detector.detect(image1,kp1);
	detector.detect(image2,kp2);

	SurfDescriptorExtractor extractor;       //get the descriptors

	extractor.compute(image1,kp1,des1);
	extractor.compute(image2,kp2,des2); 	

    	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce"); //define matcher object
	std::vector< std::vector< DMatch > >  matches;                            //define DMatch object to store matches
        std::vector< std::vector< DMatch > >  matches1;                            //define DMatch object to store matches

    	matcher->knnMatch(des1,des2, matches,2);
        matcher->knnMatch(des2,des1, matches1,2);

    	std::vector<DMatch> goodMatches;
	std::vector<DMatch> goodMatches1;

   	for(int i=0;i<des1.rows;i++)                                       //ratio test to remove poor matches
	{
		const DMatch &m = matches[i][0];
		const DMatch &n = matches[i][1];
		if(m.distance<0.75*n.distance)
			goodMatches.push_back(m);
		else if(n.distance<0.75*m.distance)
			goodMatches.push_back(n);

	}
        for(int i=0;i<des2.rows;i++)                                       //ratio test to remove poor matches
	{
		const DMatch &m = matches1[i][0];
		const DMatch &n = matches1[i][1];
		
		if(m.distance<0.75*n.distance)
			goodMatches1.push_back(m);
		else if(n.distance<0.75*m.distance)
			goodMatches1.push_back(n);
	}
	//std::vector<cv::DMatch> symMatches;
    
 /* for (vector<DMatch>::const_iterator matchIterator1= goodMatches.begin();matchIterator1!= goodMatches.end(); ++matchIterator1)

  {

    for (vector<DMatch>::const_iterator matchIterator2= goodMatches1.begin();matchIterator2!= goodMatches1.end();++matchIterator2)

    {

      if ((*matchIterator1).queryIdx ==(*matchIterator2).trainIdx &&(*matchIterator2).queryIdx ==(*matchIterator1).trainIdx)

      {

        symMatches.push_back(DMatch((*matchIterator1).queryIdx,(*matchIterator1).trainIdx,(*matchIterator1).distance));

        break;

      }

    }

  }*/
   

    	Mat image3;
    	std::vector<Point2f> obj;
    	std::vector<Point2f> scene;
    	std::vector<Point2f> scene_corners(4);
    //drawMatches(image1,kp1,image2,kp2,goodMatches,image3,2);
	if(goodMatches.size()>=4)
	{
		for(int i=0;i<goodMatches.size();i++)
		{
			obj.push_back(kp1[good_matches[i].queryIdx].pt);
			scene.push_back(kp2[good_matches[i].trainIdx].pt);
		}
		
		H=findHomography(obj,scene,CV_RANSAC);
		
		perspectiveTransform(obj_corners,scene_corners,H)

		line(image2,scene_corners[0],scene_corners[1],Scalar(0,255,0),4);
		line(image2,scene_corners[1],scene_corners[2],Scalar(0,255,0),4);
		line(image2,scene_corners[2],scene_corners[3],Scalar(0,255,0),4);
		line(image2,scene_corners[3],scene_corners[0],Scalar(0,255,0),4);

			
	}

 	cv::namedWindow("image",CV_WINDOW_NORMAL);
	cv::imshow("image",image2);
	cv::waitKey(0);

	return 0;



}


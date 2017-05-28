#include <stdio.h>

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

    Mat des1, des2;



    SurfFeatureDetector surf(500);            //get the keypoints

    surf.detect(image1,kp1);

    surf.detect(image2,kp2);



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

    std::vector<cv::DMatch> symMatches;
    
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

    drawMatches(image1,kp1,image2,kp2,goodMatches,image3,2);



 	cv::namedWindow("image",CV_WINDOW_NORMAL);

    cv::imshow("image",image3);

    cv::waitKey(0);



    return 0;



}


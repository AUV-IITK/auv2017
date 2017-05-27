#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
   
    cv::VideoCapture cap1, cap2;
    cv::Mat frame1, image1,frame2 , image2;
    sensor_msgs::ImagePtr msg;
     sensor_msgs::ImagePtr msg1;

     cap1.open(0);
    if ( !cap1.isOpened() )
    {
        std::cout << "Could not initialize cap1" << std::endl;
        return -1;
    }
   
    ros::Rate loop_rate(5);
    cv::waitKey(30);
    while (ros::ok()) {
        cap1 >> frame1;
        if ( frame1.empty() )
            break;
        frame1.copyTo(image1);
        cap2 >> frame2;
  
        if(!image1.empty()){
        cv::imshow( "Img1", image1 );}
        char c = (char)cv::waitKey(10);
        if ( c == 27 )
            break;

     //   cv::Mat image = cv::imread("/home/karmesh/arnav.jpg", CV_LOAD_IMAGE_COLOR);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();

        //sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
       }
    }

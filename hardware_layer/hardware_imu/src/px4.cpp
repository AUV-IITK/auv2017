   #include "ros/ros.h"
   #include <sensor_msgs/Imu.h>
   #include <std_msgs/Float64.h>

   #define TO_DEG(x) (x * 57.2957795131)
   std_msgs::Float64 imu_data;
   ros::Publisher imu_data_pub;

   void imu_data_callback(sensor_msgs::Imu msg)
   {
     float q0 = msg.orientation.w;
     float q1 = msg.orientation.x;
     float q2 = msg.orientation.y;
     float q3 = msg.orientation.z;
     imu_data.data = TO_DEG(atan2(2*q1*q2-2*q0*q3, 2*q0*q0+2*q1*q1-1));
     imu_data_pub.publish(imu_data);
   }

   int main(int argc, char **argv)
   {

     ros::init(argc, argv, "px4");

     ros::NodeHandle nh;

     ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, &imu_data_callback);

     imu_data_pub = nh.advertise<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000);

     ros::spin();

     return 0;
   }

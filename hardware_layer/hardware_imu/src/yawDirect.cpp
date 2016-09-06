// Copyright 2016 AUV-IITK
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <time.h>
#include "caliberation"

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

void read_sensors()
{
  Read_Gyro();   // Read gyroscope
  Read_Accel();  // Read accelerometer
  Read_Magn();   // Read magnetometer
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu");

  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000);
  std_msgs::Float64 msg;

  ros::Rate loopRate(10);

  int temp = 0;

  removegyrooff();

  while (ros::ok())
  {
    read_sensors();
    msg.data = TO_DEG(-atan2(magnetom[1], magnetom[0]));
    chatter_pub.publish(msg);
    ROS_INFO("%s %f", "yawDirect: send an imu message", TO_DEG(-atan2(magnetom[1], magnetom[0])));
    ros::spinOnce();
    loopRate.sleep();
    temp++;
  }
}

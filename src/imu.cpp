#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <linefollowing/navstik>
#include <sstream>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  NavStik ns;
  ros::init(argc, argv, "imu");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("imu", 1000);
  ros::Rate loop_rate(100);
  
  while (ros::ok())
  {
    std_msgs::Float64 msg;
    msg.data = ns.yaw();

    ROS_INFO("%lf", msg.data);
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}


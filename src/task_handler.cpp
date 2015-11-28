#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "linefollowing/line_arduino.h"
double lineangle=0.0;
double imuangle=0.0;
double setpoint=imuangle-lineangle;
linefollowing::line_arduino output;
double mod(double a){
	if(a>0)
		return a;
	else
		return -a;
}
// void linecb(std_msgs::Float64 msg){
// 	if(mod(setpoint-(imuangle-msg.data))>0.2){
// 		lineangle=msg.data;
// 		output.reset=true;
// 		setpoint=imuangle-lineangle;
// 	ROS_INFO("DATA FROM IP RECEIVED %lf AND RESET ",msg.data);
// 	}
// 	ROS_INFO("DATA FROM IP RECEIVED %lf",msg.data);
// }
void imucb(std_msgs::Float64 msg1){
	imuangle=msg1.data;
	ROS_INFO("DATA FROM IMU RECEIVED %lf",msg1.data);
}


int main(int argc,char ** argv){
	ros::init(argc,argv,"task_handler");
	ros::NodeHandle n;
	ros::Publisher final_angle=n.advertise<linefollowing::line_arduino>("final_angle",1000);
//	ros::Subscriber ip=n.subscribe<std_msgs::Float64>("lineAngle",1000,&linecb);
	ros::Subscriber imu=n.subscribe<std_msgs::Float64>("imu",10,&imucb);
	setpoint=imuangle-lineangle;
	output.setPoint=setpoint; 
	output.currentPosition=imuangle;
	output.reset=false;
	ros::Rate looprate(1000);
	while(ros::ok()){
		output.setPoint=setpoint; 
		output.currentPosition=imuangle;
		final_angle.publish(output);
		ROS_INFO("FINAL ANGLE IS GIVEN TO THE ARDUINO NODE is %lf  ",imuangle);
		ros::spinOnce();
		looprate.sleep();
		output.reset=false;
		ros::spinOnce();
	}
	return 0;
}


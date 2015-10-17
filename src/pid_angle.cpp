#include "ros/ros.h"
#include "linefollowing/line_arduino.h"
#include <std_msgs/Int32.h>
#include <stdio.h>
bool reset=false;
double current_angle;
double previous_angle=current_angle;
double setpoint;
std_msgs::Int32 out;
void move_forward(){

}
double mod(double a){
	if(a>0)
		return a;
	else
		return -a;
}
void pid(double p=1,double i=0, double d=0){
	double integral=0;
	double derivative=0;
	double output=0;
	previous_angle=current_angle;
	double error=setpoint-current_angle;
	double change=current_angle-previous_angle;
	double dt=0.001;//assuming incoming rate to be 100 Hz
	double max=120;//depends upon micro-controller
	double min=90;
	ros::NodeHandle nh;
	ros::Publisher pwm=nh.advertise<std_msgs::Int32>("turningPWM",1000);
	ros::Rate loop_rate(1000);
	reset=false;
	while(!reset&&ros::ok()){
		integral =integral +((current_angle+previous_angle)*dt)/2;
		change=current_angle- previous_angle;
		derivative = change/dt;
		error=setpoint-current_angle;
		error=error*(180/3.14);
		output=p*error+i*integral+d*derivative;


		//out.data=255;
		

		/*if(error>0)
			output = 90+(output*3/3.14);
		else{
			output=-output;
			output = 90+(output*3/3.14);
		}*/
		output=90+mod(output);
		if(output>max)
			output=max;
		//if(output<min)
		//	output=min;
		//if(((current_angle<0.1)&&(current_angle>-0.1))&&((derivative>-0.01)&&(derivative<0.01)))
		//break;
		if(error<0)
			out.data=(int)output;
		else
			out.data=-1*(int)output;
		printf("give the value of pwm");
		scanf("%d",&out.data);
		ROS_INFO("pwm send to arduino %d",out.data);
		pwm.publish(out);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return;
}
void anglecb(linefollowing::line_arduino msg){	
	if(!(msg.reset)){	
		previous_angle=current_angle;
		current_angle=msg.currentPosition;
		ROS_INFO("NOT RESET ANGLE, TARGET REMAINS SAME %lf AND currentPosition IS %lf",(setpoint*180/3.14),msg.currentPosition*(180/3.14));
	}
	else{
		setpoint=msg.setPoint;
		current_angle=msg.currentPosition;
		reset=msg.reset;	
		ROS_INFO("NEW setpoint IS %lf AND THE NEW current_position IS %lf",msg.setPoint,current_angle);
	}
}
int main(int argc, char** argv){
	ros::init(argc,argv,"pid_angle");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe<linefollowing::line_arduino>("final_angle",1000,&anglecb);

	while(ros::ok()){
		ros::spinOnce();
		pid(1,0,0);//now it's the duty of the PID function to return only when the bot becomes stable at that position
		if(reset)
			reset=false;
		
		//else
			//move_forward();
	}
	return 0;
}

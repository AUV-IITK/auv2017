#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motionlibrary/TurnAction.h>
#include <iostream>
#define minPWM 120
using namespace std;

float presentAngularPosition=0;
float previousAngularPosition=0;
float finalAngularPosition, error, output;
bool initData =false;

std_msgs::Int32 pwm;
std_msgs::Int32 dir;//that we have to send

typedef actionlib::SimpleActionServer<motionlibrary::TurnAction> Server;

class TurnAction{
private:
	ros::NodeHandle nh_;
	Server turnServer_;
	std::string action_name_;
	motionlibrary::TurnFeedback feedback_;
	motionlibrary::TurnResult result_;


public:
	TurnAction(std::string name):
		//here we are defining the server, third argument is optional
	   	turnServer_(nh_, name, boost::bind(&TurnAction::analysisCB, this, _1), false),
    	action_name_(name)
	{
//		turnServer_.registerGoalCallback(boost::bind(&TurnAction::goalCB, this));
		turnServer_.registerPreemptCallback(boost::bind(&TurnAction::preemptCB, this));

//this type callback can be used if we want to do the callback from some specific node
//		sub_ = nh_.subscribe("name of the node", 1, &TurnAction::analysisCB, this);
		turnServer_.start();
	}

	~TurnAction(void){
	}

	void preemptCB(void){
		//this command cancels the previous goal
		turnServer_.setPreempted();
		ROS_INFO("%s: Preempted", action_name_.c_str());
	}

	void analysisCB(const motionlibrary::TurnGoalConstPtr goal){
		ROS_INFO("Inside analysisCB");

		int loopRate =10 ;
		ros::Rate loop_rate(loopRate);

		//waiting till we recieve the first value from IMU else it's useless to to any calculations
		while(!initData){
			ROS_INFO("Waiting to get first input from IMU");
			loop_rate.sleep();
		}

		finalAngularPosition = presentAngularPosition + goal->AngleToTurn;
		if(finalAngularPosition >= 180)
			finalAngularPosition = finalAngularPosition -360;
		else if(finalAngularPosition <= -180)
			finalAngularPosition = finalAngularPosition +360;
		
		float derivative=0,integral=0,dt=1.0/loopRate,p=1,i=0,d=0;
		bool reached=false;
		
		ros::Publisher PWM=nh_.advertise<std_msgs::Int32>("PWM",1000);
		ros::Publisher direction=nh_.advertise<std_msgs::Int32>("direction",1000);

		pwm.data=0;
		dir.data=5;

		if (!turnServer_.isActive())
			return;

		while(!turnServer_.isPreemptRequested()&&ros::ok()){
			error = finalAngularPosition - presentAngularPosition;
			integral+= (error*dt);
			derivative = (presentAngularPosition- previousAngularPosition)/dt;

			output= (p*error)+(i*integral)+(d*derivative);

			turningOutputPWMMapping(output);

			//this lower limit depends upon the bot itself, below these values of PWM thrusters will not start
			if(pwm.data < minPWM)
				pwm.data= minPWM;	

			feedback_.AngleRemaining = error;

			turnServer_.publishFeedback(feedback_);
			PWM.publish(pwm);
			direction.publish(dir);
			ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
			
			ros::spinOnce();
			loop_rate.sleep();

			if(error<5 && error >-5){//assuming that this angle is in degree
				//write something to calculate the time we will wait for and check if we are in the range of 5 degrees
				// we can also check that if we are in this range and the angular velocity is also small then we can assume
				// that we are stable and we can now start moving 
				reached=true;
				pwm.data = 0;
				dir.data = 5;
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("thrusters stopped");
				break;
			}			

			if (turnServer_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				turnServer_.setPreempted();
				reached = false;
				break;
			}
		}

		if(reached){
			result_.MotionCompleted = reached;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			turnServer_.setSucceeded(result_);
		}
	}
	void turningOutputPWMMapping(float output){
		float maxOutput=120, minOutput=-120,scale;
		if(output > maxOutput)
			output = maxOutput;
		if(output < minOutput)
			output = minOutput;
		scale = (2*255)/(maxOutput- minOutput);
		float temp;

		temp = output*scale;

		if(temp>0){
			pwm.data = (int)temp;
			dir.data = 3; 
		}
		else{
			pwm.data = -1*(int)temp;
			dir.data = 4;
		}
	}
};


void yawCb(std_msgs::Float64 msg){

	//this is used to set the final angle after getting the value of first intial position
	if(initData == false){
		presentAngularPosition= msg.data;
		previousAngularPosition = presentAngularPosition;

		initData = true;
	}
	else{
		previousAngularPosition = presentAngularPosition;
		presentAngularPosition= msg.data;
		// ROS_INFO("New angular postion %f", msg.data);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "TurnXY");

	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("yaw",1000,&yawCb);

	ROS_INFO("Waiting for Goal");
	TurnAction turn(ros::this_node::getName());

	ros::spin();
	return 0;
}
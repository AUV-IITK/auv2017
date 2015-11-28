#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motionlibrary/TurnAction.h>
#include <iostream>
#define minPWM 120
using namespace std;

float presentAngularPosition, previousAngularPosition;
float finalAngularPosition, error, output;
bool initData =false;
float theta;

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
//		forwardServer_.registerGoalCallback(boost::bind(&forwardAction::goalCB, this));
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

		finalAngularPosition = presentAngularPosition + goal->AngleToTurn;
		error=finalAngularPosition- presentAngularPosition;
		
		int loopRate =10 ;
		float derivative=0,integral=0,dt=1.0/loopRate,p=1,i=0,d=0;
		bool reached=false;
		
		ros::Publisher PWM=nh_.advertise<std_msgs::Int32>("PWM",1000);
		ros::Publisher direction=nh_.advertise<std_msgs::Int32>("direction",1000);
		ros::Rate loop_rate(loopRate);
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

			// //use for giving manual input
			// int temp;
			// cout << "give pwm input" << endl;
			// cin >> temp;
			// pwm.data = temp;

			//this lower limit depends upon the bot itself
			if(pwm.data < minPWM)
				pwm.data= minPWM;	

			
			PWM.publish(pwm);
			direction.publish(dir);
			ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
			
			ros::spinOnce();
			loop_rate.sleep();

			if(error<5 && error >-5){//assuming that this angle is in degree
				//write something to calculate the time we will wait for and check if we are in the range of 5 degrees
				// we can also check that if we are in this range and the angular velocity is also small then we can assume
				// that we are stable and we can now start moving forward
				reached=true;
				pwm.data = 0;
				dir.data = 5;
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("thrusters stopped");
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
		scale = 510.0/(maxOutput- minOutput);
		float temp;

		temp = output*scale;
		//pwm= (int)temp;

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
	previousAngularPosition = presentAngularPosition;
	presentAngularPosition= msg.data;
	ROS_INFO("New angular postion %f", msg.data);
	if(initData == false){
		finalAngularPosition = presentAngularPosition + theta;
		initData = true;
		if(finalAngularPosition >= 180)
			finalAngularPosition = finalAngularPosition -360;
		else if(finalAngularPosition <= -180)
			finalAngularPosition = finalAngularPosition +360;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "forward");
	// input theta will be positive if we have to rotate the bot ACW
	if(argc!=2){
		cout << "incorret number of arguments" << endl;
		return 1;
	}
	else
		theta=atof(argv[1]);
	cout << "theta is " <<argv[1] << endl;

	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("yaw",1000,&yawCb);

	ROS_INFO("Waiting for Goal");
	TurnAction forward(ros::this_node::getName());

	ros::spin();
	return 0;
}
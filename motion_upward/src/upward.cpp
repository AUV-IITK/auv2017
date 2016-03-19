#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_actions/UpwardAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_upward/pidConfig.h>
#define minPWM 120
typedef actionlib::SimpleActionServer<motion_actions::UpwardAction> Server; // defining the Client type

float presentHeight=0;
float previousHeight=0;
float finalHeight, error, output;
bool initData =false;

std_msgs::Int32 pwm; // pwm to be send to arduino
std_msgs::Int32 dir; // dir to be send to arudino

// new inner class, to encapsulate the interaction with actionclient
class innerActionClass{
private:
	ros::NodeHandle nh_;
	Server upwardServer_;
	std::string action_name_;
	motion_actions::UpwardFeedback feedback_;
	motion_actions::UpwardResult result_;
	ros::Subscriber sub_;
	float timeSpent, motionTime;
	bool success;
	ros::Publisher PWM, direction;
	float p,i,d;

public:
		//Constructor, called when new instance of class declared
	innerActionClass(std::string name):
			//Defining the server, third argument is optional
	upwardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false),
	action_name_(name)
	{
			// Add preempt callback
		upwardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
			// Declaring publisher for PWM and direction
		PWM = nh_.advertise<std_msgs::Int32>("PWM",1000);
		direction = nh_.advertise<std_msgs::Int32>("direction",1000);
			// Starting new Action Server
		upwardServer_.start();
	}

		// default contructor
	~innerActionClass(void){
	}

		// callback for goal cancelled
		// Stop the bot
	void preemptCB(void){
		pwm.data = 0;
		dir.data = 5;
		PWM.publish(pwm);
		direction.publish(dir);							
		ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
			//this command cancels the previous goal
			// upwardServer_.setPreempted();
	}

		// called when new goal recieved
		// Start motion and finish it, if not interupted
	void analysisCB(const motion_actions::UpwardGoalConstPtr goal){
		ROS_INFO("Inside analysisCB");

		int loopRate =10 ;
		ros::Rate loop_rate(loopRate);

		//waiting till we recieve the first value from Camera else it's useless to to any calculations
		while(!initData){
			ROS_INFO("Waiting to get first input from Camera");
			loop_rate.sleep();
		}

		if(goal->Goal == 1)
			finalHeight = 0;

		float derivative=0,integral=0,dt=1.0/loopRate;
		bool reached=false;

		pwm.data=0;
		dir.data=5;

		if (!upwardServer_.isActive())
			return;

		while(!upwardServer_.isPreemptRequested()&&ros::ok()){
			error = finalHeight - presentHeight;
			integral+= (error*dt);
			derivative = (presentHeight- previousHeight)/dt;

			output= (p*error)+(i*integral)+(d*derivative);

			upwardOutputPWMMapping(output);

			//this lower limit depends upon the bot itself, below these values of PWM thrusters will not start
			if(pwm.data < minPWM)
				pwm.data= minPWM;	

			feedback_.HeightRemaining = error;

			upwardServer_.publishFeedback(feedback_);
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

			if (upwardServer_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				upwardServer_.setPreempted();
				reached = false;
				break;
			}
		}
	}
	void upwardOutputPWMMapping(float output){
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
	void setPID(float new_p, float new_i, float new_d) {
		p=new_p;
		i=new_i;
		d=new_d;
	}
};
innerActionClass* object; 

///dynamic reconfig 
void callback(motion_upward::pidConfig &config, double level) {
	ROS_INFO("Reconfigure Request: p= %f i= %f d=%f", config.p, config.i, config.d);
	object->setPID(config.p, config.i, config.d);
}

void distanceCb(std_msgs::Float64 msg){
	//this is used to set the final angle after getting the value of first intial position
	if(initData == false){
		presentHeight= msg.data;
		previousHeight = presentHeight;
		initData = true;
	}
	else{
		previousHeight = presentHeight;
		presentHeight= msg.data;
		// ROS_INFO("New angular postion %f", msg.data);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "upward");

	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("Height",1000,&distanceCb);

	ROS_INFO("Waiting for Goal");
	object = new innerActionClass(ros::this_node::getName());

	//register dynamic reconfig server.
	dynamic_reconfigure::Server<motion_upward::pidConfig> server;
	dynamic_reconfigure::Server<motion_upward::pidConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	return 0;
}
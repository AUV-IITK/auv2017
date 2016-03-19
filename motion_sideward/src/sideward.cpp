#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_actions/SidewardAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_sideward/pidConfig.h>
#define minPWM 120
typedef actionlib::SimpleActionServer<motion_actions::SidewardAction> Server; // defining the Client type

float presentSidePosition=0;
float previousSidePosition=0;
float finalSidePosition, error, output;
bool initData =false;

std_msgs::Int32 pwm; // pwm to be send to arduino
std_msgs::Int32 dir; // dir to be send to arudino

// new inner class, to encapsulate the interaction with actionclient
class innerActionClass{
	private:
		ros::NodeHandle nh_;
		Server sidewardServer_;
		std::string action_name_;
		motion_actions::SidewardFeedback feedback_;
		motion_actions::SidewardResult result_;
		ros::Subscriber sub_;
		float timeSpent, motionTime;
		bool success;
		ros::Publisher PWM, direction;
		float p,i,d;

	public:
		//Constructor, called when new instance of class declared
		innerActionClass(std::string name):
			//Defining the server, third argument is optional
			sidewardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false),
    		action_name_(name)
		{
			// Add preempt callback
			sidewardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
			// Declaring publisher for PWM and direction
			PWM = nh_.advertise<std_msgs::Int32>("PWM",1000);
			direction = nh_.advertise<std_msgs::Int32>("direction",1000);
			// Starting new Action Server
			sidewardServer_.start();
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
			// sidewardServer_.setPreempted();
		}

		// called when new goal recieved
		// Start motion and finish it, if not interupted
	void analysisCB(const motion_actions::SidewardGoalConstPtr goal){
		ROS_INFO("Inside analysisCB");

		int loopRate =10 ;
		ros::Rate loop_rate(loopRate);

		//waiting till we recieve the first value from Camera else it's useless to to any calculations
		while(!initData){
			ROS_INFO("Waiting to get first input from Camera");
			loop_rate.sleep();
		}

		if(goal->Goal == 1)
			finalSidePosition = 0;

		float derivative=0,integral=0,dt=1.0/loopRate;
		bool reached=false;

		pwm.data=0;
		dir.data=5;

		if (!sidewardServer_.isActive())
			return;

		while(!sidewardServer_.isPreemptRequested()&&ros::ok()){
			error = finalSidePosition - presentSidePosition;
			integral+= (error*dt);
			derivative = (presentSidePosition- previousSidePosition)/dt;

			output= (p*error)+(i*integral)+(d*derivative);

			sidewardOutputPWMMapping(output);

			//this lower limit depends upon the bot itself, below these values of PWM thrusters will not start
			if(pwm.data < minPWM)
				pwm.data= minPWM;	

			feedback_.DistanceRemaining = error;

			sidewardServer_.publishFeedback(feedback_);
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

			if (sidewardServer_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				sidewardServer_.setPreempted();
				reached = false;
				break;
			}
		}
	}
	void sidewardOutputPWMMapping(float output){
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
void callback(motion_sideward::pidConfig &config, double level) {
	ROS_INFO("Reconfigure Request: p= %f i= %f d=%f", config.p, config.i, config.d);
	object->setPID(config.p, config.i, config.d);
}

void distanceCb(std_msgs::Float64 msg){
	//this is used to set the final angle after getting the value of first intial position
	if(initData == false){
		presentSidePosition= msg.data;
		previousSidePosition = presentSidePosition;
		initData = true;
	}
	else{
		previousSidePosition = presentSidePosition;
		presentSidePosition= msg.data;
		// ROS_INFO("New angular postion %f", msg.data);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sideward");

	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("yDistance",1000,&distanceCb);

	ROS_INFO("Waiting for Goal");
	object = new innerActionClass(ros::this_node::getName());

	//register dynamic reconfig server.
	dynamic_reconfigure::Server<motion_sideward::pidConfig> server;
	dynamic_reconfigure::Server<motion_sideward::pidConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	return 0;
}
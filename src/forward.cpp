#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_forward/ForwardAction.h>

typedef actionlib::SimpleActionServer<motion_forward::ForwardAction> Server; // defining the Client type

std_msgs::Int32 pwm; // pwm to be send to arduino
std_msgs::Int32 dir; // dir to be send to arudino

// new inner class, to encapsulate the interaction with actionclient
class forwardAction{
	private:
		ros::NodeHandle nh_;
		Server forwardServer_;
		std::string action_name_;
		motion_forward::ForwardFeedback feedback_;
		motion_forward::ForwardResult result_;
		ros::Subscriber sub_;
		float timeSpent, motionTime;
		bool success;
		ros::Publisher PWM, direction;

	public:
		//Constructor, called when new instance of class declared
		forwardAction(std::string name):
			//here we are defining the server, third argument is optional
	    	forwardServer_(nh_, name, boost::bind(&forwardAction::analysisCB, this, _1), false),
    		action_name_(name)
		{
			// Add preempt callback
			forwardServer_.registerPreemptCallback(boost::bind(&forwardAction::preemptCB, this));
			// Declaring publisher for PWM and direction
			PWM = nh_.advertise<std_msgs::Int32>("PWM",1000);
			direction = nh_.advertise<std_msgs::Int32>("direction",1000);
			// Starting new Action Server
			forwardServer_.start();
		}

		// default contructor
		~forwardAction(void){
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
			// forwardServer_.setPreempted();
		}

		// called when new goal recieved
		// Start motion and finish it, if not interupted
		void analysisCB(const motion_forward::ForwardGoalConstPtr goal){
			ROS_INFO("Inside analysisCB");

			pwm.data = 90;
			dir.data = 1;
			ros::Rate looprate(1);
			success = true;

			if (!forwardServer_.isActive())
				return;

			for(timeSpent=0; timeSpent <= goal->Goal; timeSpent++){
				if (forwardServer_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
			        // set the action state to preempted
        			forwardServer_.setPreempted();
        			success = false;
        			break;
				}
				// publish the feedback
				feedback_.Feedback = goal->Goal - timeSpent;
				forwardServer_.publishFeedback(feedback_);
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);

				ROS_INFO("timeSpent %f", timeSpent);
				ros::spinOnce();
				looprate.sleep();				
			}
			if(success){
				result_.Result = success;
				ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				pwm.data = 120;
				dir.data = 5;
				PWM.publish(pwm);
				direction.publish(dir);				
				// set the action state to succeeded
				forwardServer_.setSucceeded(result_);
			}

		}
};

int main(int argc, char** argv){

	// Initializing the node
	ros::init(argc, argv, "forward");

	// declaring a new instance of inner class, constructor gets called
	forwardAction forward(ros::this_node::getName());
	ROS_INFO("Waiting for Goal");

	ros::spin();
	return 0;
}
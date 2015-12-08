#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <linedetection/orangeAction.h>

typedef actionlib::SimpleActionServer<linedetection::orangeAction> Server;

class LineDetectionInnerClass{
	private:
		ros::NodeHandle nh_;
		Server lineDetectionServer_;
		std::string action_name_;
		linedetection::orangeFeedback feedback_;
		linedetection::orangeResult result_;
		ros::Subscriber sub_;
		bool success;
		ros::Publisher onoff_pub_;
		bool isOrange;
	public:
		LineDetectionInnerClass(std::string name):
			//here we are defining the server, third argument is optional
	    	lineDetectionServer_(nh_, name, boost::bind(&LineDetectionInnerClass::analysisCB, this, _1), false),
    		action_name_(name)
		{
			isOrange=false;
			lineDetectionServer_.registerPreemptCallback(boost::bind(&LineDetectionInnerClass::preemptCB, this));
			onoff_pub_ = nh_.advertise<std_msgs::Bool>("orangeonoff", 1000);
			sub_ = nh_.subscribe<std_msgs::Bool>("linedetected", 1000,&LineDetectionInnerClass::lineDetectedListener,this);
			lineDetectionServer_.start();
		}

		~LineDetectionInnerClass(void){
		}

		void lineDetectedListener(std_msgs::Bool msg)
		{
			if(msg.data)
				isOrange=true;
			else
				isOrange=false;
		}

		void preemptCB(void){
			//Not actually preempting the goal because Shibhansh did it in analysisCB
			ROS_INFO("Called when preempted from the client");
		}

		void analysisCB(const linedetection::orangeGoalConstPtr goal){
			ROS_INFO("Inside analysisCB");
			success = true;
			ros::Rate looprate(12);

			if (!lineDetectionServer_.isActive())
				return;

			while(goal->order){
				if (lineDetectionServer_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
			        // set the action state to preempted
        			lineDetectionServer_.setPreempted();
        			success = false;
        			break;
				}
				startIP();
				looprate.sleep();
				// start moving forward.
				if(isOrange)
				{
					//stop now
					break;
				}
				else
				{
					//line not present below
				}
				// if we are past the initial line then feedback=true. else feedback=false;
				// publish the feedback
				feedback_.nosignificance = false;
				lineDetectionServer_.publishFeedback(feedback_);
				ROS_INFO("timeSpent");
				ros::spinOnce();
			}
			stopIP();
			if(success){
				isOrange=false;
				result_.MotionCompleted = success;
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				// set the action state to succeeded
				lineDetectionServer_.setSucceeded(result_);
			}

		}

		void startIP()
		{
			std_msgs::Bool msg;
			msg.data = true;
			onoff_pub_.publish(msg);
		}

		void stopIP()
		{
			std_msgs::Bool msg;
			msg.data = false;
			onoff_pub_.publish(msg);
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "linedetectionserver");
	ROS_INFO("Waiting for Goal");
	LineDetectionInnerClass lineDetectionObject(ros::this_node::getName());
	ros::spin();
	return 0;
}
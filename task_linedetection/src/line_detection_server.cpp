#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <linedetection/orangeAction.h>

#include <motionlibrary/ForwardAction.h>
#include <motionlibrary/ForwardActionFeedback.h>

typedef actionlib::SimpleActionServer<linedetection::orangeAction> Server;
typedef actionlib::SimpleActionClient<motionlibrary::ForwardAction> Client;

class LineDetectionInnerClass{
	private:
		ros::NodeHandle nh_;
		Server lineDetectionServer_;
		std::string action_name_;
		linedetection::orangeFeedback feedback_;
		linedetection::orangeResult result_;
		ros::Subscriber sub_;
		bool success;
		ros::Publisher off_pub_;
		bool isOrange;
		Client ForwardClient_;
		motionlibrary::ForwardGoal forwardgoal;
	public:
		LineDetectionInnerClass(std::string name, std::string node):
			//here we are defining the server, third argument is optional
	    	lineDetectionServer_(nh_, name, boost::bind(&LineDetectionInnerClass::analysisCB, this, _1), false),
    		action_name_(name), ForwardClient_(node)
		{
			ROS_INFO("inside constructor");
			lineDetectionServer_.registerPreemptCallback(boost::bind(&LineDetectionInnerClass::preemptCB, this));
			off_pub_ = nh_.advertise<std_msgs::Bool>("orangeoff", 1000);
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
			isOrange=false;
			ros::Rate looprate(12);

			if (!lineDetectionServer_.isActive())
				return;

			ROS_INFO("Waiting for Forward server to start.");
			ForwardClient_.waitForServer();

			boost::thread vision_thread(&LineDetectionInnerClass::startIP, this);	
			// start moving forward.
			forwardgoal.MotionTime = 100;
			ForwardClient_.sendGoal(forwardgoal);
			while(goal->order){
				if (lineDetectionServer_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
			        // set the action state to preempted
        			lineDetectionServer_.setPreempted();
        			success = false;
        			break;
				}
				looprate.sleep();
				if(isOrange)
				{
					//stop will happen outside while loop so if preempted then too it will stop
					break;
				}
				// publish the feedback
				feedback_.nosignificance = false;
				lineDetectionServer_.publishFeedback(feedback_);
				ROS_INFO("timeSpent");
				ros::spinOnce();
			}
			ForwardClient_.cancelGoal();			//stop motion here
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
			std::system("rosrun linedetection orangedetection");
		}

		void stopIP()
		{
			std_msgs::Bool msg;
			msg.data = true;
			off_pub_.publish(msg);
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "linedetectionserver");
	ROS_INFO("Waiting for Goal");
	LineDetectionInnerClass lineDetectionObject(ros::this_node::getName(),"forward");
	ros::spin();
	return 0;
}
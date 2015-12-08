#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
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

	public:
		LineDetectionInnerClass(std::string name):
			//here we are defining the server, third argument is optional
	    	lineDetectionServer_(nh_, name, boost::bind(&LineDetectionInnerClass::analysisCB, this, _1), false),
    		action_name_(name)
		{
			lineDetectionServer_.registerPreemptCallback(boost::bind(&LineDetectionInnerClass::preemptCB, this));
			lineDetectionServer_.start();
		}

		~LineDetectionInnerClass(void){
		}

		void preemptCB(void){
			//Not actually preempting the goal because Shibhansh did it in analysisCB
			ROS_INFO("Called when preempted from the client");
		}

		void analysisCB(const linedetection::orangeGoalConstPtr goal){
			ROS_INFO("Inside analysisCB");
			success = true;
			ros::Rate looprate(1);

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
				// publish the feedback
				feedback_.nosignificance = true;
				lineDetectionServer_.publishFeedback(feedback_);
				ROS_INFO("timeSpent");
				ros::spinOnce();
				looprate.sleep();				
			}

			if(success){
				result_.MotionCompleted = success;
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				// set the action state to succeeded
				lineDetectionServer_.setSucceeded(result_);
			}

		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "forward");
	ROS_INFO("Waiting for Goal");
	LineDetectionInnerClass lineDetectionObject(ros::this_node::getName());
	ros::spin();
	return 0;
}
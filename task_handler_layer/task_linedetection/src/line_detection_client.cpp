#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <linedetection/orangeAction.h>
#include <linedetection/orangeActionFeedback.h>
#include <linedetection/orangeActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<linedetection::orangeAction> Client;

Client *chutiya;
linedetection::orangeGoal goal;

bool success=false;

void spinThread(){
	Client &temp = *chutiya;
	temp.waitForResult();
	success = (*(temp.getResult())).MotionCompleted;
	if(success){
		ROS_INFO("motion successful");
	}
	else
		ROS_INFO("motion unsuccessful");
	ros::shutdown();
}

//never ever put the argument of the callback function anything other then the specified
void forwardCb(linedetection::orangeActionFeedback msg){
	ROS_INFO("feedback recieved %d",msg.feedback.nosignificance);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "linedetectionclient");
	
	ros::NodeHandle nh;
	//here linedetectionserver is the name of the node of the actionserver.
	ros::Subscriber sub_ = nh.subscribe<linedetection::orangeActionFeedback>("/linedetectionserver/feedback",1000,&forwardCb);

	Client testClient("linedetectionserver");
	chutiya = &testClient;

	ROS_INFO("Waiting for action server to start.");
	testClient.waitForServer();
	goal.order = true;
	ROS_INFO("Action server started, sending goal.");

	// Client &can = *chutiya;
	// can.cancelGoal();
	// ROS_INFO("Goal Cancelled");
	Client &can = *chutiya;
	//send goal
	can.sendGoal(goal);
	boost::thread spin_thread(&spinThread);	
	ros::spin();
	return 0;
}
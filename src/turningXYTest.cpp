#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <motionlibrary/TurnAction.h>
#include <motionlibrary/TurnActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<motionlibrary::TurnAction> Client;

bool cancel=false;
//never ever put the argument of the callback function anything other then the specified
//void forwardCb(const motionlibrary::ForwardActionFeedbackConstPtr msg){
void turnCb(motionlibrary::TurnActionFeedback msg){
	ROS_INFO("feedback recieved, %f deg remaining ",msg.feedback.AngleRemaining);
}

void spinThread()
{
	ros::spin();
}


int main(int argc, char** argv){
	ros::init(argc,argv,"testTurningXY");

	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<motionlibrary::TurnActionFeedback>("/TurnXY/feedback",1000,&turnCb);

	// Create the action client
	Client TurnTestClient("TurnXY");


	// Here the thread is created and the ros node is started spinning in the background.
	// By using this method you can create multiple threads for your action client if needed. 
	boost::thread spin_thread(&spinThread);

	ROS_INFO("Waiting for action server to start.");
	TurnTestClient.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	// Send Goal
	motionlibrary::TurnGoal goal;
	goal.AngleToTurn = 90;
	TurnTestClient.sendGoal(goal);
	ROS_INFO("Goal Send");

	//here this part of the code simply waits for the result and rest of the part can be done in the thread
	TurnTestClient.waitForResult(); //gives true after the result is accomplished
	ROS_INFO("Mission accomplished");

	ros::shutdown();

	// Now that the goal is completed and we have reported the goal status, we need to shutdown the ros node and join our thread back before exiting. 
	spin_thread.join();

	return 0;

}
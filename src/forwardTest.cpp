#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <motionlibrary/ForwardAction.h>
#include <motionlibrary/ForwardActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


typedef actionlib::SimpleActionClient<motionlibrary::ForwardAction> Client;
<<<<<<< HEAD
bool cancel=false;
=======

>>>>>>> initial commit
//never ever put the argument of the callback function anything other then the specified
//void forwardCb(const motionlibrary::ForwardActionFeedbackConstPtr msg){
void forwardCb(motionlibrary::ForwardActionFeedback msg){
	ROS_INFO("feedback recieved %fsec remaining ",msg.feedback.TimeRemaining);
<<<<<<< HEAD
	if(msg.feedback.TimeRemaining == 15){
//		forwardTestClient.cancelGoal();
		ROS_INFO("Goal cancelled");
	}
=======
>>>>>>> initial commit
}

void spinThread()
{
<<<<<<< HEAD
=======
	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<motionlibrary::ForwardActionFeedback>("/forward/feedback",1000,&forwardCb);

>>>>>>> initial commit
	ros::spin();
}
// void doneCb(){
// 	ROS_INFO("Finished successfully");
// 	ros::shutdown();
// }
// void activeCb()
// {
//    ROS_INFO("Goal just went active");
// }
// void feedbackCb(const motionlibrary::ForwardFeedbackConstPtr& feedback)
// {
//    ROS_INFO("Got Feedback,time remaining %f", feedback->TimeRemaining);
// }

int main(int argc, char** argv){
<<<<<<< HEAD

	ros::init(argc, argv, "testForwardMotion");
	
	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<motionlibrary::ForwardActionFeedback>("/forward/feedback",1000,&forwardCb);

=======
	ros::init(argc, argv, "testForwardMotion");
>>>>>>> initial commit

	// Create the action client
	Client forwardTestClient("forward");

<<<<<<< HEAD

=======
>>>>>>> initial commit
	// Here the thread is created and the ros node is started spinning in the background.
	// By using this method you can create multiple threads for your action client if needed. 
	boost::thread spin_thread(&spinThread);

	ROS_INFO("Waiting for action server to start.");
	forwardTestClient.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	// Send Goal
	motionlibrary::ForwardGoal goal;
	goal.MotionTime = 20;
	forwardTestClient.sendGoal(goal);
	ROS_INFO("Goal Send");

	//here this part of the code simply waits for the result and rest of the part can be done in the thread
<<<<<<< HEAD
	forwardTestClient.waitForResult(); //gives true after the result is accomplished
	
=======
	forwardTestClient.waitForResult();
>>>>>>> initial commit

	ros::shutdown();

	// Now that the goal is completed and we have reported the goal status, we need to shutdown the ros node and join our thread back before exiting. 
	spin_thread.join();

	return 0;
}
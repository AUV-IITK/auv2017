#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <linefollowing/AlignAction.h>
#include <linefollowing/AlignActionFeedback.h>
#include <linefollowing/AlignActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<linefollowing::AlignAction> Client;

Client *ptrClient;
linefollowing::AlignGoal goal;

void spinThread(){
	Client &temp = *ptrClient;
	temp.waitForResult();
	bool success;
	success = (*(temp.getResult())).Aligned;
	if(success){
		ROS_INFO("motion successful");
	}
	else
		ROS_INFO("motion unsuccessful");
}


// //dynamic reconfig 
// void callback(motionlibrary::forwardConfig &config, double level) {
// 	ROS_INFO("Reconfigure Request: %f %s", config.double_param, config.bool_param?"True":"False");
// 	Client &can = *ptrClient;
// 	if(!config.bool_param){
// 		if(moving){
// 			moving = false;
// 			can.cancelGoal();
// 			ROS_INFO("Goal Cancelled");
// 		}
// 	}
// 	else{
// 		if(moving){
// 			Client &can = *ptrClient;
// 			can.cancelGoal();
// 			ROS_INFO("Goal Cancelled");	
// 		}
// 		goal.MotionTime = config.double_param;
// 		can.sendGoal(goal);
// 		boost::thread spin_thread(&spinThread);
// 	 	ROS_INFO("Goal Send %f", goal.MotionTime);
// 		moving = true;
// 	}
	
// }

//never ever put the argument of the callback function anything other then the specified
//void forwardCb(const motionlibrary::ForwardActionFeedbackConstPtr msg){
void AlignCb(linefollowing::AlignActionFeedback msg){
	ROS_INFO("feedback recieved %fsec remaining ",msg.feedback.AngleRemaining);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "testAlign");

	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<linefollowing::AlignActionFeedback>("/Align/feedback",1000,&AlignCb);

	Client AlignTestClient("Align");
	ptrClient = &AlignTestClient;

	ROS_INFO("Waiting for action server to start.");
	AlignTestClient.waitForServer();
	goal.StartDetection = true;
	ROS_INFO("Action server started, sending goal.");
	// Send Goal
	AlignTestClient.sendGoal(goal);
	ROS_INFO("Goal Send");


	// Here the thread is created and the ros node is started spinning in the background.
	// By using this method you can create multiple threads for your action client if needed. 
	boost::thread spin_thread(&spinThread);

	ros::spin();
	return 0;
}
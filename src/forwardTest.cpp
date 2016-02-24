#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <motionlibrary/ForwardAction.h>
#include <motionlibrary/ForwardActionFeedback.h>
#include <motionlibrary/ForwardActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motionlibrary/forwardConfig.h>

typedef actionlib::SimpleActionClient<motionlibrary::ForwardAction> Client;

Client *chutiya;
motionlibrary::ForwardGoal goal;

bool moving=false;
bool success=false;

void spinThread(){
	Client &temp = *chutiya;
	temp.waitForResult();
	success = (*(temp.getResult())).Result;
	if(success){
		ROS_INFO("motion successful");
	}
	else
		ROS_INFO("motion unsuccessful");
}


//dynamic reconfig 
void callback(motionlibrary::forwardConfig &config, double level) {
	ROS_INFO("Reconfigure Request: %f %s", config.double_param, config.bool_param?"True":"False");
	Client &can = *chutiya;
	if(!config.bool_param){
		if(moving){
			moving = false;
			can.cancelGoal();
			ROS_INFO("Goal Cancelled");
		}
	}
	else{
		if(moving){
			Client &can = *chutiya;
			can.cancelGoal();
			ROS_INFO("Goal Cancelled");	
		}
		goal.Goal = config.double_param;
		can.sendGoal(goal);
		boost::thread spin_thread(&spinThread);
	 	ROS_INFO("Goal Send %f", goal.Goal);
		moving = true;
	}
	
}

//never ever put the argument of the callback function anything other then the specified
//void forwardCb(const motionlibrary::ForwardActionFeedbackConstPtr msg){
void forwardCb(motionlibrary::ForwardActionFeedback msg){
	ROS_INFO("feedback recieved %fsec remaining ",msg.feedback.Feedback);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "testForwardMotion");
	
	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<motionlibrary::ForwardActionFeedback>("/forward/feedback",1000,&forwardCb);

	Client forwardTestClient("forward");
	chutiya = &forwardTestClient;

	ROS_INFO("Waiting for action server to start.");
	forwardTestClient.waitForServer();
	goal.Goal =0;
	ROS_INFO("Action server started, sending goal.");



	//register dynamic reconfig server.
	dynamic_reconfigure::Server<motionlibrary::forwardConfig> server;
	dynamic_reconfigure::Server<motionlibrary::forwardConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	// // Create the action client
	// Client forwardTestClient("forward");

	// // Here the thread is created and the ros node is started spinning in the background.
	// // By using this method you can create multiple threads for your action client if needed. 
	// boost::thread spin_thread(&spinThread);

	// ROS_INFO("Waiting for action server to start.");
	// forwardTestClient.waitForServer();
	// ROS_INFO("Action server started, sending goal.");

	// // Send Goal
	// motionlibrary::ForwardGoal goal;
	// goal.MotionTime = 20;
	// forwardTestClient.sendGoal(goal);
	// chutiya = &forwardTestClient;
	// ROS_INFO("Goal Send");

	// //here this part of the code simply waits for the result and rest of the part can be done in the thread
	// forwardTestClient.waitForResult();

	// ros::shutdown();

	// // Now that the goal is completed and we have reported the goal status, we need to shutdown the ros node and join our thread back before exiting. 
	// spin_thread.join();

	ros::spin();
	return 0;
}
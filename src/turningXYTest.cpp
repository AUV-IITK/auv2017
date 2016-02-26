#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <turn_xy/TurnAction.h>
#include <turn_xy/TurnActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <turn_xy/turningConfig.h>

typedef actionlib::SimpleActionClient<turn_xy::TurnAction> Client;
Client *chutiya;
turn_xy::TurnGoal goal;

bool goalSet = false;

//dynamic reconfig 
void callback(turn_xy::turningConfig &config, double level) {
	ROS_INFO("Reconfigure Request: %f %s",  config.double_param, config.bool_param?"True":"False");
	Client &can = *chutiya;
	if(!config.bool_param){
		if(goalSet){
			goalSet = false;
			can.cancelGoal();
			ROS_INFO("Goal Cancelled");			
		}
	}
	else{
		if(goalSet){
			Client &can = *chutiya;
			can.cancelGoal();
			ROS_INFO("Goal Cancelled");	
		}
		goal.AngleToTurn = config.double_param;
		can.sendGoal(goal);
	 	ROS_INFO("Goal Send %f", goal.AngleToTurn);		
		goalSet = true;
	}



	// if(config.double_param != goal.AngleToTurn){
	// 	Client can("TurnXY");
	// 	goal.AngleToTurn = config.double_param;
	// 	can.sendGoal(goal);
	// 	ROS_INFO("Goal Send %f", goal.AngleToTurn);
	// }
	// if(!config.bool_param){
	// 	Client &can = *chutiya;
	// 	can.cancelGoal();
	// 	ROS_INFO("Goal Cancelled");
	// }
	// else{
	// 	if(config.double_param == goal.AngleToTurn);
	// 	else{
	// 		Client can("TurnXY");
	// 		goal.AngleToTurn = config.double_param;
	// 		can.sendGoal(goal);
	// 		ROS_INFO("Goal Send %f", goal.AngleToTurn);			
	// 	}
	// }
}

//never ever put the argument of the callback function anything other then the specified
//void forwardCb(const turn_xy::ForwardActionFeedbackConstPtr msg){
void turnCb(turn_xy::TurnActionFeedback msg){
	ROS_INFO("feedback recieved, %f deg remaining ",msg.feedback.AngleRemaining);
}

// void spinThread()
// {
// 	ros::spin();
// }


int main(int argc, char** argv){
	ros::init(argc,argv,"testTurningXY");

	ros::NodeHandle nh;
	ros::Subscriber sub_ = nh.subscribe<turn_xy::TurnActionFeedback>("/TurnXY/feedback",1000,&turnCb);

	Client TurnTestClient("TurnXY");
	chutiya = &TurnTestClient;
	//this wait has to be implemented here so that we can wait for the server to start
	ROS_INFO("Waiting for action server to start.");
	TurnTestClient.waitForServer();
	goal.AngleToTurn =0;
	ROS_INFO("Action server started, sending goal.");

	//register dynamic reconfig server.
	dynamic_reconfigure::Server<turn_xy::turningConfig> server;
	dynamic_reconfigure::Server<turn_xy::turningConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	// Create the action client
// 	


// 	// Here the thread is created and the ros node is started spinning in the background.
// 	// By using this method you can create multiple threads for your action client if needed. 
// 	// boost::thread spin_thread(&spinThread);

// 	ROS_INFO("Waiting for action server to start.");
// 	
// 	ROS_INFO("Action server started, sending goal.");

// 	// Send Goal
// //	goal.AngleToTurn = 90;
// 	TurnTestClient.sendGoal(goal);
// 	ROS_INFO("Goal Send");

	//here this part of the code simply waits for the result and rest of the part can be done in the thread

	ros::spin();

	// Now that the goal is completed and we have reported the goal status, we need to shutdown the ros node and join our thread back before exiting. 
	// spin_thread.join();

	return 0;

}
// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_commons/octagonAction.h>
#include <task_commons/octagonActionFeedback.h>
#include <task_commons/octagonActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::octagonAction> Client;

Client *ptrClient;
task_commons::octagonGoal goal;

bool success = false;

void spinThread()
{
  Client &temp = *ptrClient;
  temp.waitForResult();
  success = (*(temp.getResult())).MotionCompleted;
  if (success)
  {
    ROS_INFO("%s motion successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s motion unsuccessful", ros::this_node::getName().c_str());
  ros::shutdown();
}

// never ever put the argument of the callback function anything other then the specified
void forwardCb(task_commons::octagonActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f", ros::this_node::getName().c_str(), msg.feedback.x_coord,
           msg.feedback.y_coord);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octagon_client");

  ros::NodeHandle nh;
  // here octagon_server is the name of the node of the actionserver.
  ros::Subscriber sub_ =
      nh.subscribe<task_commons::octagonActionFeedback>("/octagon_server/feedback", 1000, &forwardCb);

  Client testClient("octagon_server");
  ptrClient = &testClient;

  ROS_INFO("%s: Waiting for action server to start.", ros::this_node::getName().c_str());
  testClient.waitForServer();
  goal.order = true;
  ROS_INFO("%s Action server started, sending goal.", ros::this_node::getName().c_str());

  // Client &ca = *ptrClient;
  // can.cancelGoal();
  // ROS_INFO("Goal Cancelled");
  Client &can = *ptrClient;
  // send goal
  can.sendGoal(goal);
  boost::thread spin_thread(&spinThread);
  ros::spin();
  return 0;
}

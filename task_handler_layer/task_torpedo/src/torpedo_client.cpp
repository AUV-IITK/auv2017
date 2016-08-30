// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <task_commons/torpedoAction.h>
#include <task_commons/torpedoActionFeedback.h>
#include <task_commons/torpedoActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::torpedoAction> Client;

Client *ptrClient;
task_commons::torpedoGoal goal;

bool success = false;

void spinThread()
{
  Client &temp = *ptrClient;
  temp.waitForResult();
  success = (*(temp.getResult())).MotionCompleted;
  if (success)
  {
    ROS_INFO("motion successful");
  }
  else
    ROS_INFO("motion unsuccessful");
  ros::shutdown();
}

// never ever put the argument of the callback function anything other then the specified
void forwardCb(task_commons::torpedoActionFeedback msg)
{
  ROS_INFO("feedback recieved %d", msg.feedback.nosignificance);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torpedo_client");

  ros::NodeHandle nh;
  // here torpedo_server is the name of the node of the actionserver.
  ros::Subscriber sub_ =
      nh.subscribe<task_commons::torpedoActionFeedback>("/torpedo_server/feedback", 1000, &forwardCb);

  Client testClient("torpedo_server");
  ptrClient = &testClient;

  ROS_INFO("Waiting for action server to start.");
  testClient.waitForServer();
  goal.order = true;
  ROS_INFO("Action server started, sending goal.");

  Client &can = *ptrClient;
  // send goal
  can.sendGoal(goal);
  boost::thread spin_thread(&spinThread);
  ros::spin();
  return 0;
}

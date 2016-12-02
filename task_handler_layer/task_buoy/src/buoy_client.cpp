// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_commons/buoyAction.h>
#include <task_commons/buoyActionFeedback.h>
#include <task_commons/buoyActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::buoyAction> Client;

Client *ptrClient;
task_commons::buoyGoal goal;

bool success = false;

void spinThread()
{
  Client &temp = *ptrClient;
  temp.waitForResult();
  success = (*(temp.getResult())).MotionCompleted;
  if (success)
  {
    ROS_INFO("%s: motion successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion unsuccessful", ros::this_node::getName().c_str());
  ros::shutdown();
}

// never ever put the argument of the callback function anything other then the specified
void forwardCb(task_commons::buoyActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f, distance = %f", ros::this_node::getName().c_str(), msg.feedback.x_coord,
           msg.feedback.y_coord, msg.feedback.distance);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "buoy_client");

  ros::NodeHandle nh;
  // here buoy_server is the name of the node of the actionserver.
  ros::Subscriber sub_ = nh.subscribe<task_commons::buoyActionFeedback>("/buoy_server/feedback", 1000, &forwardCb);

  Client testClient("buoy_server");
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

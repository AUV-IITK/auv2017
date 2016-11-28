// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <task_commons/buoyAction.h>
#include <task_commons/lineAction.h>
#include <task_commons/buoyActionFeedback.h>
#include <task_commons/lineActionFeedback.h>
#include <task_commons/buoyActionResult.h>
#include <task_commons/lineActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::buoyAction> ClientBuoy;
typedef actionlib::SimpleActionClient<task_commons::lineAction> ClientLine;

ClientBuoy *ptrClientBuoy;
ClientLine *ptrClientLine;
task_commons::buoyGoal goalBuoy;
task_commons::lineGoal goalLine;

bool successBuoy = false;
bool successLine = false;

void spinThreadBuoy()
{
  ClientBuoy &temp = *ptrClientBuoy;
  temp.waitForResult();
  successBuoy = (*(temp.getResult())).MotionCompleted;
  if (successBuoy)
  {
    ROS_INFO("%s: motion buoy successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion buoy unsuccessful", ros::this_node::getName().c_str());
}

void spinThreadLine()
{
  ClientLine &temp = *ptrClientLine;
  temp.waitForResult();
  successLine = (*(temp.getResult())).MotionCompleted;
  if (successLine)
  {
    ROS_INFO("%s: motion line successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion line unsuccessful", ros::this_node::getName().c_str());
}
// never ever put the argument of the callback function anything other then the specified
void forwardCbBuoy(task_commons::buoyActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f, distance = %f",
    ros::this_node::getName().c_str(), msg.feedback.x_coord, msg.feedback.y_coord, msg.feedback.distance);
}

void forwardCbLine(task_commons::lineActionFeedback msg)
{
  ROS_INFO("%s: feedback recieved Angle Remaining = %f, x_coord = %f, y_coord = %f",
    ros::this_node::getName().c_str(), msg.feedback.AngleRemaining, msg.feedback.x_coord, msg.feedback.y_coord);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");

  ros::NodeHandle nh;
  // here buoy_server is the name of the node of the actionserver.
  ros::Subscriber sub_buoy = nh.subscribe<task_commons::buoyActionFeedback>("/buoy_server/feedback",
    1000, &forwardCbBuoy);
  ros::Subscriber sub_line = nh.subscribe<task_commons::lineActionFeedback>("/line_server/feedback",
    1000, &forwardCbLine);

  ClientBuoy buoyClient("buoy_server");
  ptrClientBuoy = &buoyClient;
  ClientLine lineClient("line_server");
  ptrClientLine = &lineClient;

  ROS_INFO("Waiting for action server to start.");
  lineClient.waitForServer();
  buoyClient.waitForServer();

  goalLine.order = true;
  ROS_INFO("Action server started, sending goal to line.");

  ClientLine &canLine = *ptrClientLine;
  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line(&spinThreadLine);
  while (!successLine)
  {
    ROS_INFO("wait for the line dude :p");
    ros::spinOnce();
  }

  goalBuoy.order = true;
  ROS_INFO("Action server started, sending goal to buoy.");

  ClientBuoy &canBuoy = *ptrClientBuoy;
  // send goal
  canBuoy.sendGoal(goalBuoy);
  boost::thread spin_thread_buoy(&spinThreadBuoy);
  ros::spin();
  return 0;
}

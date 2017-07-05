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
#include <motion_commons/ForwardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::buoyAction> ClientBuoy;
typedef actionlib::SimpleActionClient<task_commons::lineAction> ClientLine;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;

ClientBuoy *ptrClientBuoy;
ClientLine *ptrClientLine;
ClientUpward *ptrClientUpward;
ClientForward *ptrClientForward;
task_commons::buoyGoal goalBuoy;
task_commons::lineGoal goalLine;
motion_commons::UpwardGoal goalUpward;
motion_commons::ForwardGoal goalForward;

ros::Publisher pub_upward;
ros::Publisher pub_forward;

bool successBuoy = false;
bool successLine = false;
bool successUpward = false;
bool successDownward = false;

float present_depth;

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

void spinThreadUpwardPressure()
{
  ClientUpward &temp = *ptrClientUpward;
  temp.waitForResult();
  successUpward = (*(temp.getResult())).Result;
  if (successUpward)
  {
    ROS_INFO("%s Bot is at desired height.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired height, something went wrong", ros::this_node::getName().c_str());
  }
}

void spinThreadDownwardPressure()
{
  ClientUpward &temp = *ptrClientUpward;
  temp.waitForResult();
  successDownward = (*(temp.getResult())).Result;
  if (successDownward)
  {
    ROS_INFO("%s Bot is at desired height.", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_INFO("%s Bot is not at desired height, something went wrong", ros::this_node::getName().c_str());
  }
}
// never ever put the argument of the callback function anything other then the specified
void buoyCB(task_commons::buoyActionFeedback msg)
{
  ROS_INFO("%s: x_coord = %f, y_coord = %f, distance = %f", ros::this_node::getName().c_str(), msg.feedback.x_coord,
           msg.feedback.y_coord, msg.feedback.distance);
}

void lineCB(task_commons::lineActionFeedback msg)
{
  ROS_INFO("%s: feedback recieved Angle Remaining = %f, x_coord = %f, y_coord = %f", ros::this_node::getName().c_str(),
           msg.feedback.AngleRemaining, msg.feedback.x_coord, msg.feedback.y_coord);
}

void pressureCB(std_msgs::Float64 pressure_sensor_data)
{
  present_depth = pressure_sensor_data.data;
  if (successBuoy)
    pub_upward.publish(pressure_sensor_data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");

  ros::NodeHandle nh;
  // here buoy_server is the name of the node of the actionserver.
  ros::Subscriber sub_buoy = nh.subscribe<task_commons::buoyActionFeedback>("/buoy_server/feedback", 1000, &buoyCB);
  ros::Subscriber sub_line = nh.subscribe<task_commons::lineActionFeedback>("/line_server/feedback", 1000, &lineCB);
  ros::Subscriber sub_upward =
      nh.subscribe<std_msgs::Float64>("/varun/sensors/pressure_sensor/depth", 1000, &pressureCB);
  pub_upward = nh.advertise<std_msgs::Float64>("/varun/motion/z_distance", 1000);
  pub_forward = nh.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);

  ClientBuoy buoyClient("buoy_server");
  ptrClientBuoy = &buoyClient;
  ClientLine lineClient("line_server");
  ptrClientLine = &lineClient;
  ClientUpward upwardClient("upward");
  ptrClientUpward = &upwardClient;
  ClientForward forwardClient("forward");
  ptrClientForward = &forwardClient;

  ROS_INFO("Waiting for action server to start.");
  lineClient.waitForServer();
  buoyClient.waitForServer();
  upwardClient.waitForServer();
  forwardClient.waitForServer();

  goalLine.order = true;
  ROS_INFO("Action server started, sending goal to line.");

  ClientLine &canLine = *ptrClientLine;
  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line(&spinThreadLine);

  while (!successLine)
  {
    ros::spinOnce();
  }

  goalBuoy.order = true;
  ROS_INFO("Action server started, sending goal to buoy.");

  ClientBuoy &canBuoy = *ptrClientBuoy;
  // send goal
  canBuoy.sendGoal(goalBuoy);
  boost::thread spin_thread_buoy(&spinThreadBuoy);

  while (!successBuoy)
  {
    ros::spinOnce();
  }

  goalUpward.Goal = present_depth + 10;  // add param
  goalUpward.loop = 10;
  ROS_INFO("Action server started, sending goal to upward.");

  ClientUpward &canUpward = *ptrClientUpward;
  // send goal
  canUpward.sendGoal(goalUpward);
  boost::thread spin_thread_upward(&spinThreadUpwardPressure);

  while (!successUpward)
  {
    ros::spinOnce();
  }

  goalForward.Goal = 0;
  goalForward.loop = 10;
  ROS_INFO("Action server started, sending goal to forward.");

  ClientForward &canForward = *ptrClientForward;
  // send goal
  canForward.sendGoal(goalForward);

  std_msgs::Float64 mock_forward;
  mock_forward.data = 250;
  pub_forward.publish(mock_forward);

  sleep(5);  // add param

  canForward.cancelGoal();

  goalUpward.Goal = present_depth - 11;  // add param
  goalUpward.loop = 10;
  ROS_INFO("Sending goal to upward.");

  ClientUpward &canDownward = *ptrClientUpward;
  // send goal
  canDownward.sendGoal(goalUpward);
  boost::thread spin_thread_downward(&spinThreadDownwardPressure);

  while (!successDownward)
  {
    ros::spinOnce();
  }

  successLine = false;
  goalLine.order = true;
  ROS_INFO("Sending goal to line.");

  // send goal
  canLine.sendGoal(goalLine);
  boost::thread spin_thread_line2(&spinThreadLine);

  while (!successLine)
  {
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}

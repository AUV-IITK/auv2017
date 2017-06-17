// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motion_upward/upwardConfig.h>

typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client;  // defining the Client type

Client *clientPointer;            // pointer for sharing client across threads
motion_commons::UpwardGoal goal;  // new goal object to send to action server

bool moving = false;
bool success = false;
ros::Publisher pressure_data_pub;

// New thread for recieving result, called from dynamic reconfig callback
// Result recieved, start next motion or if motion unsuccessful then do error
// handling
void spinThread()
{
  Client &temp = *clientPointer;
  temp.waitForResult();
  success = (*(temp.getResult())).Result;
  if (success)
  {
    ROS_INFO("%s: motion successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s: motion unsuccessful", ros::this_node::getName().c_str());
}

// dynamic reconfig; Our primary way of debugging
// Send new goal or cancel goal depending on input from GUI
void callback(motion_upward::upwardConfig &config, double level)
{
  ROS_INFO("%s: Reconfigure Request: %f %s %d", ros::this_node::getName().c_str(), config.double_param,
           config.bool_param ? "True" : "False", config.loop);
  Client &can = *clientPointer;
  if (!config.bool_param)
  {
    if (moving)
    {
      moving = false;
      can.cancelGoal();
      ROS_INFO("%s: Goal Cancelled", ros::this_node::getName().c_str());
    }
  }
  else
  {
    if (moving)
    {
      Client &can = *clientPointer;
      can.cancelGoal();
      ROS_INFO("%s: Goal Cancelled", ros::this_node::getName().c_str());
    }
    goal.Goal = config.double_param;
    goal.loop = config.loop;
    can.sendGoal(goal);
    boost::thread spin_thread(&spinThread);
    ROS_INFO("%s: Goal Send %f loop: %d", ros::this_node::getName().c_str(), goal.Goal, goal.loop);
    moving = true;
  }
}

void pressure_data_callback(std_msgs::Float64 msg)
{
  pressure_data_pub.publish(msg);
}

// Callback for Feedback from Action Server
void upwardCb(motion_commons::UpwardActionFeedback msg)
{
  ROS_INFO("%s feedback recieved %fsec remaining ", ros::this_node::getName().c_str(), msg.feedback.DepthRemaining);
}

int main(int argc, char **argv)
{
  // Initializing the node
  ros::init(argc, argv, "testupwardMotion");

  ros::NodeHandle nh;
  // Subscribing to feedback from ActionServer
  ros::Subscriber sub_ = nh.subscribe<motion_commons::UpwardActionFeedback>("/upward/feedback", 1000, &upwardCb);
  ros::Subscriber pressure_data_sub =
      nh.subscribe<std_msgs::Float64>("/varun/sensors/pressure_sensor/depth", 1000, &pressure_data_callback);
  pressure_data_pub = nh.advertise<std_msgs::Float64>("/varun/motion/z_distance", 1000);

  // Declaring a new ActionClient
  Client upwardTestClient("upward");
  // Saving pointer to use across threads
  clientPointer = &upwardTestClient;

  // Waiting for action server to start
  ROS_INFO("%s Waiting for action server to start.", ros::this_node::getName().c_str());
  upwardTestClient.waitForServer();

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_upward::upwardConfig> server;
  dynamic_reconfigure::Server<motion_upward::upwardConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  motion_upward::upwardConfig config;
  config.bool_param = false;
  callback(config, 0);

  // waiting for goal
  ros::spin();
  return 0;
}

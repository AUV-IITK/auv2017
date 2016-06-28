// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/ForwardActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motion_forward/forwardConfig.h>

typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client;  // defining the Client type

Client *clientPointer;             // pointer for sharing client across threads
motion_commons::ForwardGoal goal;  // new goal object to send to action server

bool moving = false;
bool success = false;

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
    ROS_INFO("motion successful");
  }
  else
    ROS_INFO("motion unsuccessful");
}

// dynamic reconfig; Our primary way of debugging
// Send new goal or cancel goal depending on input from GUI
void callback(motion_forward::forwardConfig &config, double level)
{
  ROS_INFO("Reconfigure Request: %f %s %d", config.double_param, config.bool_param ? "True" : "False", config.loop);
  Client &can = *clientPointer;
  if (!config.bool_param)
  {
    if (moving)
    {
      moving = false;
      can.cancelGoal();
      ROS_INFO("Goal Cancelled");
    }
  }
  else
  {
    if (moving)
    {
      Client &can = *clientPointer;
      can.cancelGoal();
      ROS_INFO("Goal Cancelled");
    }
    goal.Goal = config.double_param;
    goal.loop = config.loop;
    can.sendGoal(goal);
    boost::thread spin_thread(&spinThread);
    ROS_INFO("Goal Send %f loop:%d", goal.Goal, goal.loop);
    moving = true;
  }
}

// Callback for Feedback from Action Server
void forwardCb(motion_commons::ForwardActionFeedback msg)
{
  ROS_INFO("feedback recieved %fsec remaining ", msg.feedback.DistanceRemaining);
}

int main(int argc, char **argv)
{
  // Initializing the node
  ros::init(argc, argv, "testForwardMotion");

  ros::NodeHandle nh;
  // Subscribing to feedback from ActionServer
  ros::Subscriber sub_ = nh.subscribe<motion_commons::ForwardActionFeedback>("/forward/feedback", 1000, &forwardCb);

  // Declaring a new ActionClient
  Client forwardTestClient("forward");
  // Saving pointer to use across threads
  clientPointer = &forwardTestClient;

  // Waiting for action server to start
  ROS_INFO("Waiting for action server to start.");
  forwardTestClient.waitForServer();

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_forward::forwardConfig> server;
  dynamic_reconfigure::Server<motion_forward::forwardConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  motion_forward::forwardConfig config;
  config.bool_param = false;
  callback(config, 0);

  // waiting for goal
  ros::spin();
  return 0;
}

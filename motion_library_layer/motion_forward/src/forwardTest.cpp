// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
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

ros::Publisher ip_data_pub;
ros::Publisher ip_switch;
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
    ROS_INFO("%s motion successful", ros::this_node::getName().c_str());
  }
  else
    ROS_INFO("%s motion unsuccessful", ros::this_node::getName().c_str());
}

// dynamic reconfig; Our primary way of debugging
// Send new goal or cancel goal depending on input from GUI
void callback(motion_forward::forwardConfig &config, double level)
{
  ROS_INFO("%s Reconfigure Request: %f %s %d", ros::this_node::getName().c_str(), config.double_param,
           config.bool_param ? "True" : "False", config.loop);
  Client &can = *clientPointer;
  if (!config.bool_param)
  {
    if (moving)
    {
      moving = false;
      can.cancelGoal();
      ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
    }
    // stoping ip
    std_msgs::Bool msg;
    msg.data = true;
    ip_switch.publish(msg);
  }
  else
  {
    if (moving)
    {
      Client &can = *clientPointer;
      can.cancelGoal();
      ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
    }
    // starting ip
    std_msgs::Bool msg;
    msg.data = false;
    ip_switch.publish(msg);
    goal.Goal = config.double_param;
    goal.loop = config.loop;
    can.sendGoal(goal);
    boost::thread spin_thread(&spinThread);
    ROS_INFO("%s Goal Send %f loop:%d", ros::this_node::getName().c_str(), goal.Goal, goal.loop);
    moving = true;
  }
}

void ip_data_callback(std_msgs::Float64MultiArray array)
{
  std_msgs::Float64 data_forward;
  data_forward.data = array.data[1];
  ip_data_pub.publish(data_forward);
}

// Callback for Feedback from Action Server
void forwardCb(motion_commons::ForwardActionFeedback msg)
{
  ROS_INFO("%s feedback recieved %fsec remaining ", ros::this_node::getName().c_str(), msg.feedback.DistanceRemaining);
}

int main(int argc, char **argv)
{
  // Initializing the node
  ros::init(argc, argv, "testForwardMotion");

  ros::NodeHandle nh;
  // Subscribing to feedback from ActionServer
  ros::Subscriber sub_ = nh.subscribe<motion_commons::ForwardActionFeedback>("/forward/feedback", 1000, &forwardCb);
  ros::Subscriber ip_data_sub = nh.subscribe<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000, &ip_data_callback);
  ip_data_pub = nh.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);
  ip_switch = nh.advertise<std_msgs::Bool>("buoy_detection_switch", 1000);

  // Declaring a new ActionClient
  Client forwardTestClient("forward");
  // Saving pointer to use across threads
  clientPointer = &forwardTestClient;

  // Waiting for action server to start
  ROS_INFO("%s Waiting for action server to start.", ros::this_node::getName().c_str());
  forwardTestClient.waitForServer();

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_forward::forwardConfig> server;
  dynamic_reconfigure::Server<motion_forward::forwardConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  /*motion_forward::forwardConfig config;
  config.bool_param = false;
  callback(config, 0);*/

  // waiting for goal
  ros::spin();
  return 0;
}

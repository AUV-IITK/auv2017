// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/TurnActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motion_turn/turningConfig.h>

typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client;
Client *clientPointer;
motion_commons::TurnGoal goal;

bool goalSet = false;

// dynamic reconfig
void callback(motion_turn::turningConfig &config, double level)
{
  ROS_INFO("Reconfigure Request: %f %s %d", config.double_param, config.bool_param ? "True" : "False", config.loop);
  Client &can = *clientPointer;
  if (!config.bool_param)
  {
    if (goalSet)
    {
      goalSet = false;
      can.cancelGoal();
      ROS_INFO("Goal Cancelled");
    }
  }
  else
  {
    if (goalSet)
    {
      Client &can = *clientPointer;
      can.cancelGoal();
      ROS_INFO("Goal Cancelled");
    }
    goal.AngleToTurn = config.double_param;
    goal.loop = config.loop;
    can.sendGoal(goal);
    ROS_INFO("Goal Send %f loop:%d", goal.AngleToTurn, goal.loop);
    goalSet = true;
  }
}

// never ever put the argument of the callback function anything other then the
// specified
// void forwardCb(const motion_turn::ForwardActionFeedbackConstPtr msg){
void turnCb(motion_commons::TurnActionFeedback msg)
{
  ROS_INFO("feedback recieved, %f deg remaining ", msg.feedback.AngleRemaining);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testTurningXY");

  ros::NodeHandle nh;
  ros::Subscriber sub_ = nh.subscribe<motion_commons::TurnActionFeedback>("/turningXY/feedback", 1000, &turnCb);

  Client TurnTestClient("turningXY");
  clientPointer = &TurnTestClient;
  // this wait has to be implemented here so that we can wait for the server to
  // start
  ROS_INFO("Waiting for action server to start.");
  TurnTestClient.waitForServer();
  goal.AngleToTurn = 0;
  ROS_INFO("Action server started, sending goal.");

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_turn::turningConfig> server;
  dynamic_reconfigure::Server<motion_turn::turningConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}

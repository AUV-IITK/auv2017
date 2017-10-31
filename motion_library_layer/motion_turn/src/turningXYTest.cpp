// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/TurnActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motion_turn/turningConfig.h>

typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client;
Client *clientPointer;
motion_commons::TurnGoal goal;

ros::Publisher imu_data_pub;
bool goalSet = false;

// dynamic reconfig
void callback(motion_turn::turningConfig &config, double level)
{
  ROS_INFO("%s Reconfigure Request: %f %s %d", ros::this_node::getName().c_str(), config.double_param,
           config.bool_param ? "True" : "False", config.loop);
  Client &can = *clientPointer;
  if (!config.bool_param)
  {
    if (goalSet)
    {
      goalSet = false;
      can.cancelGoal();
      ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
    }
  }
  else
  {
    if (goalSet)
    {
      Client &can = *clientPointer;
      can.cancelGoal();
      ROS_INFO("%s Goal Cancelled", ros::this_node::getName().c_str());
    }
    goal.AngleToTurn = config.double_param;
    goal.loop = config.loop;
    can.sendGoal(goal);
    ROS_INFO("%s Goal Send %f loop:%d", ros::this_node::getName().c_str(), goal.AngleToTurn, goal.loop);
    goalSet = true;
  }
}

void imu_data_callback(std_msgs::Float64 msg)
{
  imu_data_pub.publish(msg);
}

// never ever put the argument of the callback function anything other then the
// specified
void turnCb(motion_commons::TurnActionFeedback msg)
{
  ROS_INFO("%s feedback recieved, %f deg remaining ", ros::this_node::getName().c_str(), msg.feedback.AngleRemaining);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testTurningXY");

  ros::NodeHandle nh;
  ros::Subscriber sub_ = nh.subscribe<motion_commons::TurnActionFeedback>("/turningXY/feedback", 1000, &turnCb);
  ros::Subscriber imu_data_sub = nh.subscribe<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000, &imu_data_callback);
  imu_data_pub = nh.advertise<std_msgs::Float64>("/varun/motion/yaw", 1000);

  Client TurnTestClient("turningXY");
  clientPointer = &TurnTestClient;
  // this wait has to be implemented here so that we can wait for the server to
  // start
  ROS_INFO("%s Waiting for action server to start.", ros::this_node::getName().c_str());
  TurnTestClient.waitForServer();
  goal.AngleToTurn = 0;
  ROS_INFO("%s Action server started, sending goal.", ros::this_node::getName().c_str());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_turn::turningConfig> server;
  dynamic_reconfigure::Server<motion_turn::turningConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  motion_turn::turningConfig config;
  config.bool_param = false;
  callback(config, 0);
  ros::spin();
  return 0;
}

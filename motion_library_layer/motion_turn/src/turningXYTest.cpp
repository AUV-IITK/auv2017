// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/TurnActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <motion_turn/turningConfig.h>

typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client;
Client *clientPointer;
motion_commons::TurnGoal goal;
#define TO_DEG(x) (x * 57.2957795131);
std_msgs::Float64 imu_data;
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

void yawCB(const sensor_msgs::Imu msg)
{
  float q0 = msg.orientation.w;
  float q1 = msg.orientation.x;
  float q2 = msg.orientation.y;
  float q3 = msg.orientation.z;
  imu_data.data = TO_DEG(atan2(2*q1*q2-2*q0*q3, 2*q0*q0+2*q1*q1-1));
  imu_data_pub.publish(imu_data);
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
  ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, &yawCB);
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
  /*motion_turn::turningConfig config;
  config.bool_param = false;
  callback(config, 0);*/
  ros::spin();
  return 0;
}

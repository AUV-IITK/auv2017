// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <task_commons/gateAction.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/UpwardActionResult.h>
#include <motion_commons/SidewardActionResult.h>
#include <string>

typedef actionlib::SimpleActionServer<task_commons::gateAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> ClientSideward;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> ClientTurn;

class TaskGateInnerClass
{
private:
  ros::NodeHandle nh_;
  Server gate_server_;
  std::string action_name_;
  std_msgs::Float64 data_X_;
  std_msgs::Float64 data_Y_;
  task_commons::gateFeedback feedback_;
  task_commons::gateResult result_;
  ros::Subscriber sub_gate_;
  ros::Subscriber sub_line_;
  ros::Subscriber yaw_sub_;
  ros::Publisher switch_gate_detection;
  ros::Publisher switch_line_detection;
  ros::Publisher yaw_pub_;
  ros::Publisher present_distance_;
  ros::Publisher present_X_;
  ros::Publisher present_Y_;
  ClientForward ForwardClient_;
  ClientSideward SidewardClient_;
  ClientUpward UpwardClient_;
  ClientTurn TurnClient_;
  motion_commons::ForwardGoal forwardgoal;
  motion_commons::SidewardGoal sidewardgoal;
  motion_commons::UpwardGoal upwardgoal;
  motion_commons::TurnGoal turngoal;
  bool success_gate, heightCenter, sideCenter, isOrange, success_line;

public:
  TaskGateInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
    : gate_server_(nh_, name, boost::bind(&TaskGateInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
    , SidewardClient_(node1)
    , UpwardClient_(node2)
    , TurnClient_(node3)
  {
    ROS_INFO("inside constructor");
    gate_server_.registerPreemptCallback(boost::bind(&TaskGateInnerClass::preemptCB, this));

    switch_gate_detection = nh_.advertise<std_msgs::Bool>("gate_detection_switch", 1000);
    switch_line_detection = nh_.advertise<std_msgs::Bool>("line_detection_switch", 1000);
    present_X_ = nh_.advertise<std_msgs::Float64>("/varun/motion/y_distance", 1000);
    present_Y_ = nh_.advertise<std_msgs::Float64>("/varun/motion/z_distance", 1000);
    present_distance_ = nh_.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/varun/motion/yaw", 1000);
    sub_gate_ =
        nh_.subscribe<std_msgs::Float64MultiArray>("/varun/ip/gate", 1000, &TaskGateInnerClass::gateNavigation, this);
    yaw_sub_ = nh_.subscribe<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000, &TaskGateInnerClass::yawCB, this);
    sub_line_ = nh_.subscribe<std_msgs::Bool>("lineDetection", 1000, &TaskGateInnerClass::lineDetectedListener, this);
    gate_server_.start();
  }

  ~TaskGateInnerClass(void)
  {
  }

  void yawCB(std_msgs::Float64 imu_data)
  {
    yaw_pub_.publish(imu_data);
  }

  void gateNavigation(std_msgs::Float64MultiArray array)
  {
    data_X_.data = array.data[0];
    data_Y_.data = array.data[1];
    present_X_.publish(data_X_);
    present_Y_.publish(data_Y_);
  }

  void lineDetectedListener(std_msgs::Bool msg)
  {
    if (msg.data)
      isOrange = true;
    else
      isOrange = false;
  }

  void preemptCB(void)
  {
    // Not actually preempting the goal because Prakhar did it in analysisCB
    ROS_INFO("Called when preempted from the client");
  }

  void spinThreadUpward()
  {
    ClientUpward &tempUpward = UpwardClient_;
    tempUpward.waitForResult();
    heightCenter = (*(tempUpward.getResult())).Result;
    if (heightCenter)
    {
      ROS_INFO("%s Bot is at height center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s Bot is not at height center, something went wrong", action_name_.c_str());
    }
  }

  void spinThreadSideward()
  {
    ClientSideward &tempSideward = SidewardClient_;
    tempSideward.waitForResult();
    sideCenter = (*(tempSideward.getResult())).Result;
    if (sideCenter)
    {
      ROS_INFO("%s: Bot is at side center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s Bot is not at side center, something went wrong", action_name_.c_str());
    }
  }

  void analysisCB(const task_commons::gateGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");
    heightCenter = false;
    sideCenter = false;
    isOrange = false;
    success_line = false;
    success_gate = false;
    ros::Rate looprate(12);
    if (!gate_server_.isActive())
      return;

    ROS_INFO("%s: Waiting for Forward server to start.", action_name_.c_str());
    ForwardClient_.waitForServer();
    SidewardClient_.waitForServer();
    UpwardClient_.waitForServer();
    TurnClient_.waitForServer();

    TaskGateInnerClass::startGateDetection();

    sidewardgoal.Goal = 0;
    sidewardgoal.loop = 50;
    SidewardClient_.sendGoal(sidewardgoal);
    boost::thread spin_thread_sideward(&TaskGateInnerClass::spinThreadSideward, this);

    // Stabilization of yaw
    turngoal.AngleToTurn = 0;
    turngoal.loop = 100000;
    TurnClient_.sendGoal(turngoal);

    upwardgoal.Goal = 0;
    upwardgoal.loop = 50;
    UpwardClient_.sendGoal(upwardgoal);
    boost::thread spin_thread_upward(&TaskGateInnerClass::spinThreadUpward, this);

    while (goal->order)
    {
      if (gate_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        gate_server_.setPreempted();
        success_gate = false;
        break;
      }
      looprate.sleep();
      if (heightCenter && sideCenter)
      {
        success_gate = true;
        std_msgs::Float64 init_data;
        init_data.data = 50;
        present_distance_.publish(init_data);
        break;
      }
      // publish the feedback
      feedback_.nosignificance = false;
      gate_server_.publishFeedback(feedback_);
      ROS_INFO("x = %f, y = %f", data_X_.data, data_Y_.data);
      ros::spinOnce();
    }

    forwardgoal.Goal = 10;
    forwardgoal.loop = 10;
    ForwardClient_.sendGoal(forwardgoal);  // stop motion here
    TaskGateInnerClass::stopGateDetection();

    TaskGateInnerClass::startLineDetection();

    while (goal->order)
    {
      if (gate_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        gate_server_.setPreempted();
        success_gate = false;
        break;
      }
      looprate.sleep();
      if (isOrange)
      {
        success_line = true;
        break;
      }
      // publish the feedback
      feedback_.nosignificance = false;
      gate_server_.publishFeedback(feedback_);
      ROS_INFO("%s:: x = %f, y = %f", action_name_.c_str(), data_X_.data, data_Y_.data);
      ros::spinOnce();
    }

    ForwardClient_.cancelGoal();  // stop motion here
    TaskGateInnerClass::stopLineDetection();

    result_.MotionCompleted = success_gate && success_line;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    gate_server_.setSucceeded(result_);
  }

  void startGateDetection()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_gate_detection.publish(msg);
  }

  void stopGateDetection()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_gate_detection.publish(msg);
  }

  void startLineDetection()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_line_detection.publish(msg);
  }

  void stopLineDetection()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_line_detection.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gate_server");
  ROS_INFO("Waiting for Goal");
  TaskGateInnerClass taskGateObject(ros::this_node::getName(), "forward", "sideward", "upward", "turningXY");
  ros::spin();
  return 0;
}

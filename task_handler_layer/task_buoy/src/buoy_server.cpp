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
#include <task_commons/buoyAction.h>
#include <motion_commons/ForwardAction.h>
// #include <motion_commons/TurnAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
// #include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <string>

typedef actionlib::SimpleActionServer<task_commons::buoyAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> ClientSideward;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
// typedef actionlib::SimpleActionClient<motion_commons::TurnAction> ClientTurn;

class TaskBuoyInnerClass
{
private:
  ros::NodeHandle nh_;
  Server buoy_server_;
  std::string action_name_;
  task_commons::buoyFeedback feedback_;
  task_commons::buoyResult result_;
  ros::Subscriber sub_;
  ros::Publisher off_pub_;
  ros::Publisher present_distance_;
  ros::Publisher present_X_;
  ros::Publisher present_Y_;
  ClientForward ForwardClient_;
  ClientSideward SidewardClient_;
  ClientUpward UpwardClient_;
  // ClientTurn TurnClient_;
  motion_commons::ForwardGoal forwardgoal;
  motion_commons::SidewardGoal sidewardgoal;
  motion_commons::UpwardGoal upwardgoal;
  // motion_commons::TurnGoal turngoal;
  bool TargetSet, success, Infront, forward_goal_set;

public:
  TaskBuoyInnerClass(std::string name, std::string node, std::string node1, std::string node2)
    : buoy_server_(nh_, name, boost::bind(&TaskBuoyInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
    , SidewardClient_(node1)
    , UpwardClient_(node2)
  // ,TurnClient_(node)
  {
    ROS_INFO("inside constructor");
    buoy_server_.registerPreemptCallback(boost::bind(&TaskBuoyInnerClass::preemptCB, this));

    off_pub_ = nh_.advertise<std_msgs::Bool>("buoy_detection_switch", 1000);
    present_X_ = nh_.advertise<std_msgs::Float64>("yDistance", 1000);
    present_Y_ = nh_.advertise<std_msgs::Float64>("zDistance", 1000);
    present_distance_ = nh_.advertise<std_msgs::Float64>("xDistance", 1000);
    sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/varun/sensors/front_camera/image_raw", 1000,
           &TaskBuoyInnerClass::buoyNavigation, this);
    buoy_server_.start();
  }

  ~TaskBuoyInnerClass(void)
  {
  }

  void buoyNavigation(std_msgs::Float64MultiArray array)
  {
    if (array.data[5])
    {
      TargetSet = true;
    }
    if (array.data[4])
    {
      Infront = true;
    }
    std_msgs::Float64 data1;
    std_msgs::Float64 data2;
    std_msgs::Float64 data3;
    data1.data = array.data[1];
    data2.data = array.data[2];
    data3.data = array.data[3];
    present_X_.publish(data1);
    present_Y_.publish(data2);
    present_distance_.publish(data3);
  }

  void preemptCB(void)
  {
    // Not actually preempting the goal because Prakhar did it in analysisCB
    ROS_INFO("Called when preempted from the client");
  }

  void analysisCB(const task_commons::buoyGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");
    TargetSet = false;
    Infront = false;
    forward_goal_set = false;
    ros::Rate looprate(12);
    if (!buoy_server_.isActive())
      return;

    ROS_INFO("Waiting for Forward server to start.");
    ForwardClient_.waitForServer();
    SidewardClient_.waitForServer();
    UpwardClient_.waitForServer();
    // TurnClient_.waitForServer();

    boost::thread vision_thread(&TaskBuoyInnerClass::startIP, this);
    TaskBuoyInnerClass::startIP();
    sidewardgoal.Goal = 0;
    SidewardClient_.sendGoal(sidewardgoal);
    upwardgoal.Goal = 0;
    UpwardClient_.sendGoal(upwardgoal);

    while (goal->order)
    {
      if (buoy_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        buoy_server_.setPreempted();
        success = false;
        break;
      }
      looprate.sleep();
      if (TargetSet)
      {
        // now need to go forward
        break;
      }
      if (Infront && !forward_goal_set)
      {
        forward_goal_set = true;
        forwardgoal.Goal = 0;
        ForwardClient_.sendGoal(forwardgoal);
      }
      // publish the feedback
      feedback_.nosignificance = false;
      buoy_server_.publishFeedback(feedback_);
      ROS_INFO("timeSpent");
      ros::spinOnce();
    }

    forwardgoal.Goal = 10;
    ForwardClient_.sendGoal(forwardgoal);  // stop motion here
    stopIP();

    if (success)
    {
      TargetSet = false;
      result_.MotionCompleted = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      buoy_server_.setSucceeded(result_);
    }
  }

  void startIP()
  {
    std_msgs::Bool msg;
    msg.data = false;
    off_pub_.publish(msg);
  }

  void stopIP()
  {
    std_msgs::Bool msg;
    msg.data = true;
    off_pub_.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "buoy_server");
  ROS_INFO("Waiting for Goal");
  TaskBuoyInnerClass taskBuoyObject(ros::this_node::getName(), "forward", "sideward", "upward");
  ros::spin();
  return 0;
}

// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <task_commons/octagonAction.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/SidewardAction.h>
#include <motion_commons/UpwardAction.h>
#include <motion_commons/TurnAction.h>
#include <motion_commons/SidewardActionFeedback.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/UpwardActionFeedback.h>
#include <motion_commons/TurnActionFeedback.h>
#include <motion_commons/SidewardActionResult.h>
#include <motion_commons/ForwardActionResult.h>
#include <motion_commons/TurnActionResult.h>
#include <motion_commons/UpwardActionResult.h>
#include <string>

typedef actionlib::SimpleActionServer<task_commons::octagonAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client_Forward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> Client_Sideward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client_Turn;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> Client_Upward;

class TaskoctagonInnerClass
{
private:
  ros::NodeHandle nh_;
  Server octagon_server_;
  std::string action_name_;
  task_commons::octagonFeedback feedback_;
  task_commons::octagonResult result_;
  ros::Subscriber detection_data;
  ros::Subscriber yaw_sub;
  ros::Subscriber centralize_data;
  ros::Publisher switch_centralize;
  ros::Publisher switch_detection;
  ros::Publisher present_X_;
  ros::Publisher present_Y_;
  ros::Publisher yaw_pub_;
  Client_Forward ForwardClient_;
  Client_Upward UpwardClient_;
  Client_Sideward SidewardClient_;
  Client_Turn TurnClient_;
  motion_commons::ForwardGoal forwardgoal;
  motion_commons::SidewardGoal sidewardgoal;
  motion_commons::TurnGoal turngoal;
  motion_commons::UpwardGoal upwardgoal;
  std_msgs::Float64 data_X_;
  std_msgs::Float64 data_Y_;
  bool isblue, success, FrontCenter, SideCenter, octagonAlign;

public:
  TaskoctagonInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
    :  // here we are defining the server, third argument is optional
    octagon_server_(nh_, name, boost::bind(&TaskoctagonInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
    , TurnClient_(node1)
    , SidewardClient_(node2)
    , UpwardClient_(node3)
  {
    ROS_INFO("inside constructor");
    octagon_server_.registerPreemptCallback(boost::bind(&TaskoctagonInnerClass::preemptCB, this));
    present_X_ = nh_.advertise<std_msgs::Float64>("/varun/motion/y_distance", 1000);
    present_Y_ = nh_.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);
    switch_detection = nh_.advertise<std_msgs::Bool>("octagon_detection_switch", 1000);
    switch_centralize = nh_.advertise<std_msgs::Bool>("octagon_centralize_switch", 1000);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/varun/motion/yaw", 1000);

    detection_data = nh_.subscribe<std_msgs::Bool>("/varun/ip/octagon_detection", 1000,
                                                   &TaskoctagonInnerClass::octagonDetectedListener, this);
    yaw_sub = nh_.subscribe<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000, &TaskoctagonInnerClass::yawCB, this);
    centralize_data = nh_.subscribe<std_msgs::Float64MultiArray>(
        "/varun/ip/octagon_centralize", 1000, &TaskoctagonInnerClass::octagonCentralizeListener, this);
    octagon_server_.start();
  }

  ~TaskoctagonInnerClass(void)
  {
  }

  void yawCB(std_msgs::Float64 imu_data)
  {
    yaw_pub_.publish(imu_data);
  }

  void octagonDetectedListener(std_msgs::Bool msg)
  {
    if (msg.data)
      isblue = true;
    else
      isblue = false;
  }

  void octagonCentralizeListener(std_msgs::Float64MultiArray array)
  {
    data_X_.data = array.data[0];
    data_Y_.data = array.data[1];
    present_X_.publish(data_X_);
    present_Y_.publish(data_Y_);
  }

  void spinThreadSidewardCamera()
  {
    Client_Sideward &tempSideward = SidewardClient_;
    tempSideward.waitForResult();
    SideCenter = (*(tempSideward.getResult())).Result;
    if (SideCenter)
    {
      ROS_INFO("%s: Bot is at side center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s: Bot is not at side center, something went wrong", action_name_.c_str());
      success = false;
    }
  }

  void spinThreadForwardCamera()
  {
    Client_Forward &tempForward = ForwardClient_;
    tempForward.waitForResult();
    FrontCenter = (*(tempForward.getResult())).Result;
    if (FrontCenter)
    {
      ROS_INFO("%s Bot is at Front center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s: Bot is not at Front center, something went wrong", action_name_.c_str());
      success = false;
    }
  }

  void spinThreadTurnCamera()
  {
    Client_Turn &tempTurn = TurnClient_;
    tempTurn.waitForResult();
    octagonAlign = (*(tempTurn.getResult())).Result;
    if (octagonAlign)
    {
      ROS_INFO("%s: Bot is aligned", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s: Bot is not aligned, something went wrong", action_name_.c_str());
      success = false;
    }
  }

  void preemptCB(void)
  {
    ROS_INFO("%s: Called when preempted from the client", action_name_.c_str());
  }

  void analysisCB(const task_commons::octagonGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");
    success = true;
    isblue = false;
    // octagonAlign = false;
    FrontCenter = false;
    SideCenter = false;
    ros::Rate looprate(12);

    if (!octagon_server_.isActive())
      return;

    ROS_INFO("%s Waiting for Forward server to start.", action_name_.c_str());
    ForwardClient_.waitForServer();
    SidewardClient_.waitForServer();
    TurnClient_.waitForServer();

    TaskoctagonInnerClass::detection_switch_on();

    // Stabilization of yaw
    turngoal.AngleToTurn = 0;
    turngoal.loop = 100000;
    TurnClient_.sendGoal(turngoal);
    // start moving forward.
    // send initial data to forward.
    forwardgoal.Goal = 100;
    forwardgoal.loop = 10;
    ForwardClient_.sendGoal(forwardgoal);
    ROS_INFO("%s: searching octagon", action_name_.c_str());

    while (goal->order && success)
    {
      if (octagon_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        octagon_server_.setPreempted();
        success = false;
        break;
      }
      looprate.sleep();

      if (isblue)
      {
        // stop will happen outside while loop so if preempted then too it will
        // stop
        break;
      }
      // publish the feedback

      // feedback_.AngleRemaining = angle_goal.data;
      feedback_.x_coord = data_X_.data;
      feedback_.y_coord = data_Y_.data;
      octagon_server_.publishFeedback(feedback_);
      ros::spinOnce();
    }

    ForwardClient_.cancelGoal();  // stop motion here
    TaskoctagonInnerClass::detection_switch_off();

    TaskoctagonInnerClass::centralize_switch_on();

    sidewardgoal.Goal = 0;
    sidewardgoal.loop = 10;
    SidewardClient_.sendGoal(sidewardgoal);
    boost::thread spin_thread_sideward_camera(&TaskoctagonInnerClass::spinThreadSidewardCamera, this);

    forwardgoal.Goal = 0;
    forwardgoal.loop = 10;
    ForwardClient_.sendGoal(forwardgoal);
    boost::thread spin_thread_forward_camera(&TaskoctagonInnerClass::spinThreadForwardCamera, this);
    ROS_INFO("%s: octagon is going to be centralized", action_name_.c_str());

    while (goal->order && success)
    {
      if (octagon_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        octagon_server_.setPreempted();
        success = false;
        break;
      }
      looprate.sleep();
      if (FrontCenter && SideCenter)
      {
        ROS_INFO("%s: octagon has been centralized.", action_name_.c_str());
        break;
      }
      // publish the feedback
      // feedback_.AngleRemaining = angle_goal.data;
      feedback_.x_coord = data_X_.data;
      feedback_.y_coord = data_Y_.data;
      octagon_server_.publishFeedback(feedback_);
      ros::spinOnce();
    }

    TaskoctagonInnerClass::centralize_switch_off();

    // TaskoctagonInnerClass::angle_switch_on();

    TurnClient_.cancelGoal();  // stopped stablisation

    // while (angle_goal.data == 0.0)
    // {
    //   sleep(.01);
    // }
    // turngoal.AngleToTurn = angle_goal.data;
    // turngoal.loop = 10;
    // TurnClient_.sendGoal(turngoal);
    // boost::thread spin_thread_turn_camera(&TaskoctagonInnerClass::spinThreadTurnCamera, this);
    // ROS_INFO("%s: octagon is going to be aligned", action_name_.c_str());

    // while (goal->order && success)
    // {
    //   if (octagon_server_.isPreemptRequested() || !ros::ok())
    //   {
    //     ROS_INFO("%s: Preempted", action_name_.c_str());
    //     // set the action state to preempted
    //     octagon_server_.setPreempted();
    //     success = false;
    //     break;
    //   }

    //   looprate.sleep();
    //   if (octagonAlign)
    //   {
    //     ROS_INFO("%s: octagon is aligned.", action_name_.c_str());
    //     break;
    //   }

    //   feedback_.AngleRemaining = angle_goal.data;
    //   feedback_.x_coord = data_X_.data;
    //   feedback_.y_coord = data_Y_.data;
    //   octagon_server_.publishFeedback(feedback_);
    //   ros::spinOnce();
    // }

    upwardgoal.Goal = 100;
    upwardgoal.loop = 10;
    UpwardClient_.sendGoal(upwardgoal);
    ROS_INFO("%s: coming out of the octagon", action_name_.c_str());

    result_.MotionCompleted = success;
    ROS_INFO("%s: Success is %s", action_name_.c_str(), success ? "true" : "false");
    octagon_server_.setSucceeded(result_);
  }

  void detection_switch_on()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_detection.publish(msg);
  }

  void detection_switch_off()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_detection.publish(msg);
  }

  void centralize_switch_on()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_centralize.publish(msg);
  }

  void centralize_switch_off()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_centralize.publish(msg);
  }
};
//***********************************************************************************//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octagon_server");

  ROS_INFO("Waiting for Goal");

  TaskoctagonInnerClass taskoctagonObject(ros::this_node::getName(), "forward", "turningXY", "sideward", "upward");

  ros::spin();
  return 0;
}

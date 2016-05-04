// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <task_commons/orangeAction.h>
#include <motion_commons/ForwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <string>
typedef actionlib::SimpleActionServer<task_commons::orangeAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> Client;

class task_line_detectionInnerClass
{
private:
  ros::NodeHandle nh_;
  Server task_line_detectionServer_;
  std::string action_name_;
  task_commons::orangeFeedback feedback_;
  task_commons::orangeResult result_;
  ros::Subscriber sub_;
  bool success;
  ros::Publisher off_pub_;
  bool isOrange;
  Client ForwardClient_;
  motion_commons::ForwardGoal forwardgoal;

public:
  task_line_detectionInnerClass(std::string name, std::string node)
    :  // here we are defining the server, third argument is optional
    task_line_detectionServer_(nh_, name, boost::bind(&task_line_detectionInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
  {
    ROS_INFO("inside constructor");
    task_line_detectionServer_.registerPreemptCallback(boost::bind(&task_line_detectionInnerClass::preemptCB, this));
    off_pub_ = nh_.advertise<std_msgs::Bool>("orangeoff", 1000);
    sub_ = nh_.subscribe<std_msgs::Bool>("linedetected",
      1000, &task_line_detectionInnerClass::lineDetectedListener, this);
    task_line_detectionServer_.start();
  }

  ~task_line_detectionInnerClass(void)
  {
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
    // Not actually preempting the goal because Shibhansh did it in analysisCB
    ROS_INFO("Called when preempted from the client");
  }

  void analysisCB(const task_commons::orangeGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");
    success = true;
    isOrange = false;
    ros::Rate looprate(12);

    if (!task_line_detectionServer_.isActive())
      return;

    ROS_INFO("Waiting for Forward server to start.");
    ForwardClient_.waitForServer();

    boost::thread vision_thread(&task_line_detectionInnerClass::startIP, this);
    // start moving forward.
    forwardgoal.Goal = 100;
    ForwardClient_.sendGoal(forwardgoal);
    while (goal->order)
    {
      if (task_line_detectionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        task_line_detectionServer_.setPreempted();
        success = false;
        break;
      }
      looprate.sleep();
      if (isOrange)
      {
        // stop will happen outside while loop so if preempted then too it will
        // stop
        break;
      }
      // publish the feedback
      feedback_.nosignificance = false;
      task_line_detectionServer_.publishFeedback(feedback_);
      ROS_INFO("timeSpent");
      ros::spinOnce();
    }
    ForwardClient_.cancelGoal();  // stop motion here
    stopIP();
    if (success)
    {
      isOrange = false;
      result_.MotionCompleted = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      task_line_detectionServer_.setSucceeded(result_);
    }
  }

  void startIP()
  {
    std::system("rosrun task_line_detection orangedetection");
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
  ros::init(argc, argv, "task_line_detectionserver");
  ROS_INFO("Waiting for Goal");
  task_line_detectionInnerClass task_line_detectionObject(ros::this_node::getName(), "forward");
  ros::spin();
  return 0;
}

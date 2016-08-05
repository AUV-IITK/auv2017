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

class TaskLineDetectionInnerClass
{
private:
  ros::NodeHandle nh_;
  Server line_detection_server_;
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
  TaskLineDetectionInnerClass(std::string name, std::string node)
    :  // here we are defining the server, third argument is optional
    line_detection_server_(nh_, name, boost::bind(&TaskLineDetectionInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
  {
    ROS_INFO("inside constructor");
    line_detection_server_.registerPreemptCallback(boost::bind(&TaskLineDetectionInnerClass::preemptCB, this));
    off_pub_ = nh_.advertise<std_msgs::Bool>("line_detection_switch", 1000);
    sub_ =
        nh_.subscribe<std_msgs::Bool>("lineDetection", 1000, &TaskLineDetectionInnerClass::lineDetectedListener, this);
    line_detection_server_.start();
  }

  ~TaskLineDetectionInnerClass(void)
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

    if (!line_detection_server_.isActive())
      return;

    ROS_INFO("Waiting for Forward server to start.");
    ForwardClient_.waitForServer();

    boost::thread vision_thread(&TaskLineDetectionInnerClass::startIP, this);
    // start moving forward.
    forwardgoal.Goal = 100;
    ForwardClient_.sendGoal(forwardgoal);
    while (goal->order)
    {
      if (line_detection_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        line_detection_server_.setPreempted();
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
      line_detection_server_.publishFeedback(feedback_);
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
      line_detection_server_.setSucceeded(result_);
    }
  }

  void startIP()
  {
    std::system("rosrun task_line_detection line_detection");
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
  ros::init(argc, argv, "line_detection_server");
  ROS_INFO("Waiting for Goal");
  TaskLineDetectionInnerClass taskLineDetectionObject(ros::this_node::getName(), "forward");
  ros::spin();
  return 0;
}

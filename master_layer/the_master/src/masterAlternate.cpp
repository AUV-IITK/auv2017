// Copyright 2016 AUV-IITK
#include <ros/ros.h>

#include <motion_commons/ForwardAction.h>
#include <motion_commons/ForwardActionFeedback.h>
#include <motion_commons/ForwardActionResult.h>

#include <task_commons/lineAction.h>
#include <task_commons/lineActionFeedback.h>
#include <task_commons/lineActionResult.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::lineAction> line;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> forward;

task_commons::lineGoal linegoal;
motion_commons::ForwardGoal forwardgoal;

bool lineSuccess = false;
bool forwardSuccess = false;

void spinThread()
{
  ros::spin();
}

void forwardCb(task_commons::lineActionFeedback msg)
{
  ROS_INFO("%s: feedback recieved %f", ros::this_node::getName().c_str(), msg.feedback.AngleRemaining);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_INFO("%s: please specify the time for forward motion\n", ros::this_node::getName().c_str());
    return 0;
  }
  float forwardTime;
  forwardTime = atof(argv[1]);

  ros::init(argc, argv, "TheMasterNode");
  ros::NodeHandle nh;
  // here task_commonsserver is the name of the node of the actionserver.
  ros::Subscriber subline =
      nh.subscribe<task_commons::lineActionFeedback>("/task_commonsserver/feedback", 1000, &forwardCb);

  line lineClient("task_line");

  forward forwardClient("forward");

  ROS_INFO("%s: Waiting for task_commons server to start.", ros::this_node::getName().c_str());
  lineClient.waitForServer();
  ROS_INFO("%s task_commons server started", ros::this_node::getName().c_str());

  boost::thread spin_thread(&spinThread);

  while (ros::ok())
  {
    linegoal.order = true;
    lineClient.sendGoal(linegoal);
    ROS_INFO("%s line detection started", ros::this_node::getName().c_str());
    lineClient.waitForResult();
    lineSuccess = (*(lineClient.getResult())).MotionCompleted;
    if (lineSuccess)
    {
      ROS_INFO("%s line colour detected", ros::this_node::getName().c_str());
    }
    else
      ROS_INFO("%s line not detected", ros::this_node::getName().c_str());

    forwardgoal.Goal = forwardTime;
    forwardClient.sendGoal(forwardgoal);
    ROS_INFO("%s Moving forward", ros::this_node::getName().c_str());
    forwardClient.waitForResult();
    forwardSuccess = (*(forwardClient.getResult())).Result;
    if (forwardSuccess)
    {
      ROS_INFO("%s forward motion successful", ros::this_node::getName().c_str());
    }
    else
      ROS_INFO("%s forward motion unsuccessful", ros::this_node::getName().c_str());
  }
  return 0;
}

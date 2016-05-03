// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <task_commons/alignAction.h>
#include <task_commons/alignActionFeedback.h>
#include <task_commons/alignActionResult.h>
#include <task_commons/orangeAction.h>
#include <task_commons/orangeActionFeedback.h>
#include <task_commons/orangeActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::orangeAction> orange;
typedef actionlib::SimpleActionClient<task_commons::alignAction> align;

task_commons::orangeGoal orangegoal;
task_commons::alignGoal aligngoal;

bool orangeSuccess = false;
bool alignSuccess = false;

void spinThread()
{
  ros::spin();
}

void forwardCb(task_commons::orangeActionFeedback msg)
{
  ROS_INFO("feedback recieved %d", msg.feedback.nosignificance);
}
void alignCb(task_commons::alignActionFeedback msg)
{
  ROS_INFO("feedback recieved %fsec remaining ", msg.feedback.AngleRemaining);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TheMasterNode");
  ros::NodeHandle nh;
  // here task_commonsserver is the name of the node of the actionserver.
  ros::Subscriber subOrange =
      nh.subscribe<task_commons::orangeActionFeedback>("/task_commonsserver/feedback", 1000, &forwardCb);
  ros::Subscriber subalign = nh.subscribe<task_commons::alignActionFeedback>("/align/feedback", 1000, &alignCb);

  orange orangeClient("task_commonsserver");

  align alignClient("align");

  ROS_INFO("Waiting for task_commons server to start.");
  orangeClient.waitForServer();
  ROS_INFO("task_commons server started");

  ROS_INFO("Waiting for align server to start.");
  alignClient.waitForServer();
  ROS_INFO("align server started.");

  boost::thread spin_thread(&spinThread);

  orangegoal.order = true;
  orangeClient.sendGoal(orangegoal);
  ROS_INFO("Orange detection started");
  orangeClient.waitForResult();
  orangeSuccess = (*(orangeClient.getResult())).MotionCompleted;
  if (orangeSuccess)
  {
    ROS_INFO("orange colour detected");
  }
  else
    ROS_INFO("orange not detected");

  aligngoal.StartDetection = true;
  alignClient.sendGoal(aligngoal);
  ROS_INFO("alignment started");
  alignClient.waitForResult();
  alignSuccess = (*(orangeClient.getResult())).MotionCompleted;
  if (alignSuccess)
  {
    ROS_INFO("alignment successful");
  }
  else
    ROS_INFO("alignment failed");
  return 0;
}
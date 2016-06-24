// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <task_commons/alignAction.h>
#include <task_commons/alignActionFeedback.h>
#include <task_commons/alignActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<task_commons::alignAction> Client;

Client *ptrClient;
task_commons::alignGoal goal;

void spinThread()
{
  Client &temp = *ptrClient;
  temp.waitForResult();
  bool success;
  success = (*(temp.getResult())).Aligned;
  if (success)
  {
    ROS_INFO("motion successful");
  }
  else
    ROS_INFO("motion unsuccessful");
  ros::shutdown();
}

// never ever put the argument of the callback function anything other then the
// specified
// void forwardCb(const motionlibrary::ForwardActionFeedbackConstPtr msg){
void alignCb(task_commons::alignActionFeedback msg)
{
  ROS_INFO("feedback recieved %fsec remaining ", msg.feedback.AngleRemaining);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_angle_client");

  ros::NodeHandle nh;
  ros::Subscriber sub_ = nh.subscribe<task_commons::alignActionFeedback>("/line_angle_server/feedback", 1000, &alignCb);

  Client testClient("line_angle_server");
  ptrClient = &testClient;

  ROS_INFO("Waiting for action server to start.");
  testClient.waitForServer();
  goal.StartDetection = true;
  ROS_INFO("Action server started, sending goal.");
  // Send Goal
  testClient.sendGoal(goal);
  ROS_INFO("Goal Send");

  // Here the thread is created and the ros node is started spinning in the
  // background.
  // By using this method you can create multiple threads for your action client
  // if needed.
  boost::thread spin_thread(&spinThread);

  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_commons/alignAction.h>

#include <motion_commons/TurnAction.h>
#include <motion_commons/TurnActionFeedback.h>

typedef actionlib::SimpleActionServer<task_commons::alignAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> Client;

class alignAction
{
private:
  ros::NodeHandle nh_;
  Server alignServer_;
  int loopRate;

  std::string action_name_;
  ros::Subscriber subIP_, subTurn_;
  ros::Publisher off_pub_;

  task_commons::alignResult result_;
  task_commons::alignFeedback feedback_;

  Client TurnClient_;
  motion_commons::TurnGoal goal;

  float lineAngle, feedbackFromTurn;
  bool intiData, success, turnActive, feedbackTurn, firstData;

public:
  alignAction(std::string name, std::string node)
    : alignServer_(nh_, name, boost::bind(&alignAction::analysisCB, this, _1), false)
    , action_name_(name)
    , TurnClient_(node)
  {
    alignServer_.registerPreemptCallback(boost::bind(&alignAction::preemptCB, this));
    subIP_ = nh_.subscribe("lineAngle", 100, &alignAction::lineCB, this);
    subTurn_ = nh_.subscribe("/TurnXY/feedback", 100, &alignAction::turnCB, this);
    off_pub_ = nh_.advertise<std_msgs::Bool>("lineoff", 1000);
    alignServer_.start();
  }

  ~alignAction(void)
  {
  }

  float modulus(float a, float b)
  {
    if (a > b)
      return a - b;
    else
      return b - a;
  }

  void lineCB(std_msgs::Float64 msg)
  {
    lineAngle = msg.data;
    // find the condition for here
    if (!firstData)
    {
      ROS_INFO("goal Cancelled sending new goal");
      goal.AngleToTurn = lineAngle;
      TurnClient_.sendGoal(goal);
      feedbackTurn = false;
      firstData = true;
    }
    if (feedbackTurn)
    {
      if (modulus(lineAngle, feedbackFromTurn) >= 10)
      {
        if (!intiData)
          intiData = true;
        // TurnClient_.cancelGoal();
        ROS_INFO("goal Cancelled sending new goal");
        goal.AngleToTurn = lineAngle;
        TurnClient_.sendGoal(goal);
        feedbackTurn = false;
      }
    }
  }

  void preemptCB(void)
  {
    if (turnActive)
    {
      TurnClient_.cancelGoal();
      turnActive = false;
    }
    ROS_INFO("Turn Goal Cancelled");
    alignServer_.setPreempted();
  }

  void turnCB(motion_commons::TurnActionFeedback msg)
  {
    ROS_INFO("feedback recieved %fdeg remaining ", msg.feedback.AngleRemaining);
    feedbackFromTurn = msg.feedback.AngleRemaining;
    feedback_.AngleRemaining = feedbackFromTurn;
    alignServer_.publishFeedback(feedback_);
    feedbackTurn = true;
  }

  void analysisCB(const task_commons::alignGoalConstPtr &target)
  {
    ROS_INFO("Inside analysisCB");
    intiData = false;
    success = false;
    turnActive = false;
    feedbackTurn = false;
    firstData = false;

    loopRate = 10;
    ros::Rate loop_rate(loopRate);

    if (!alignServer_.isActive())
    {
      ROS_INFO("alignServer_ is not active");
      return;
    }

    boost::thread vision_thread(&alignAction::startIP, this);
    ROS_INFO("Waiting for Turn server to start.");
    TurnClient_.waitForServer();
    turnActive = true;
    ROS_INFO("Turn server started");

    // boost::thread spin_thread(&alignAction::spinThread, this);
    ROS_INFO("waiting for data from edge detecting node");
    while (!intiData)
    {
      loop_rate.sleep();
    }
    ROS_INFO("got data form edge detecting node");

    // ensure that goal has been send here
    TurnClient_.waitForResult();
    success = (*(TurnClient_.getResult())).MotionCompleted;
    // make sure that target has been reached
    stopIP();
    if (success)
    {
      result_.Aligned = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded

      alignServer_.setSucceeded(result_);
    }
  }

  // void spinThread(){
  //  ros::spin();
  // }

  void startIP()
  {
    std::system("rosrun task_commons LineNew 1 0");
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
  ros::init(argc, argv, "align");
  ros::NodeHandle n;

  // Client Turn("TurnXY");
  alignAction align(ros::this_node::getName(), "TurnXY");
  ROS_INFO("Waiting for master command");

  ros::spin();
  return 0;
}
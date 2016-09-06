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
#include <task_commons/torpedoAction.h>
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

typedef actionlib::SimpleActionServer<task_commons::torpedoAction> Server;
typedef actionlib::SimpleActionClient<motion_commons::ForwardAction> ClientForward;
typedef actionlib::SimpleActionClient<motion_commons::SidewardAction> ClientSideward;
typedef actionlib::SimpleActionClient<motion_commons::UpwardAction> ClientUpward;
typedef actionlib::SimpleActionClient<motion_commons::TurnAction> ClientTurn;

class TaskBuoyInnerClass
{
private:
  ros::NodeHandle nh_;
  Server torpedo_server_;
  std::string action_name_;
  std_msgs::Float64 data_X_;
  std_msgs::Float64 data_Y_;
  std_msgs::Float64 data_distance_;
  task_commons::torpedoFeedback feedback_;
  task_commons::torpedoResult result_;
  ros::Subscriber sub_ip_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber pressure_sensor_sub;
  ros::Publisher switch_torpedo_detection;
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
  bool successBuoy, heightCenter, sideCenter, IP_stopped, heightGoal;
  float present_depth;

public:
  TaskBuoyInnerClass(std::string name, std::string node, std::string node1, std::string node2, std::string node3)
    : torpedo_server_(nh_, name, boost::bind(&TaskBuoyInnerClass::analysisCB, this, _1), false)
    , action_name_(name)
    , ForwardClient_(node)
    , SidewardClient_(node1)
    , UpwardClient_(node2)
    , TurnClient_(node3)
  {
    ROS_INFO("%s inside constructor", action_name_.c_str());
    torpedo_server_.registerPreemptCallback(boost::bind(&TaskBuoyInnerClass::preemptCB, this));

    switch_torpedo_detection = nh_.advertise<std_msgs::Bool>("torpedo_detection_switch", 1000);
    present_X_ = nh_.advertise<std_msgs::Float64>("/varun/motion/y_distance", 1000);
    present_Y_ = nh_.advertise<std_msgs::Float64>("/varun/motion/z_distance", 1000);
    present_distance_ = nh_.advertise<std_msgs::Float64>("/varun/motion/x_distance", 1000);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/varun/motion/yaw", 1000);
    sub_ip_ = nh_.subscribe<std_msgs::Float64MultiArray>("/varun/ip/torpedo", 1000,
                                                         &TaskBuoyInnerClass::torpedoNavigation, this);
    yaw_sub_ = nh_.subscribe<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000, &TaskBuoyInnerClass::yawCB, this);
    pressure_sensor_sub = nh_.subscribe<std_msgs::Float64>("/varun/sensors/pressure_sensor/depth", 1000,
                                                           &TaskBuoyInnerClass::pressureCB, this);
    torpedo_server_.start();
  }

  ~TaskBuoyInnerClass(void)
  {
  }

  void yawCB(std_msgs::Float64 imu_data)
  {
    yaw_pub_.publish(imu_data);
  }

  void pressureCB(std_msgs::Float64 pressure_sensor_data)
  {
    present_depth = pressure_sensor_data.data;
    if (successBuoy)
      present_Y_.publish(pressure_sensor_data);
  }

  void torpedoNavigation(std_msgs::Float64MultiArray array)
  {
    data_X_.data = array.data[1];
    data_Y_.data = array.data[2];
    data_distance_.data = array.data[3];
    present_X_.publish(data_X_);
    present_Y_.publish(data_Y_);

    if (data_distance_.data > 0)
      present_distance_.publish(data_distance_);

    else if (data_distance_.data < 0)
    {
      IP_stopped = true;
      stopBuoyDetection();
      ROS_INFO("%s Bot is in front of torpedo, IP stopped.", action_name_.c_str());
    }
  }

  void preemptCB(void)
  {
    // Not actually preempting the goal because Prakhar did it in analysisCB
    ROS_INFO("%s Called when preempted from the client", action_name_.c_str());
  }

  void spinThreadUpwardCamera()
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

  void spinThreadUpwardPressure()
  {
    ClientUpward &tempUpward = UpwardClient_;
    tempUpward.waitForResult();
    heightGoal = (*(tempUpward.getResult())).Result;
    if (heightGoal)
    {
      ROS_INFO("%s Bot is at desired height.", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s Bot is not at desired height, something went wrong", action_name_.c_str());
    }
  }

  void spinThreadSidewardCamera()
  {
    ClientSideward &tempSideward = SidewardClient_;
    tempSideward.waitForResult();
    sideCenter = (*(tempSideward.getResult())).Result;
    if (sideCenter)
    {
      ROS_INFO("%s Bot is at side center", action_name_.c_str());
    }
    else
    {
      ROS_INFO("%s Bot is not at side center, something went wrong", action_name_.c_str());
    }
  }

  void analysisCB(const task_commons::torpedoGoalConstPtr goal)
  {
    ROS_INFO("%s Inside analysisCB", action_name_.c_str());
    heightCenter = true;
    sideCenter = false;
    successBuoy = false;
    IP_stopped = false;
    heightGoal = false;
    ros::Rate looprate(12);
    if (!torpedo_server_.isActive())
      return;

    ROS_INFO("%s Waiting for Forward server to start.", action_name_.c_str());
    ForwardClient_.waitForServer();
    SidewardClient_.waitForServer();
    UpwardClient_.waitForServer();
    TurnClient_.waitForServer();

    TaskBuoyInnerClass::startBuoyDetection();

    sidewardgoal.Goal = 0;
    sidewardgoal.loop = 10;
    SidewardClient_.sendGoal(sidewardgoal);
    boost::thread spin_thread_sideward_camera(&TaskBuoyInnerClass::spinThreadSidewardCamera, this);

    // Stabilization of yaw
    turngoal.AngleToTurn = 0;
    turngoal.loop = 100000;
    TurnClient_.sendGoal(turngoal);

    upwardgoal.Goal = 0;
    upwardgoal.loop = 10;
    // UpwardClient_.sendGoal(upwardgoal);
    // boost::thread spin_thread_upward_camera(&TaskBuoyInnerClass::spinThreadUpwardCamera, this);

    while (goal->order)
    {
      if (torpedo_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        torpedo_server_.setPreempted();
        successBuoy = false;
        break;
      }
      looprate.sleep();
      if (heightCenter && sideCenter)
      {
        break;
      }
      // publish the feedback
      feedback_.nosignificance = false;
      torpedo_server_.publishFeedback(feedback_);
      ROS_INFO("%s :: x = %f, y = %f, front distance = %f", action_name_.c_str(), data_X_.data, data_Y_.data,
               data_distance_.data);
      ros::spinOnce();
    }

    ROS_INFO("%s Bot is in center of torpedo", action_name_.c_str());
    forwardgoal.Goal = 0;
    forwardgoal.loop = 10;
    ForwardClient_.sendGoal(forwardgoal);

    while (goal->order)
    {
      if (torpedo_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        torpedo_server_.setPreempted();
        successBuoy = false;
        break;
      }
      looprate.sleep();

      if (IP_stopped)
      {
        successBuoy = true;
        ROS_INFO("%s Waiting for hitting the torpedo...", action_name_.c_str());
        sleep(1);
        break;
      }

      feedback_.nosignificance = false;
      torpedo_server_.publishFeedback(feedback_);
      ROS_INFO("%s :: x = %f, y = %f, front distance = %f", action_name_.c_str(), data_X_.data, data_Y_.data,
               data_distance_.data);
      ros::spinOnce();
    }

    ForwardClient_.cancelGoal();  // stop motion here

    upwardgoal.Goal = present_depth + 5;
    upwardgoal.loop = 10;
    UpwardClient_.sendGoal(upwardgoal);
    ROS_INFO("%s moving upward", action_name_.c_str());
    boost::thread spin_thread_upward_pressure(&TaskBuoyInnerClass::spinThreadUpwardPressure, this);

    while (!heightGoal)
    {
      ROS_INFO("%s present depth = %f", action_name_.c_str(), present_depth);
      looprate.sleep();
      ros::spinOnce();
    }

    result_.MotionCompleted = successBuoy && heightGoal;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    torpedo_server_.setSucceeded(result_);
  }

  void startBuoyDetection()
  {
    std_msgs::Bool msg;
    msg.data = false;
    switch_torpedo_detection.publish(msg);
  }

  void stopBuoyDetection()
  {
    std_msgs::Bool msg;
    msg.data = true;
    switch_torpedo_detection.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torpedo_server");
  ROS_INFO("Waiting for Goal");
  TaskBuoyInnerClass taskBuoyObject(ros::this_node::getName(), "forward", "sideward", "upward", "turningXY");
  ros::spin();
  return 0;
}

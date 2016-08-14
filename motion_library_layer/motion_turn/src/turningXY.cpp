// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/TurnAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_turn/pidConfig.h>
#include <string>
using std::string;

typedef actionlib::SimpleActionServer<motion_commons::TurnAction> Server;
float presentAngularPosition = 0;
float previousAngularPosition = 0;
float finalAngularPosition, error, output;
bool initData = false;
std_msgs::Int32 pwm;  // pwm to be send to arduino

// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server turnServer_;
  std::string action_name_;
  motion_commons::TurnFeedback feedback_;
  motion_commons::TurnResult result_;
  ros::Publisher PWM;
  float p, i, d;

public:
  // Constructor, called when new instance of class declared
  explicit innerActionClass(std::string name)
    :  // Defining the server, third argument is optional
    turnServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false)
    , action_name_(name)
  {
    // Add preempt callback
    turnServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    PWM = nh_.advertise<std_msgs::Int32>("/pwm/turn", 1000);
    // Starting new Action Server
    turnServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled, stop the bot
  void preemptCB(void)
  {
    pwm.data = 0;
    PWM.publish(pwm);
    ROS_INFO("pwm send to arduino %d", pwm.data);
    // this command cancels the previous goal
    turnServer_.setPreempted();
  }

  // called when new goal recieved; start motion and finish it, if not interupted
  void analysisCB(const motion_commons::TurnGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from IMU else it's useless to any calculations
    while (!initData)
    {
      ROS_INFO("Waiting to get first input from IMU");
      loop_rate.sleep();
      ros::spinOnce();
    }

    finalAngularPosition = presentAngularPosition + goal->AngleToTurn;
    if (finalAngularPosition >= 180)
      finalAngularPosition = finalAngularPosition - 360;
    else if (finalAngularPosition <= -180)
      finalAngularPosition = finalAngularPosition + 360;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate;
    bool reached = false;
    pwm.data = 0;

    if (!turnServer_.isActive())
      return;

    while (!turnServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      error = finalAngularPosition - presentAngularPosition;
      integral += (error * dt);
      derivative = (presentAngularPosition - previousAngularPosition) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      turningOutputPWMMapping(output);

      if (error < 2 && error > -2)
      {
        reached = true;
        pwm.data = 0;
        PWM.publish(pwm);
        ROS_INFO("thrusters stopped");
        count++;
      }
      else
      {
        reached = false;
        count = 0;
      }

      if (turnServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        turnServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.AngleRemaining = error;
      turnServer_.publishFeedback(feedback_);
      PWM.publish(pwm);
      ROS_INFO("pwm send to arduino %d", pwm.data);

      ros::spinOnce();
      loop_rate.sleep();
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      turnServer_.setSucceeded(result_);
    }
  }
  void turningOutputPWMMapping(float output)
  {
    float maxOutput = 1000, minOutput = -maxOutput;
    float scale = 255 / maxOutput;
    if (output > maxOutput)
      output = maxOutput;
    if (output < minOutput)
      output = minOutput;
    float temp = output * scale;
    pwm.data = static_cast<int>(temp);
  }

  void setPID(float new_p, float new_i, float new_d)
  {
    p = new_p;
    i = new_i;
    d = new_d;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(motion_turn::pidConfig &config, double level)
{
  ROS_INFO("TurnServer: Reconfigure Request: p= %f i= %f d=%f", config.p, config.i, config.d);
  object->setPID(config.p, config.i, config.d);
}

void yawCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial position
  if (initData == false)
  {
    presentAngularPosition = msg.data;
    previousAngularPosition = presentAngularPosition;
    initData = true;
  }
  else
  {
    previousAngularPosition = presentAngularPosition;
    presentAngularPosition = msg.data;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turningXY");
  ros::NodeHandle n;
  double p_param, i_param, d_param;
  n.getParam("turningXY/p_param", p_param);
  n.getParam("turningXY/i_param", i_param);
  n.getParam("turningXY/d_param", d_param);

  ros::Subscriber yaw = n.subscribe<std_msgs::Float64>("/varun/motion/yaw", 1000, &yawCb);

  ROS_INFO("Waiting for Goal");
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_turn::pidConfig> server;
  dynamic_reconfigure::Server<motion_turn::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  motion_turn::pidConfig config;
  config.p = p_param;
  config.i = i_param;
  config.d = d_param;
  callback(config, 0);

  ros::spin();
  return 0;
}

// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/UpwardAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_upward/pidConfig.h>
#include <string>
using std::string;

typedef actionlib::SimpleActionServer<motion_commons::UpwardAction> Server;  // defining the Client type
float presentDepth = 0;
float previousDepth = 0;
float finalDepth, error, output;
bool initData = false;
std_msgs::Int32 pwm;  // pwm to be send to arduino

// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server upwardServer_;
  std::string action_name_;
  motion_commons::UpwardFeedback feedback_;
  motion_commons::UpwardResult result_;
  ros::Publisher PWM;
  float p, i, d;

public:
  // Constructor, called when new instance of class declared
  explicit innerActionClass(std::string name)
    :  // Defining the server, third argument is optional
    upwardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false)
    , action_name_(name)
  {
    // Add preempt callback
    upwardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    PWM = nh_.advertise<std_msgs::Int32>("/pwm/upward", 1000);
    // Starting new Action Server
    upwardServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled; Stop the bot
  void preemptCB(void)
  {
    pwm.data = 0;
    PWM.publish(pwm);
    ROS_INFO("pwm send to arduino %d", pwm.data);
    // this command cancels the previous goal
    upwardServer_.setPreempted();
  }

  // called when new goal recieved; Start motion and finish it, if not interrupted
  void analysisCB(const motion_commons::UpwardGoalConstPtr goal)
  {
    ROS_INFO("Inside analysisCB");

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from Camera/pressure sensor else it's useless do any calculations
    while (!initData)
    {
      ROS_INFO("Waiting to get first input at topic zDistance");
      loop_rate.sleep();
    }

    finalDepth = goal->Goal;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate;
    bool reached = false;
    pwm.data = 0;

    if (!upwardServer_.isActive())
      return;

    while (!upwardServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      error = finalDepth - presentDepth;
      integral += (error * dt);
      derivative = (presentDepth - previousDepth) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      upwardOutputPWMMapping(output);

      if (error <  0.01 && error > -0.01)
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

      if (upwardServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        upwardServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.DepthRemaining = error;
      upwardServer_.publishFeedback(feedback_);
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
      upwardServer_.setSucceeded(result_);
    }
  }

  void upwardOutputPWMMapping(float output)
  {
    const float maxOutput = 1000, minOutput = -maxOutput;
    const float scale = 255 / maxOutput;
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
void callback(motion_upward::pidConfig &config, double level)
{
  ROS_INFO("Reconfigure Request: p= %f i= %f d=%f", config.p, config.i, config.d);
  object->setPID(config.p, config.i, config.d);
}

void distanceCb(std_msgs::Float64 msg)
{
  // this is used to set the final depth after getting the value of first intial position
  if (initData == false)
  {
    presentDepth = msg.data;
    previousDepth = presentDepth;
    initData = true;
  }
  else
  {
    previousDepth = presentDepth;
    presentDepth = msg.data;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "upward");

  ros::NodeHandle n;
  ros::Subscriber zDistance = n.subscribe<std_msgs::Float64>("/varun/motion/z_distance", 1000, &distanceCb);

  ROS_INFO("Waiting for Goal");
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_upward::pidConfig> server;
  dynamic_reconfigure::Server<motion_upward::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}

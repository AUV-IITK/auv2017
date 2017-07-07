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
  float p, i, d, band, p_stablize, i_stablize, d_stablize, band_stablize, p_upward, i_upward, d_upward, band_upward;
  float maxPwm;

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
    ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    upwardServer_.setPreempted();
  }

  // called when new goal recieved; Start motion and finish it, if not interrupted
  void analysisCB(const motion_commons::UpwardGoalConstPtr goal)
  {
    ROS_INFO("%s: Inside analysisCB", ros::this_node::getName().c_str());

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from Camera/pressure sensor else it's useless do any calculations
    while (!initData)
    {
      ROS_INFO("%s: Waiting to get first input at topic zDistance", ros::this_node::getName().c_str());
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
      if (goal->loop > 10000)
      {
        p = p_stablize;
        i = i_stablize;
        d = d_stablize;
        band = band_stablize;
      }

      else
      {
        p = p_upward;
        i = i_upward;
        d = d_upward;
        band = band_upward;
      }

      error = finalDepth - presentDepth;
      integral += (error * dt);
      derivative = (presentDepth - previousDepth) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      upwardOutputPWMMapping(output);

      if (pwm.data <= band && pwm.data >= -band)
      {
        reached = true;
        pwm.data = 0;
        PWM.publish(pwm);
        ROS_INFO("%s: thrusters stopped", ros::this_node::getName().c_str());
        count++;
      }
      else
      {
        reached = false;
        count = 0;
      }

      if (upwardServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: %s: Preempted", ros::this_node::getName().c_str(), action_name_.c_str());
        // set the action state to preempted
        upwardServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.DepthRemaining = error;
      upwardServer_.publishFeedback(feedback_);
      PWM.publish(pwm);
      ROS_INFO("%s: pwm send to arduino upward %d", ros::this_node::getName().c_str(), pwm.data);

      ros::spinOnce();
      loop_rate.sleep();
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: %s: Succeeded", ros::this_node::getName().c_str(), action_name_.c_str());
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
    int output_pwm = static_cast<int>(temp);
    if (output_pwm > maxPwm)
      output_pwm = maxPwm;
    if (output_pwm < -maxPwm)
      output_pwm = -maxPwm;
    pwm.data = output_pwm;
  }

  void setPID(float new_max_pwm, float new_p_stablize, float new_p_upward, float new_i_stablize, float new_i_upward,
              float new_d_stablize, float new_d_upward, float new_band_stablize, float new_band_upward)
  {
    maxPwm = new_max_pwm;
    p_stablize = new_p_stablize;
    p_upward = new_p_upward;
    i_stablize = new_i_stablize;
    i_upward = new_i_upward;
    d_stablize = new_d_stablize;
    d_upward = new_d_upward;
    band_stablize = new_band_stablize;
    band_upward = new_band_upward;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(motion_upward::pidConfig &config, double level)
{
  ROS_INFO("%s: UpwardServer: Reconfigure Request: maxPwm=%f p_stablize=%f p_upward=%f "
           "i_stablize=%f i_upward=%f d_stablize=%f d_upward=%f error band_upward=%f",
           ros::this_node::getName().c_str(), config.maxPwm, config.p_stablize, config.p_upward, config.i_stablize,
           config.i_upward, config.d_stablize, config.d_upward, config.band_upward);
  object->setPID(config.maxPwm, config.p_stablize, config.p_upward, config.i_stablize, config.i_upward,
                 config.d_stablize, config.d_upward, config.band_stablize, config.band_upward);
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
  double p_stablize, p_upward, i_stablize, i_upward, d_stablize, d_upward, band_stablize, band_upward;
  double maxPwm;
  n.getParam("upward/maxPwm", maxPwm);
  n.getParam("upward/p_stablize", p_stablize);
  n.getParam("upward/p_upward", p_upward);
  n.getParam("upward/i_stablize", i_stablize);
  n.getParam("upward/i_upward", i_upward);
  n.getParam("upward/d_stablize", d_stablize);
  n.getParam("upward/d_upward", d_upward);
  n.getParam("upward/band_stablize", band_stablize);
  n.getParam("upward/band_upward", band_upward);

  ros::Subscriber zDistance = n.subscribe<std_msgs::Float64>("/varun/motion/z_distance", 1000, &distanceCb);

  ROS_INFO("%s: Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_upward::pidConfig> server;
  dynamic_reconfigure::Server<motion_upward::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  motion_upward::pidConfig config;
  config.maxPwm = maxPwm;
  config.p_stablize = p_stablize;
  config.p_upward = p_upward;
  config.i_stablize = i_stablize;
  config.i_upward = i_upward;
  config.d_stablize = d_stablize;
  config.d_upward = d_upward;
  config.band_stablize = band_stablize;
  config.band_upward = band_upward;
  callback(config, 0);

  ros::spin();
  return 0;
}

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
float finalAngularPosition, error, output, error_val;
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
  float p, i, d, band, p_stablize, i_stablize, d_stablize, p_turn, i_turn, d_turn, band_stablize, band_turn;

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
    ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    turnServer_.setPreempted();
  }

  // called when new goal recieved; start motion and finish it, if not
  // interupted
  void analysisCB(const motion_commons::TurnGoalConstPtr goal)
  {
    ROS_INFO("%s Inside analysisCB, Goal = %f", ros::this_node::getName().c_str(), goal->AngleToTurn);

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from IMU else it's useless to any
    // calculations
    while (!initData)
    {
      ROS_INFO("%s Waiting to get first input from IMU", ros::this_node::getName().c_str());
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
      if (goal->loop > 10000)
      {
        p = p_stablize;
        i = i_stablize;
        d = d_stablize;
        band = band_stablize;
      }

      else
      {
        p = p_turn;
        i = i_turn;
        d = d_turn;
        band = band_turn;
      }

      error = finalAngularPosition - presentAngularPosition;
      if (error < 0)
        error_val = error + 360;
      else
        error_val = error - 360;

      if (abs(error_val) < abs(error))
        error = error_val;

      integral += (error * dt);
      derivative = (presentAngularPosition - previousAngularPosition) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      turningOutputPWMMapping(output);

      if (error < band && error > -band)
      {
        reached = true;
        pwm.data = 0;
        PWM.publish(pwm);
        ROS_INFO("%s thrusters stopped", ros::this_node::getName().c_str());
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
      ROS_INFO("%s pwm send to arduino turn %d", ros::this_node::getName().c_str(), pwm.data);

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
    int output_pwm = static_cast<int>(temp);
    if (output_pwm > 255)
      output_pwm = 255;
    if (output_pwm < -255)
      output_pwm = -255;
    pwm.data = output_pwm;
  }

  void setPID(float new_p_stablize, float new_p_turn, float new_i_stablize, float new_i_turn, float new_d_stablize,
              float new_d_turn, float new_band_stablize, float new_band_turn)
  {
    p_stablize = new_p_stablize;
    p_turn = new_p_turn;
    i_stablize = new_i_stablize;
    i_turn = new_i_turn;
    d_stablize = new_d_stablize;
    d_turn = new_d_turn;
    band_stablize = new_band_stablize;
    band_turn = new_band_turn;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(motion_turn::pidConfig &config, double level)
{
  ROS_INFO("%s TurnServer: Reconfigure Request: p_stablize=%f p_turn=%f "
           "i_stablize=%f i_turn=%f d_stablize=%f d_turn=%f error band_turn=%f",
           ros::this_node::getName().c_str(), config.p_stablize, config.p_turn, config.i_stablize, config.i_turn,
           config.d_stablize, config.d_turn, config.band_turn);
  object->setPID(config.p_stablize, config.p_turn, config.i_stablize, config.i_turn, config.d_stablize, config.d_turn,
                 config.band_stablize, config.band_turn);
}

void yawCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial
  // position
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
  double p_stablize, p_turn, i_stablize, i_turn, d_stablize, d_turn, band_stablize, band_turn;
  n.getParam("turningXY/p_stablize", p_stablize);
  n.getParam("turningXY/p_turn", p_turn);
  n.getParam("turningXY/i_stablize", i_stablize);
  n.getParam("turningXY/i_turn", i_turn);
  n.getParam("turningXY/d_stablize", d_stablize);
  n.getParam("turningXY/d_turn", d_turn);
  n.getParam("turningXY/band_stablize", band_stablize);
  n.getParam("turningXY/band_turn", band_turn);

  ros::Subscriber yaw = n.subscribe<std_msgs::Float64>("/varun/motion/yaw", 1000, &yawCb);

  ROS_INFO("%s Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_turn::pidConfig> server;
  dynamic_reconfigure::Server<motion_turn::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  motion_turn::pidConfig config;
  config.p_stablize = p_stablize;
  config.p_turn = p_turn;
  config.i_stablize = i_stablize;
  config.i_turn = i_turn;
  config.d_stablize = d_stablize;
  config.d_turn = d_turn;
  config.band_stablize = band_stablize;
  config.band_turn = band_turn;
  callback(config, 0);

  ros::spin();
  return 0;
}

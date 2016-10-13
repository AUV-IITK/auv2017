// Copyright 2016 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_commons/SidewardAction.h>
#include <dynamic_reconfigure/server.h>
#include <motion_sideward/pidConfig.h>
#include <string>
using std::string;

typedef actionlib::SimpleActionServer<motion_commons::SidewardAction> Server;  // defining the Client type
float presentSidePosition = 0;
float previousSidePosition = 0;
float finalSidePosition, error, output;
bool initData = false;
std_msgs::Int32 pwm;  // pwm to be send to arduino

// new inner class, to encapsulate the interaction with actionclient
class innerActionClass
{
private:
  ros::NodeHandle nh_;
  Server sidewardServer_;
  std::string action_name_;
  motion_commons::SidewardFeedback feedback_;
  motion_commons::SidewardResult result_;
  ros::Publisher PWM;
  float p, i, d, band;

public:
  // Constructor, called when new instance of class declared
  explicit innerActionClass(std::string name)
    :  // Defining the server, third argument is optional
    sidewardServer_(nh_, name, boost::bind(&innerActionClass::analysisCB, this, _1), false)
    , action_name_(name)
  {
    // Add preempt callback
    sidewardServer_.registerPreemptCallback(boost::bind(&innerActionClass::preemptCB, this));
    // Declaring publisher for PWM
    PWM = nh_.advertise<std_msgs::Int32>("/pwm/sideward", 1000);
    // Starting new Action Server
    sidewardServer_.start();
  }

  // default contructor
  ~innerActionClass(void)
  {
  }

  // callback for goal cancelled ,Stop the bot
  void preemptCB(void)
  {
    pwm.data = 0;
    PWM.publish(pwm);
    ROS_INFO("%s pwm send to arduino %d", ros::this_node::getName().c_str(), pwm.data);
    // this command cancels the previous goal
    sidewardServer_.setPreempted();
  }

  // called when new goal recieved; start motion and finish it, if not interupted
  void analysisCB(const motion_commons::SidewardGoalConstPtr goal)
  {
    ROS_INFO("%s Inside analysisCB", ros::this_node::getName().c_str());

    int count = 0;
    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    // waiting till we recieve the first value from Camera else it's useless to any calculations
    while (!initData)
    {
      ROS_INFO("%s Waiting to get first input from Camera", ros::this_node::getName().c_str());
      loop_rate.sleep();
    }

    if (goal->Goal == 1)
      finalSidePosition = 0;

    float derivative = 0, integral = 0, dt = 1.0 / loopRate;
    bool reached = false;
    pwm.data = 0;

    if (!sidewardServer_.isActive())
      return;

    while (!sidewardServer_.isPreemptRequested() && ros::ok() && count < goal->loop)
    {
      error = finalSidePosition - presentSidePosition;
      integral += (error * dt);
      derivative = (presentSidePosition - previousSidePosition) / dt;
      output = (p * error) + (i * integral) + (d * derivative);
      sidewardOutputPWMMapping(output);

      if (pwm.data <= band && pwm.data >= -band)
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

      if (sidewardServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        sidewardServer_.setPreempted();
        reached = false;
        break;
      }

      feedback_.DistanceRemaining = error;
      sidewardServer_.publishFeedback(feedback_);
      PWM.publish(pwm);
      ROS_INFO("%s pwm send to arduino sideward %d", ros::this_node::getName().c_str(), pwm.data);

      ros::spinOnce();
      loop_rate.sleep();
    }
    if (reached)
    {
      result_.Result = reached;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      sidewardServer_.setSucceeded(result_);
    }
  }
  void sidewardOutputPWMMapping(float output)
  {
    float maxOutput = 1000, minOutput = -maxOutput;
    float scale = 255 / (maxOutput - 0);
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

  void setPID(float new_p, float new_i, float new_d, float new_band)
  {
    p = new_p;
    i = new_i;
    d = new_d;
    band = new_band;
  }
};
innerActionClass *object;

// dynamic reconfig
void callback(motion_sideward::pidConfig &config, double level)
{
  ROS_INFO("%s SidewardServer: Reconfigure Request: p= %f i= %f d=%f error band=%f", ros::this_node::getName().c_str(),
           config.p, config.i, config.d, config.band);
  object->setPID(config.p, config.i, config.d, config.band);
}

void distanceCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial position
  if (initData == false)
  {
    presentSidePosition = msg.data;
    previousSidePosition = presentSidePosition;
    initData = true;
  }
  else
  {
    previousSidePosition = presentSidePosition;
    presentSidePosition = msg.data;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sideward");
  ros::NodeHandle n;
  double p_param, i_param, d_param, band_param;
  n.getParam("sideward/p_param", p_param);
  n.getParam("sideward/i_param", i_param);
  n.getParam("sideward/d_param", d_param);
  n.getParam("sideward/band_param", band_param);

  ros::Subscriber yDistance = n.subscribe<std_msgs::Float64>("/varun/motion/y_distance", 1000, &distanceCb);

  ROS_INFO("%s Waiting for Goal", ros::this_node::getName().c_str());
  object = new innerActionClass(ros::this_node::getName());

  // register dynamic reconfig server.
  dynamic_reconfigure::Server<motion_sideward::pidConfig> server;
  dynamic_reconfigure::Server<motion_sideward::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  // set launch file pid
  motion_sideward::pidConfig config;
  config.p = p_param;
  config.i = i_param;
  config.d = d_param;
  config.band = band_param;
  callback(config, 0);

  ros::spin();
  return 0;
}

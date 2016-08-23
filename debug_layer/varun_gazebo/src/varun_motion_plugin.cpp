// Copyright 2016 AUV-IITK
#include <gazebo/common/Plugin.hh>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
const float THRUSTER_FORCE = 9.8;  // each thruster applies 1kgf. Gazebo uses SI units
const float FULL_FORCE = THRUSTER_FORCE * 2;
const float VARUN_SIDEWARD_LENGTH = 0.16;
const float ERROR_FACTOR = 100;
namespace gazebo
{
class VarunMotionPlugin : public WorldPlugin
{
private:
  ros::NodeHandle* _nh;
  ros::Subscriber _subPwmForward;
  ros::Subscriber _subPwmSideward;
  ros::Subscriber _subPwmUpward;
  ros::Subscriber _subPwmTurn;
  gazebo::physics::ModelPtr _model;

public:
  VarunMotionPlugin() : WorldPlugin()
  {
    _nh = new ros::NodeHandle("");
    // subscribe to motion library
    _subPwmForward = _nh->subscribe("/pwm/forward", 1, &VarunMotionPlugin::PWMCbForward, this);
    _subPwmSideward = _nh->subscribe("/pwm/sideward", 1, &VarunMotionPlugin::PWMCbSideward, this);
    _subPwmUpward = _nh->subscribe("/pwm/upward", 1, &VarunMotionPlugin::PWMCbUpward, this);
    _subPwmTurn = _nh->subscribe("/pwm/turn", 1, &VarunMotionPlugin::PWMCbTurn, this);
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    ROS_INFO("Varun Motion Plugin loaded");
    // Getting model for varun
    _model = _world->GetModel("varun");
  }

  void PWMCbForward(const std_msgs::Int32& msg)
  {
    math::Vector3 force;
    math::Vector3 reset_force;
    int pwm = msg.data;
    force.x = FULL_FORCE * pwm / 255 * ERROR_FACTOR;
    _model->GetLink("body")->SetForce(reset_force);
    _model->GetLink("body")->AddRelativeForce(force);
  }

  void PWMCbSideward(const std_msgs::Int32& msg)
  {
    math::Vector3 force;
    math::Vector3 reset_force;
    int pwm = msg.data;
    pwm = -pwm;  // this is because motionlibrary assumes right to be positive while gazebo y axis points to left.
    force.y = FULL_FORCE * pwm / 255 * ERROR_FACTOR;
    _model->GetLink("body")->SetForce(reset_force);
    _model->GetLink("body")->AddRelativeForce(force);
  }

  void PWMCbUpward(const std_msgs::Int32& msg)
  {
    math::Vector3 force;
    int pwm = msg.data;
    force.z = FULL_FORCE * pwm / 255 * ERROR_FACTOR;
    _model->GetLink("body")->SetForce(force);
  }

  void PWMCbTurn(const std_msgs::Int32& msg)
  {
    math::Vector3 torque;
    int pwm = msg.data;
    torque.z = FULL_FORCE * pwm / 255 * (VARUN_SIDEWARD_LENGTH / 2) * ERROR_FACTOR;
    _model->GetLink("body")->SetTorque(torque);
  }
};
GZ_REGISTER_WORLD_PLUGIN(VarunMotionPlugin)
}  // namespace gazebo

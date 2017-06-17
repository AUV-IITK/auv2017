// Copyright 2016 AUV-IITK
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include "math"
#include "DCM"
#include <time.h>

void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    // ROS_INFO(" YPR=");
    // ROS_INFO("%f\t",TO_DEG(yaw));
    // ROS_INFO("%f\t",TO_DEG(pitch));
    // ROS_INFO("%f\n",TO_DEG(roll));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dcm");
  ros::NodeHandle n;
  ros::Publisher out_pub = n.advertise<sensor_msgs::Imu>("imuyrp", 1000);
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("/varun/sensors/imu/yaw", 1000);
  sensor_msgs::Imu imu_msg;
  std_msgs::Float64 msg;

  // Read sensors, init DCM algorithm
  removegyrooff();

  reset_sensor_fusion();
  float Y = 0.0;
  int i = 0;
  ros::Rate loopRate(10);

  // float temp=0;
  timestamp = clock();

  while (ros::ok())
  {
    // Time to read the sensors again?
    if ((clock() - timestamp) >= OUTPUT__DATA_INTERVAL)
    {
      timestamp_old = timestamp;
      timestamp = clock();
      if (timestamp > timestamp_old)
      {
        G_Dt = static_cast<float>((timestamp - timestamp_old) / 10000.0f);  // Real time of loop run. We use this on the
                                                                            // DCM algorithm (gyro integration time)
        // temp+= G_Dt;
      }
      else
        G_Dt = 0;

      // cout << G_Dt << endl;
      // Update sensor readings
      read_sensors();

      // Run DCM algorithm
      Compass_Heading();  // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      output_angles();

      imu_msg = sensor_msgs::Imu();
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "imu";
      imu_msg.orientation.x = TO_DEG(pitch);
      imu_msg.orientation.y = TO_DEG(roll);
      imu_msg.orientation.z = TO_DEG(yaw);
      imu_msg.orientation.w = 0.0;
      imu_msg.orientation_covariance[0] = -1;
      imu_msg.angular_velocity.x = 0.0;
      imu_msg.angular_velocity.y = 0.0;
      imu_msg.angular_velocity.z = 0.0;
      imu_msg.angular_velocity_covariance[0] = -1;
      imu_msg.linear_acceleration.x = 0.0;
      imu_msg.linear_acceleration.y = 0.0;
      imu_msg.linear_acceleration.z = 0.0;
      imu_msg.linear_acceleration_covariance[0] = -1;

      msg.data = TO_DEG(yaw);
      chatter_pub.publish(msg);
      out_pub.publish(imu_msg);

      // cout << temp << endl;
      ROS_INFO("%s: %s %f", ros::this_node::getName().c_str(), "send an imu message", TO_DEG(yaw));
    }
    loopRate.sleep();
  }
}

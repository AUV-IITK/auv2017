#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from math import degrees
roll = 0.0
pitch = 0.0
yaw = 0.0


def varun_imu(pose):
    # Take out eular angles
    global roll, pitch, yaw
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    roll = degrees(euler[0])
    pitch = degrees(euler[1])
    yaw = degrees(euler[2])


def mock_imu_eular():
    "this fuction publishes imu data"
    global yaw
    rospy.init_node('gazebo_imu_eular')
    # Subscribe to imu_raw topic
    rospy.Subscriber('/varun/sensors/imu/imu_raw', Imu, varun_imu)
    # Publish imu data.
    pub_roll = rospy.Publisher(
        '/varun/sensors/imu/roll', Float64, queue_size=10)
    pub_pitch = rospy.Publisher(
        '/varun/sensors/imu/pitch', Float64, queue_size=10)
    pub_yaw = rospy.Publisher(
        '/varun/sensors/imu/yaw', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub_roll.publish(roll)
        pub_pitch.publish(pitch)
        pub_yaw.publish(yaw)
        rate.sleep()


if __name__ == '__main__':
    try:
        mock_imu_eular()
    except rospy.ROSInterruptException:
        pass

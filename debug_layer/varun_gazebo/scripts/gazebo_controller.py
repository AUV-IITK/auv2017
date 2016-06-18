#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
THRUSTER_FORCE = 9.8
MOTION_LIBRARY_FREQUENCY = 0.1
ERR_FACTOR = 1000
FULL_FORCE = THRUSTER_FORCE * 2 * ERR_FACTOR


def PWMCbForward(data):
    "Apply effort on varun on recieving forward pwm"
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    body_name = "varun::body"
    wrench = Wrench()
    pwm = data.data
    if abs(pwm) > 255:
        print "pwm can't have value more than 255"
    wrench.force.x = FULL_FORCE * pwm / 255  # each thruster exerts about 1 kgf
    duration = rospy.Duration()  # this is inverse of motion library update rate
    duration.secs = MOTION_LIBRARY_FREQUENCY
    duration.nsecs = 0
    apply_body_wrench(body_name, None, None, wrench, None, duration)


def PWMCbSideward(data):
    "Apply effort on varun on recieving sideward pwm"
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    body_name = "varun::body"
    wrench = Wrench()
    pwm = data.data
    # we are multiplying pwm with -1 to change its sign
    # Since we have defined positive pwm to move the bot in right but in the simulation y axis is towards the left of the bot
    pwm = pwm * -1
    if abs(pwm) > 255:
        print "pwm can't have value more than 255"
    wrench.force.y = FULL_FORCE * pwm / 255  # each thruster exerts about 1 kgf
    duration = rospy.Duration()  # this is inverse of motion library update rate
    duration.secs = MOTION_LIBRARY_FREQUENCY
    duration.nsecs = 0
    apply_body_wrench(body_name, None, None, wrench, None, duration)


def PWMCbUpward(data):
    "Apply effort on varun on recieving upward pwm"
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    body_name = "varun::body"
    wrench = Wrench()
    pwm = data.data
    if abs(pwm) > 255:
        print "pwm can't have value more than 255"
    wrench.force.z = FULL_FORCE * pwm / 255  # each thruster exerts about 1 kgf
    duration = rospy.Duration()  # this is inverse of motion library update rate
    duration.secs = MOTION_LIBRARY_FREQUENCY
    duration.nsecs = 0
    apply_body_wrench(body_name, None, None, wrench, None, duration)


def PWMCbTurn(data):
    "Apply wrench on varun on recieving turn pwm"
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    body_name = "varun::body"
    wrench = Wrench()
    pwm = data.data
    if abs(pwm) > 255:
        print "pwm can't have value more than 255"
    wrench.torque.z = FULL_FORCE * pwm / 255  # each thruster exerts about 1 kgf
    duration = rospy.Duration()  # this is inverse of motion library update rate
    duration.secs = MOTION_LIBRARY_FREQUENCY
    duration.nsecs = 0
    apply_body_wrench(body_name, None, None, wrench, None, duration)


def send_motion():
    "sends instructions to gazebo to do the appropriate motion"
    rospy.init_node('gazebo_controller')
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.Subscriber("/pwm/forward", Int32, PWMCbForward)
    rospy.Subscriber("/pwm/sideward", Int32, PWMCbSideward)
    rospy.Subscriber("/pwm/upward", Int32, PWMCbUpward)
    rospy.Subscriber("/pwm/turn", Int32, PWMCbTurn)
    rospy.spin()


if __name__ == '__main__':
    try:
        send_motion()
    except rospy.ROSInterruptException:
        pass

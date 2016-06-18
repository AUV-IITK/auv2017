#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def send_motion():
    rospy.init_node('listener')
    rospy.Subscriber("chatter", String, callback)


if __name__ == '__main__':
    try:
        send_motion()
    except rospy.ROSInterruptException:
        pass

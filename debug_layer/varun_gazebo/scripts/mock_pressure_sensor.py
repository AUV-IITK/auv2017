#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
depth = 0.0
WATER_HEIGHT = 40


def varun_height(data):
    # Take out z position
    global depth
    names = data.name
    index = names.index('varun')
    pose = data.pose[index]
    z_position = pose.position.z
    # negative because convention is to take upward direction as positive
    depth = -(WATER_HEIGHT - z_position)


def mock_pressure_sensor():
    "this fuction subscribes to gazebo model state topic and publishes depth data"
    global depth
    rospy.init_node('gazebo_pressure_sensor')
    # Subscribe to gazebo model state
    rospy.Subscriber('/gazebo/model_states', ModelStates, varun_height)
    # Publish pressure sensor data.
    pub = rospy.Publisher(
        '/varun/sensors/pressure_sensor/depth', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(depth)
        rate.sleep()


if __name__ == '__main__':
    try:
        mock_pressure_sensor()
    except rospy.ROSInterruptException:
        pass

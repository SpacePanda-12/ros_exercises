#!/usr/bin/env python
import numpy as np
import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
global rate

# dist_pub = rospy.Publisher('open_space', distance, queue_size=10)
# angle_pub = rospy.Publisher('open_space', angle, queue_size=10)
publisher_topic = rospy.get_param('open_space_publisher_topic')
subscriber_topic = rospy.get_param('open_space_subscriber_topic')
# pub = rospy.Publisher('open_space', OpenSpace, queue_size=10)
pub = rospy.Publisher(publisher_topic, OpenSpace, queue_size=10)

def callback(data):
    index = data.ranges.index(max(data.ranges), 0, len(data.ranges))
    angle = float(-2)/3*np.pi + float(index) * float(1)/300*np.pi
    msg = OpenSpace()
    msg.angle = angle
    msg.distance = max(data.ranges)
    pub.publish(msg)
    # dist_pub.publish(max(data.ranges))
    # angle_pub.publish(angle)
    # rospy.loginfo(max(data.ranges))
    # rospy.loginfo(angle)

    rate.sleep()


def simple_subscriber():
    global rate
    rospy.init_node('open_space_publisher')
    rate = rospy.Rate(20)
    # rospy.Subscriber("fake_scan", LaserScan, callback)
    rospy.Subscriber(subscriber_topic, LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass

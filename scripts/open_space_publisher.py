#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
global rate

publisher_topic = rospy.get_param('/open_space_publish_topic', 'open_space')
subscriber_topic = rospy.get_param('/open_space_subscriber_topic', 'fake_scan')

pub = rospy.Publisher(publisher_topic, OpenSpace, queue_size=10)

def callback(data):
    index = data.ranges.index(max(data.ranges), 0, len(data.ranges))
    angle = float(-2)/3*np.pi + float(index) * float(1)/300*np.pi
    msg = OpenSpace()
    msg.angle = angle
    msg.distance = max(data.ranges)
    pub.publish(msg)
    rate.sleep()


def simple_subscriber():
    global rate
    rospy.init_node('open_space_publisher')
    rate = rospy.Rate(20)
    rospy.Subscriber(subscriber_topic, LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass

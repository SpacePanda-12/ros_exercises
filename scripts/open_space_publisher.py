#!/usr/bin/env python
import numpy as np
import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
global rate

dist_pub = rospy.Publisher('open_space/distance', Float32, queue_size=10)
angle_pub = rospy.Publisher('open_space/angle', Float32, queue_size=10)



def callback(data):
    index = data.ranges.index(max(data.ranges), 0, len(data.ranges))
    angle = float(-2)/3*np.pi + float(index) * float(1)/300*np.pi
    rospy.loginfo(max(data.ranges))
    rospy.loginfo(angle)
    # dist_pub.publish()
    # angle_pub.publish()
    rate.sleep()


def simple_subscriber():
    global rate
    rospy.init_node('open_space_publisher')
    rate = rospy.Rate(20)
    rospy.Subscriber("fake_scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass

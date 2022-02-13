#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

dist_pub = rospy.Publisher('open_space/distance', Float32, queue_size=10)
angle_pub = rospy.Publisher('open_space/angle', Float32, queue_size=10)



def callback(data):
    rospy.loginfo(max(data.ranges))
    # dist_pub.publish()
    # angle_pub.publish()
    rate.sleep()

def simple_subscriber():
    rospy.init_node('open_space_publisher')
    rate = rospy.Rate(20)
    rospy.Subscriber("fake_scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass

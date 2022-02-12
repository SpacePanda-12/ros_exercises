#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import LaserScan
import random
pub = rospy.Publisher('random_float_log', Float32, queue_size=10)

def callback(data):
    number = math.log(data.data)
    rospy.loginfo(math.log(data.data))
    pub.publish(number)
    rospy.spin()


def simple_subscriber():
    rospy.init_node('simple_listener')
    rospy.Subscriber("my_random_float", Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    simple_subscriber()

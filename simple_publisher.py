#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import LaserScan
import random


def simple_publisher():
    pub = rospy.Publisher('my_random_float', Float32)
    rospy.init_node('simple_publisher', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        number = random.uniform(0, 10)
        pub.publish(number)
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass











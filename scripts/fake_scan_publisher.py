#!/usr/bin/env python
import math

import rospy
# from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import random


def fake_scan_publisher():
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
    rospy.init_node('fake_scan_publisher')
    rate = rospy.Rate(20)
    scan = LaserScan()
    scan.header.stamp = rospy.get_rostime()
    scan.header.frame_id = "base_link"
    scan.angle_min = (-2/3)*math.pi
    scan.angle_max = (-2/3)*math.pi
    scan.angle_increment = (1/300)*math.pi
    scan.scan_time = 1
    scan.range_min = 1
    scan.range_max = 10

    # array_length calculated as in below line. Hard coded value to remove style warning
    # array_length = 2*(2/3)*math.pi/(1/300*math.pi)+1
    array_length = 401
    array = []
    for i in range(array_length):
        n = random.uniform(1, 10)
        array.append(n)

    scan.ranges = array

    while not rospy.is_shutdown():
        pub.publish(scan)
        rate.sleep()


if __name__ == '__main__':
    try:
        fake_scan_publisher()
    except rospy.ROSInterruptException:
        pass

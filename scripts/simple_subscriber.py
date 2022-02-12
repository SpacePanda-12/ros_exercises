#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32

pub = rospy.Publisher('random_float_log', Float32, queue_size=10)

def callback(data):
    number = math.log(data.data)
    rospy.loginfo(math.log(data.data))
    pub.publish(number)


def simple_subscriber():
    rospy.init_node('simple_listener')
    rospy.Subscriber("my_random_float", Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass

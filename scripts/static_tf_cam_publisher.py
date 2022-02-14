#!/usr/bin/env python2
import numpy as np
import rospy
import tf2_ros
import tf
import geometry_msgs.msg

rospy.init_node("tf2_broadcast")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
r = rospy.Rate(20)

# precomputed transforms for the cameras
transform_matrix_left_from_robot = np.array([[float(1), float(0), float(0), float(-0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])
transform_matrix_right_from_robot = np.array([[float(1), float(0), float(0), float(0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])
br = tf2_ros.TransformBroadcaster()

transform_matrix_left = np.array([[float(1), float(0), float(0), float(0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])
transform_matrix_right = np.array([[float(1), float(0), float(0), float(-0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])

# left cam trans plus rot
robot_to_left_quaternion = tf.transformations.quaternion_from_matrix(transform_matrix_left)
robot_to_left_translation = np.array([transform_matrix_left[0][3], transform_matrix_left[1][3], transform_matrix_left[2][3]])

robot_to_left = geometry_msgs.msg.TransformStamped()
robot_to_left.header.stamp = rospy.Time.now()
robot_to_left.header.frame_id = "left_cam"
robot_to_left.child_frame_id = "base_link_gt"

# left cam translations
robot_to_left.transform.translation.x = robot_to_left_translation[0]
robot_to_left.transform.translation.y = robot_to_left_translation[1]
robot_to_left.transform.translation.z = robot_to_left_translation[2]

# left cam rotations
robot_to_left.transform.rotation.x = robot_to_left_quaternion[0]
robot_to_left.transform.rotation.y = robot_to_left_quaternion[1]
robot_to_left.transform.rotation.z = robot_to_left_quaternion[2]
robot_to_left.transform.rotation.w = robot_to_left_quaternion[3]

# right cam trans plus rot
robot_to_right_quaternion = tf.transformations.quaternion_from_matrix(transform_matrix_right)
robot_to_right_translation = np.array([transform_matrix_right[0][3], transform_matrix_right[1][3], transform_matrix_right[2][3]])

robot_to_right = geometry_msgs.msg.TransformStamped()
robot_to_right.header.stamp = rospy.Time.now()
robot_to_right.header.frame_id = "right_cam"
robot_to_right.child_frame_id = "base_link_gt"

# right cam translations
robot_to_right.transform.translation.x = robot_to_right_translation[0]
robot_to_right.transform.translation.y = robot_to_right_translation[1]
robot_to_right.transform.translation.z = robot_to_right_translation[2]

# right cam rotations
robot_to_right.transform.rotation.x = robot_to_right_quaternion[0]
robot_to_right.transform.rotation.y = robot_to_right_quaternion[1]
robot_to_right.transform.rotation.z = robot_to_right_quaternion[2]
robot_to_right.transform.rotation.w = robot_to_right_quaternion[3]

while not rospy.is_shutdown():
    br.sendTransform(robot_to_left)
    br.sendTransform(robot_to_right)

    r.sleep()
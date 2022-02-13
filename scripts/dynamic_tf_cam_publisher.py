#!/usr/bin/env python2
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node("tf2_broadcast")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
r = rospy.Rate(20)

# precomputed transforms for the cameras
transform_matrix_left = np.array([[1, 0, 0, -0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
transform_matrix_right = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
br = tf2_ros.TransformBroadcaster()


while not rospy.is_shutdown():

    # fetch transform from world to base_link
    try:
        tf2_ros.waitForTransform("world", "base_link_gt", rospy.Time())
        (translation, rotation) = tfBuffer.lookup_transform("world", "base_link_gt", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        r.sleep()
    
    # turn quaternion into rotation matrix
    rot_matrix = tf2_ros.transformations.quaternion_matrix(rotation)

    # create 4x4 transformation matrix
    robot_transform_matrix = np.array([[rot_matrix[0], translation[0]], [rot_matrix[1], translation[1]], [rot_matrix[2], translation[2]], [0, 0, 0, 1]])

    # compose world-to-robot then robot-to-left camera to get world-to-left camera
    world_to_left_camera_transform = np.multiply(robot_transform_matrix, transform_matrix_left)

    world_to_left_transform = geometry_msgs.msg.TransformStamped()
    world_to_left_transform.header.stamp = rospy.Time.now()
    world_to_left_transform.header.frame_id = "world"
    world_to_left_transform.child_frame_id = "left_cam"

    # left cam translations
    world_to_left_transform.transform.translation.x = transform_matrix_left[0][3]
    world_to_left_transform.transform.translation.y = transform_matrix_left[1][3]
    world_to_left_transform.transform.translation.z = transform_matrix_left[2][3]

    # left cam rotations
    world_to_left_quaternion = tf2_ros.quaternion_from_matrix(np.array([transform_matrix_left[0][0:2], transform_matrix_left[1][0:2], transform_matrix_left[2][0:2]]))

    world_to_left_transform.transform.rotation.x = world_to_left_quaternion[0]
    world_to_left_transform.transform.rotation.y = world_to_left_quaternion[1]
    world_to_left_transform.transform.rotation.z = world_to_left_quaternion[2]
    world_to_left_transform.transform.rotation.w = world_to_left_quaternion[3]



    # find transformation from left camera to robot. Since no rotation, it's just the opposite translation.
    left_to_robot_transform = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # compose inverse(robot-to-left) = left-to-robot with robot-to-right to get left-to-right transform
    left_to_right_transform = np.multiply(left_to_robot_transform, transform_matrix_right)
    left_to_right_quaternion = tf2_ros.quaternion_from_matrix(np.array([left_to_right_transform[0][0:2], left_to_right_transform[1][0:2], left_to_right_transform[2][0:2]]))
    left_to_right_translation = [left_to_right_transform[0][3], left_to_right_transform[1][3], left_to_right_transform[2][3]]

    left_to_right = geometry_msgs.msg.TransformStamped()
    left_to_right.header.stamp = rospy.Time.now()
    left_to_right.header.frame_id = "left_cam"
    left_to_right.header.child_frame_id = "right_cam"

    # right cam translations
    left_to_right.transform.translation.x = left_to_right_translation[0]
    left_to_right.transform.translation.y = left_to_right_translation[1]
    left_to_right.transform.translation.z = left_to_right_translation[2]

    # right cam rotations
    left_to_right.transform.rotation.x = left_to_right_quaternion[0]
    left_to_right.transform.rotation.y = left_to_right_quaternion[1]
    left_to_right.transform.rotation.z = left_to_right_quaternion[2]
    left_to_right.transform.rotation.w = left_to_right_quaternion[3]

    br.sendTransform(world_to_left_transform)
    br.sendTransform(left_to_right)










    r.sleep()


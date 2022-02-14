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
transform_matrix_left = np.array([[float(1), float(0), float(0), float(-0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])
transform_matrix_right = np.array([[float(1), float(0), float(0), float(0.5)], [float(0), float(1), float(0), float(0)], [float(0), float(0), float(1), float(0)], [float(0), float(0), float(0), float(1)]])
br = tf2_ros.TransformBroadcaster()


while not rospy.is_shutdown():

    # fetch transform from world to base_link
    try:
        transform = tfBuffer.lookup_transform("world", "left_cam", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        r.sleep()
        continue

    translation = transform.transform.translation
    translation = np.array([float(translation.x), float(translation.y), float(translation.z)])
    rotation = transform.transform.rotation
    rotation = np.array([float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)])

    # turn quaternion into rotation matrix
    rot_matrix = tf.transformations.quaternion_matrix(rotation)

    # create 4x4 transformation matrix
    world_to_left = [[rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2], translation[0]], [rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2], translation[1]], [rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2], translation[2]], [float(0), float(0), float(0), float(1)]]
    left_to_world = np.linalg.inv(world_to_left)
    base_to_world = np.dot(left_to_world, transform_matrix_left)

    base_to_world_transform = geometry_msgs.msg.TransformStamped()
    base_to_world_transform.header.stamp = rospy.Time.now()
    base_to_world_transform.header.frame_id = "base_link_gt_2"

    # left cam translations
    base_to_world_transform.transform.translation.x = base_to_world_transform[0][3]
    base_to_world_transform.transform.translation.y = base_to_world_transform[1][3]
    base_to_world_transform.transform.translation.z = base_to_world_transform[2][3]

    # left cam rotations
    base_to_world_quaternion = tf.transformations.quaternion_from_matrix(base_to_world_transform)
    base_to_world_transform.transform.rotation.x = base_to_world_quaternion[0]
    base_to_world_transform.transform.rotation.y = base_to_world_quaternion[1]
    base_to_world_transform.transform.rotation.z = base_to_world_quaternion[2]
    base_to_world_transform.transform.rotation.w = base_to_world_quaternion[3]

    br.sendTransform(base_to_world_transform)

    r.sleep()

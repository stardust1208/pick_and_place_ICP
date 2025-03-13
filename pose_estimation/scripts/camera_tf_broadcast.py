#!/usr/bin/env python3

import rospy
import tf2_ros
import tf.transformations
from geometry_msgs.msg import TransformStamped
import numpy as np

def broadcast_transform(parent_frame, child_frame, xyz, rpy):
    # Create a TransformBroadcaster
    br = tf2_ros.TransformBroadcaster()

    # Create a TransformStamped message
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    # Set translation (xyz)
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]

    # Convert RPY to quaternion
    quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

    # Set rotation (quaternion)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Publish the transform
    br.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node("camera_tf_broadcast")

    # Parent and child frames
    parent_frame = "rx150/base_link"
    child_frame = "camera_link"

    # Define the translation and rotation of the camera relative to the base link
    xyz = [0.245, 0.095, 0.45]  # Example translation (x, y, z) in meters
    rpy = [0.0, np.pi/2, 0] # Example rotation (roll, pitch, yaw) in radians

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Broadcast the transform
        broadcast_transform(parent_frame, child_frame, xyz, rpy)
        rate.sleep()

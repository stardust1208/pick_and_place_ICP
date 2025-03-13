#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

class ConvertPoint:
    def __init__(self):
        rospy.init_node("convert_point2BaseLink")
        self.sub = rospy.Subscriber("/camera/obj_poses", PoseStamped, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/rx150/obj_poses", PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2.0)  # Give time for TF buffer to populate

    def callback(self, msg: PoseStamped):
        self.tf_listener.waitForTransform("rx150/base_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
        # Transform the pose
        pose_A = self.tf_listener.transformPose("rx150/base_link", msg)
        self.pub.publish(pose_A)

if __name__ == "__main__":
    convertpoint = ConvertPoint()
    rospy.spin()

#!/usr/bin/env python3

import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

bot = InterbotixManipulatorXS("rx150", "arm", "gripper")

# def callback(msg : PoseStamped):                  # TEST
#     input("Press enter to execute")
#     bot.arm.set_ee_pose_components(x=0.15, z=0.25)
#     q = msg.pose.orientation
#     x = msg.pose.position.x
#     y = msg.pose.position.y

#     quaternion = [q.x, q.y, q.z, q.w]
#     rotation_matrix = R.from_quat(quaternion).as_matrix()  # 3x3 rotation matrix
#     z_axis_obj = rotation_matrix[:, 2]  # Third column of rotation matrix (Z-axis of obj)
#     x_axis_base = np.array([1, 0, 0])  # X-axis of base_link
#     cross_product = np.cross(x_axis_base, z_axis_obj)  # Cross product gives rotation direction
#     dot_product = np.dot(x_axis_base, z_axis_obj)
#     roll_obj = np.arccos(dot_product)
#     if cross_product[2] > 0:
#         roll_obj = -roll_obj
#     roll_obj = np.arctan2(y, x) + roll_obj
#     if roll_obj > np.pi/2:
#         roll_obj = roll_obj - np.pi
#     elif roll_obj < -np.pi/2:
#         roll_obj = roll_obj + np.pi

#     # bot.arm.set_ee_pose_components(x=x, y=y, z=0.07, roll = np.arctan2(y, x), pitch = np.pi/2)
#     bot.arm.set_ee_pose_components(x=x, y=y, z=0.07, roll = roll_obj, pitch = np.pi/2)
#     rospy.sleep(100.0)

def callback(msg : PoseStamped):                  # IMPLEMENT
    input("Press enter to execute")
    bot.arm.set_ee_pose_components(x=0.15, z=0.25)
    q = msg.pose.orientation
    x = msg.pose.position.x
    y = msg.pose.position.y

    quaternion = [q.x, q.y, q.z, q.w]
    rotation_matrix = R.from_quat(quaternion).as_matrix()  # 3x3 rotation matrix
    z_axis_obj = rotation_matrix[:, 2]  # Third column of rotation matrix (Z-axis of obj)
    x_axis_base = np.array([1, 0, 0])  # X-axis of base_link
    cross_product = np.cross(x_axis_base, z_axis_obj)  # Cross product gives rotation direction
    dot_product = np.dot(x_axis_base, z_axis_obj)
    roll_obj = np.arccos(dot_product)
    if cross_product[2] > 0:
        roll_obj = -roll_obj
    roll_obj = np.arctan2(y, x) + roll_obj
    if roll_obj > np.pi/2:
        roll_obj = roll_obj - np.pi
    elif roll_obj < -np.pi/2:
        roll_obj = roll_obj + np.pi

    # go to pre-pick position
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.08, roll = roll_obj, pitch = np.pi/2)
    # pick the object
    bot.gripper.open()
    # bot.arm.set_ee_pose_components(x=x, y=y, z=0.07, roll = roll_obj, pitch = np.pi/2)
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.025, roll = roll_obj, pitch = np.pi/2)
    bot.gripper.close()
    # go back to pre-pick position
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.08, roll = roll_obj, pitch = np.pi/2)
    # go to wait position
    bot.arm.set_ee_pose_components(x=0.15, z=0.25)
    # go to drop position
    bot.arm.set_ee_pose_components(x=-0.1, y=0.17, z=0.2)
    bot.gripper.open()
    rospy.sleep(1.0)
    bot.gripper.close()
    bot.arm.set_ee_pose_components(x=0.15, z=0.25)

def main():
    # rospy.init_node('pick_n_place')
    rospy.Subscriber('/rx150/obj_poses', PoseStamped, callback, queue_size=1)
    rospy.spin()

    # # TESTING
    # while not rospy.is_shutdown():
    #     bot.gripper.close()
    #     rospy.sleep(100.0)
    #     bot.gripper.close()
    #     bot.arm.go_to_home_pose()
    #     bot.arm.set_ee_pose_components(x=0.15, z=0.25)
    #     bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()

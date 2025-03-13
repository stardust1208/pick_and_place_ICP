#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
from helper_function import ICPTransform
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose
from msg_and_srv.msg import PointCloud2Set

class PointCloudProcessor:
    def __init__(self):
        self.err_list = np.array([])
        self.x_length = 0.025
        self.y_length = 0.06
        self.z_length = 0.12
        self.offset = 0.46
        self.step = 0.003
        self.max_iter = 35
        self.process = ICPTransform(self.x_length, self.y_length, self.z_length, self.offset, self.step, self.max_iter)
        self.model = self.process.create_uniform_box_points()
        self.xyz_clusters = []  # List of (M, 3) arrays, one for each cluster
        self.rgb_clusters = []  # List of (M, 3) arrays, one for each cluster
        self.sub = rospy.Subscriber("/camera/obj_points", PointCloud2Set, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/camera/obj_poses", PoseStamped, queue_size=1)

    def callback(self, msg: PointCloud2Set):
        self.xyz_clusters = []
        self.rgb_clusters = []
        i=0

        # # print("###############################################################")
        # new_msg = PoseStamped()
        # new_msg.header.stamp = rospy.Time.now()
        # new_msg.header.frame_id = "camera_color_optical_frame"
        # new_msg.pose = Pose()
        # new_msg.pose.position.x = 0
        # new_msg.pose.position.y = 0
        # new_msg.pose.position.z = 0.45

        # new_msg.pose.orientation.x = 0
        # new_msg.pose.orientation.y = 0
        # new_msg.pose.orientation.z = 0
        # new_msg.pose.orientation.w = 1
        # self.pub.publish(new_msg)


        # print("###############################################################")
        for pointcloud2_msg in msg.pointcloud2:
            xyz_array = self.pointcloud2_to_np_arrays(pointcloud2_msg)
            self.xyz_clusters.append(xyz_array)  # (3, N) array
            height, case = self.get_case(xyz_array)

            if case == 0:
                continue

            # new_xyz_array = self.process.project_pc_to_lower_plane(xyz_array, 0.01, case)
            new_xyz_array = self.process.project_pc_to_lower_plane_and_delete_mid_points(xyz_array, 0.01, case)
            MatObj2Camera, _, err = self.process.ICP(new_xyz_array, case)
            MatObj2Camera.rotation = self.process.align_axis_to_z(MatObj2Camera.rotation)

            # xang, yang, zang = self.process.angles_between_axes(MatObj2Camera.rotation)
            # new_xang, new_yang, new_zang = self.process.angles_between_axes(MatObj2Camera.rotation)

            if err > 0.003:
                continue

            # print("################################")
            # print("Cluster number: ", i, " case: ", case, " height: ", height)
            # print("Rotation: ", np.degrees(MatObj2Camera.rotation))
            # print("Translation: ", MatObj2Camera.translation)
            # print("Angle: ", np.rad2deg(xang), np.rad2deg(yang), np.rad2deg(zang))
            # print("Angle: ", np.rad2deg(new_xang), np.rad2deg(new_yang), np.rad2deg(new_zang))

            i+=1
            new_msg = PoseStamped()
            new_msg.header.stamp = rospy.Time.now()
            new_msg.header.frame_id = "camera_color_optical_frame"
            new_msg.pose = Pose()
            new_msg.pose.position.x = MatObj2Camera.translation[0][0]
            new_msg.pose.position.y = MatObj2Camera.translation[1][0]
            new_msg.pose.position.z = MatObj2Camera.translation[2][0]

            rotation = R.from_matrix(MatObj2Camera.rotation)
            quaternion = rotation.as_quat()

            new_msg.pose.orientation.x = quaternion[0]
            new_msg.pose.orientation.y = quaternion[1]
            new_msg.pose.orientation.z = quaternion[2]
            new_msg.pose.orientation.w = quaternion[3]

            self.pub.publish(new_msg)

            # print ("err: ", err)
            self.err_list = np.append(self.err_list, err)
        # print("###############################################################")



        # for i in range(0, msg.n_clusters):
        #     print("Cluster number: ", i)
        #     print(self.xyz_clusters[i].shape)
        #     print(self.rgb_clusters[i].shape)

    def pointcloud2_to_np_arrays(self, pointcloud2_msg: PointCloud2):
        points = pc2.read_points(pointcloud2_msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz_list = []
        for point in points:
            x, y, z = point
            # Append XYZ coordinates
            xyz_list.append([x, y, z])
        xyz_array = np.array(xyz_list)
        return xyz_array.T

    def get_case(self, points: np.array):
        if points is None:
            return 0, 0
        z_values = points[2, :]
        sorted_z = np.sort(z_values)
        bottom_5_percent_count = max(1, int(0.05 * len(sorted_z)))
        bottom_5_percent_z = sorted_z[:bottom_5_percent_count]
        mean_bottom_5_percent = np.mean(bottom_5_percent_z)
        # distance from upper plane to camera z axis
        if mean_bottom_5_percent > 0.326 and mean_bottom_5_percent < 0.350:
            case = 3
        elif mean_bottom_5_percent > 0.384 and mean_bottom_5_percent < 0.408:
            case = 2
        elif mean_bottom_5_percent > 0.419 and mean_bottom_5_percent < 0.443:
            case = 1
        else:
            case = 0
        estimated_height_from_camera = mean_bottom_5_percent
        return estimated_height_from_camera, case

if __name__ == "__main__":
    rospy.init_node("object_pose2Camera")
    processor = PointCloudProcessor()
    rospy.spin()
    print(processor.err_list.mean())

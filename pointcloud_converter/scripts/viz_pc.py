#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import rospy
from msg_and_srv.msg import PointCloudArr

# TEST to visualize pointcloud topic

class VIZPC:
    def __init__(self):
        self.sub = rospy.Subscriber("/pcarr/first_cluster", PointCloudArr, self.pc_arr_callback)
        self.xyz = None
        self.rgb = None
        self.minZ = 999

    def pc_arr_callback(self, msg: PointCloudArr):
        # rospy.loginfo("pc_arr_callback triggered.")
        self.xyz = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.rgb = np.array([msg.b, msg.g, msg.r], dtype=np.float32)
        # if self.xyz[2, :].min() < self.minZ:
        #     self.minZ = self.xyz[2, :].min()
        #     print(self.minZ)
        # np.save("plane_pc.npy", self.xyz)

    def get_numpy_arrays(self):
        return self.xyz, self.rgb

if __name__ == "__main__":
    rospy.init_node("viz_pc")
    vizpc = VIZPC()
    rospy.loginfo("Waiting for PointCloudArr data...")

    while True:
        input()
        xyz, rgb = vizpc.get_numpy_arrays()
        if xyz is not None and xyz.shape[1] == rgb.shape[1]:
            # Convert the arrays to the format required by Open3D (N x 3)
            points = xyz.T
            colors = (rgb / 255.0).T  # Normalize RGB to range [0, 1] and convert to N x 3

            # Add orgin points and axis xyz according to color rgb
            rootP = np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]])
            rootC = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
            points = np.vstack((points, rootP))
            colors = np.vstack((colors, rootC))

            # Create an Open3D PointCloud object
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points)
            point_cloud.colors = o3d.utility.Vector3dVector(colors)

            # Visualize the point cloud
            o3d.visualization.draw_geometries([point_cloud], window_name="Point Cloud Visualization")

        else:
            print("failed to get array")

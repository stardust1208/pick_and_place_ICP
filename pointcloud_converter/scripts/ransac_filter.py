#!/usr/bin/env python3

# TEST the ransac algorithm to remove plane-point cloud

import rospy
from msg_and_srv.msg import PointCloudArr
import numpy as np

"""
Using RANSAC algorithm to filter outliers point cloud (flat plane)
"""

class RANSAC():
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/arr/ds_points", PointCloudArr, self.callback)
        self.pub = rospy.Publisher("/camera/arr/obj_points", PointCloudArr, queue_size=1)
        self.xyz, self.rgb = None, None
        self.obj_xyz, self.obj_rgb = None, None
        self.rng = np.random.default_rng(135)
        self.tolerance = 0.01

    def callback(self, msg: PointCloudArr):
        x, y, z = np.array(msg.x), np.array(msg.y), np.array(msg.z)
        r, g, b = np.array(msg.r), np.array(msg.g), np.array(msg.b)
        # Create a 3xN array for XYZ
        self.xyz = np.vstack((x, y, z))
        # Create a 3xN array for RGB
        self.rgb = np.vstack((r, g, b))
        self.remove_plane()
        if self.obj_rgb is None or self.obj_rgb.shape != self.obj_rgb.shape:
            rospy.logerr("obj_rgb and obj_xyz must be the same")
        else:
            new_msg = PointCloudArr()
            new_msg.length = self.obj_xyz.shape[1]
            # Set the x, y, z coordinates
            msg.x = self.obj_xyz[0, :].tolist()
            msg.y = self.obj_xyz[1, :].tolist()
            msg.z = self.obj_xyz[2, :].tolist()
            # Set the RGB values
            msg.r = self.obj_rgb[0, :].tolist()
            msg.g = self.obj_rgb[1, :].tolist()
            msg.b = self.obj_rgb[2, :].tolist()
            self.pub.publish()

    def fit_plane(self, xyzs):
        """
        Args:
        xyzs is (3, N) numpy array
        Returns:
        (4,) numpy array
        """
        center = np.mean(xyzs, axis=1)
        cxyzs = xyzs.T - center
        U, S, V = np.linalg.svd(cxyzs)
        normal = V[-1]  # last row of V
        d = -center.dot(normal)
        plane_equation = np.hstack([normal, d])
        return plane_equation

    def ransac(self, model_fit_func, max_iterations=500):
        """
        Args:
            max_iterations is a (small) integer
            model_fit_func: the function to fit the model (point clouds)
        Returns:
            (4,) numpy array
        """
        best_ic = 0  # inlier count
        best_model = np.ones(4)  # plane equation ((4,) array)
        ###################################
        iteration = 0
        length = self.xyz.shape[1]
        while iteration < max_iterations:
            iteration = iteration + 1
            seed = self.rng.integers(low=0, high=length, size=3, dtype=int)
            p1 = self.xyz[:, seed[0]]
            p2 = self.xyz[:, seed[1]]
            p3 = self.xyz[:, seed[2]]
            vec1 = p2 - p1
            vec2 = p3 - p1
            normal = np.cross(vec1, vec2)
            if np.linalg.norm(normal) < 1e-8:
                iteration = iteration - 1
                continue
            d = -np.dot(normal, p1)
            a, b, c = normal
            dist = np.abs(a * self.xyz[0, :] + b * self.xyz[1, :] + c * self.xyz[2, :] + d)
            denom = np.sqrt(a ** 2 + b ** 2 + c ** 2)
            dist = dist/denom
            cnt = np.sum(dist < self.tolerance)
            if cnt > best_ic:
                best_ic = cnt
                mask = dist < self.tolerance
                plane_pcl = self.xyz[:, mask]
                best_model = model_fit_func(plane_pcl)
        ###################################
        return best_ic, best_model

    def remove_plane(self, tol=1e-4):
        _, plane = self.ransac(self.fit_plane, 500)
        a, b, c, d = plane
        dist = np.abs(a*self.xyz[0, :] + b*self.xyz[1, :] + c*self.xyz[2, :] + d)
        mask = dist > tol
        self.obj_xyz = self.xyz[:, mask]
        self.obj_rgb = self.rgb[:, mask]

if __name__ == '__main__':
    rospy.init_node("ransac_filter")
    ransac = RANSAC()
    rospy.spin()

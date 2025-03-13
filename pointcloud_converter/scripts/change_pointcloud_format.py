#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from msg_and_srv.msg import PointCloudArr
import sensor_msgs.point_cloud2 as pc2
import struct
import tf

# convert msg between PointCloud2 to PointCloudArr

class PointCloudSubscriber:
    def __init__(self):
        rospy.init_node('change_pointcloud_format')
        self.sub = rospy.Subscriber('/first_cluster', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/pcarr/first_cluster', PointCloudArr, queue_size=10)

    def filter_points(self, xyz, rgb, xRange=None, yRange=None, zRange=None):
        if xRange is None:
            xRange=np.array([-99, 99])
        if yRange is None:
            yRange=np.array([-99, 99])
        if zRange is None:
            zRange=np.array([-99, 99])
        # Boolean mask for points within the specified ranges
        mask = (
            (xyz[0, :] >= xRange[0]) & (xyz[0, :] <= xRange[1]) &  # x range
            (xyz[1, :] >= yRange[0]) & (xyz[1, :] <= yRange[1]) &  # y range
            (xyz[2, :] >= zRange[0]) & (xyz[2, :] <= zRange[1])    # z range
        )
        # Apply the mask to xyz and rgb
        xyzFiltered = xyz[:, mask]
        rgbFiltered = rgb[:, mask]
        return xyzFiltered, rgbFiltered

    def callback(self, msg : PointCloud2):
        # Get total number of points
        n_points = msg.width * msg.height
        # Initialize arrays (3xN)
        xyz = np.zeros((3, n_points))
        rgb = np.zeros((3, n_points))
        # Get points from point cloud
        pc_data = pc2.read_points(msg, skip_nans=True)
        # Fill arrays
        for i, point in enumerate(pc_data):
            # XYZ coordinates
            xyz[0, i] = point[0]  # X
            xyz[1, i] = point[1]  # Y
            xyz[2, i] = point[2]  # Z
            # RGB values (stored as float32, need to unpack)
            rgb_float = point[3]
            # Convert float to bytes
            rgb_bytes = struct.pack('f', rgb_float)
            # Unpack bytes to RGB
            r, g, b, _ = struct.unpack('BBBB', rgb_bytes)
            rgb[0, i] = r
            rgb[1, i] = g
            rgb[2, i] = b

        # min_values = np.min(xyz, axis=1)
        # max_values = np.max(xyz, axis=1)
        # print("Min: ", min_values)
        # print("Max: ", max_values)
        print(xyz.shape[1], rgb.shape[1])

        # eliminate x y outlier
        # xyz, rgb = self.filter_points(xyz, rgb, xRange=np.array([-0.18, 0.20]), yRange=np.array([-0.12, 0.15]))
        # eliminate x y z outlier
        # xyz, rgb = self.filter_points(xyz, rgb, xRange=np.array([-0.18, 0.20]), yRange=np.array([-0.12, 0.15]), zRange=np.array([0.0, 0.44]))
        # plant
        # xyz, rgb = self.filter_points(xyz, rgb, xRange=np.array([-0.2, 0.2]), yRange=np.array([-0.3, 0.3]), zRange=np.array([0, 0.45]))

        dataSend = PointCloudArr()
        dataSend.length = xyz.shape[1]
        dataSend.x = xyz[0, :].tolist()
        dataSend.y = xyz[1, :].tolist()
        dataSend.z = xyz[2, :].tolist()
        dataSend.r = rgb[0, :].tolist()
        dataSend.g = rgb[1, :].tolist()
        dataSend.b = rgb[2, :].tolist()
        self.pub.publish(dataSend)

if __name__ == '__main__':
    try:
        subscriber = PointCloudSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

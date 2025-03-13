#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

# access to PointCloud2 msg to inspect

def point_cloud_callback(msgs: PointCloud2):
    # Accessing the 'fields' inside the PointCloud2 message
    print(f"Header time: {msgs.header.stamp}")
    print(f"Header frame: {msgs.header.frame_id}")
    for field in msgs.fields:
        print(f"Field name: {field.name}")
        print(f"Field datatype: {field.datatype}")
        print(f"Field offset: {field.offset}")
        print(f"Field count: {field.count}")
        print("---------------------")
    print("----------------------------------------------------------------")

def listener():
    rospy.init_node('get_field_data')
    rospy.Subscriber('/camera/depth/color/ds_points', PointCloud2, point_cloud_callback)

if __name__ == '__main__':
    listener()
    rospy.spin()

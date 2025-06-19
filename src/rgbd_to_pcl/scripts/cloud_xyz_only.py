#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def callback(msg):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    # Extract only x, y, z
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    cloud_out = pc2.create_cloud_xyz32(header, list(points))

    pub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("cloud_xyz_only")
    pub = rospy.Publisher("/points_xyz", PointCloud2, queue_size=1)
    sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback)
    rospy.spin()

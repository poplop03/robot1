#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu

def imu_callback(msg):
    br = tf.TransformBroadcaster()
    # No translation, just orientation from IMU
    br.sendTransform(
        (0.0, 0.0, 0.0),
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        msg.header.stamp,
        "imu_link",      # child
        "base_link"      # parent
    )

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    rospy.Subscriber('/imu/raw', Imu, imu_callback)
    rospy.spin()
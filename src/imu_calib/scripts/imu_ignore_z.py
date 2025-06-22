#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

class IMUIgnoreZNode:
    def __init__(self):
        self.sub = rospy.Subscriber('/imu/butterfilter', Imu, self.imu_callback, queue_size=100)
        self.pub = rospy.Publisher('/imu/butterfilter/ignore_z', Imu, queue_size=100)
        rospy.loginfo("IMU Ignore Z Node (Z-accel only) started.")

    def imu_callback(self, msg):
        filtered_msg = Imu()
        filtered_msg.header = msg.header

        # Copy X and Y linear acceleration, zero out Z
        filtered_msg.linear_acceleration.x = msg.linear_acceleration.x
        filtered_msg.linear_acceleration.y = msg.linear_acceleration.y
        filtered_msg.linear_acceleration.z = 0.0  # Z-axis acceleration removed

        # Copy full angular velocity (including yaw)
        filtered_msg.angular_velocity = msg.angular_velocity

        # Copy full orientation (used for extrapolation)
        filtered_msg.orientation = msg.orientation

        # Copy covariances unchanged
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance

        self.pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('imu_ignore_z_node')
    node = IMUIgnoreZNode()
    rospy.spin()

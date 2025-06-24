#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

def normalize_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm == 0:
        rospy.logwarn("Quaternion norm is zero, skipping normalization.")
        return q
    return Quaternion(
        x=q.x / norm,
        y=q.y / norm,
        z=q.z / norm,
        w=q.w / norm
    )

class ImuNormalizer:
    def __init__(self):
        rospy.init_node('imu_normalizer', anonymous=True)
        
        self.sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=10)
        self.pub = rospy.Publisher('/imu/normalized', Imu, queue_size=10)

        rospy.loginfo("IMU Normalizer node started. Subscribing to /imu/data and publishing to /imu/normalized")

    def imu_callback(self, msg: Imu):
        # Normalize orientation quaternion
        normalized_orientation = normalize_quaternion(msg.orientation)

        # Create new IMU message
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.orientation = normalized_orientation
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(new_msg)

if __name__ == '__main__':
    try:
        ImuNormalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

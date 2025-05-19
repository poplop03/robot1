#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from scipy.signal import butter, lfilter_zi, lfilter
import math

deg_to_rad = math.pi / 180.0

class ButterworthFilter:
    def __init__(self, cutoff, fs, order=2):
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False)
        self.zi_ang = [lfilter_zi(self.b, self.a) * 0 for _ in range(3)]
        self.zi_acc = [lfilter_zi(self.b, self.a) * 0 for _ in range(3)]

    def filter_ang(self, data, i):
        filtered, self.zi_ang[i] = lfilter(self.b, self.a, [data], zi=self.zi_ang[i])
        return filtered[0]

    def filter_acc(self, data, i):
        filtered, self.zi_acc[i] = lfilter(self.b, self.a, [data], zi=self.zi_acc[i])
        return filtered[0]

class ImuButterFilterNode:
    def __init__(self):
        rospy.init_node('imu_butter_filter_node')

        self.fs = 50.0  # Fixed publish rate: 50 Hz
        self.cutoff = rospy.get_param("~cutoff_freq", 5.0)
        self.order = rospy.get_param("~filter_order", 2)

        self.filter = ButterworthFilter(self.cutoff, self.fs, self.order)

        self.latest_msg = None

        self.sub = rospy.Subscriber("/imu/corrected", Imu, self.imu_callback)
        self.pub = rospy.Publisher("/imu/butterfilter", Imu, queue_size=10)

        rospy.loginfo("Butterworth Filter Node running at 50 Hz (cutoff=%.2f Hz, order=%d)", self.cutoff, self.order)

    def imu_callback(self, msg):
        self.latest_msg = msg

    def spin(self):
        rate = rospy.Rate(50)  # 50 Hz loop
        while not rospy.is_shutdown():
            if self.latest_msg:
                msg = self.latest_msg

                filtered = Imu()
                filtered.header.stamp = rospy.Time.now()
                filtered.header.frame_id = "imu_link"

                # Orientation unchanged
                filtered.orientation = msg.orientation*deg_to_rad
                filtered.orientation_covariance = msg.orientation_covariance

                # Angular velocity
                filtered.angular_velocity.x = self.filter.filter_ang(msg.angular_velocity.x*deg_to_rad, 0)
                filtered.angular_velocity.y = self.filter.filter_ang(msg.angular_velocity.y*deg_to_rad, 1)
                filtered.angular_velocity.z = self.filter.filter_ang(msg.angular_velocity.z*deg_to_rad, 2)

                # Linear acceleration
                filtered.linear_acceleration.x = self.filter.filter_acc(msg.linear_acceleration.x, 0)
                filtered.linear_acceleration.y = self.filter.filter_acc(msg.linear_acceleration.y, 1)
                filtered.linear_acceleration.z = self.filter.filter_acc(msg.linear_acceleration.z, 2)

                self.pub.publish(filtered)

            rate.sleep()

if __name__ == '__main__':
    try:
        node = ImuButterFilterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

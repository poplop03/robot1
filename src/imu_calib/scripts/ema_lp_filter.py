#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
#prgrammed by duc
class ImuEmaFilterNode:
    def __init__(self):
        rospy.init_node('imu_ema_filter_node')

        # Filter smoothing factor (adjustable via param)
        self.alpha = rospy.get_param("~alpha", 0.1)

        # Previous filtered values
        self.prev_ang_vel = [0.0, 0.0, 0.0]
        self.prev_lin_acc = [0.0, 0.0, 0.0]

        # Subscribe to input IMU
        self.sub = rospy.Subscriber("/imu/corrected", Imu, self.imu_callback)

        # Publish filtered IMU
        self.pub = rospy.Publisher("/imu/ema_filtered", Imu, queue_size=10)

        rospy.loginfo("IMU EMA Filter Node started. Listening on /imu/corrected")

    def ema_filter(self, current, prev):
        return self.alpha * current + (1.0 - self.alpha) * prev

    def imu_callback(self, msg):
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = "imu_link"  # ensure consistent frame

        # Orientation: pass through (if needed, filter orientation separately)
        filtered_msg.orientation = msg.orientation
        filtered_msg.orientation_covariance = msg.orientation_covariance

        # Angular velocity
        filtered_msg.angular_velocity.x = self.ema_filter(msg.angular_velocity.x, self.prev_ang_vel[0])
        filtered_msg.angular_velocity.y = self.ema_filter(msg.angular_velocity.y, self.prev_ang_vel[1])
        filtered_msg.angular_velocity.z = self.ema_filter(msg.angular_velocity.z, self.prev_ang_vel[2])

        self.prev_ang_vel = [
            filtered_msg.angular_velocity.x,
            filtered_msg.angular_velocity.y,
            filtered_msg.angular_velocity.z,
        ]

        # Linear acceleration
        filtered_msg.linear_acceleration.x = self.ema_filter(msg.linear_acceleration.x, self.prev_lin_acc[0])
        filtered_msg.linear_acceleration.y = self.ema_filter(msg.linear_acceleration.y, self.prev_lin_acc[1])
        filtered_msg.linear_acceleration.z = self.ema_filter(msg.linear_acceleration.z, self.prev_lin_acc[2])

        self.prev_lin_acc = [
            filtered_msg.linear_acceleration.x,
            filtered_msg.linear_acceleration.y,
            filtered_msg.linear_acceleration.z,
        ]

        # Publish the filtered message
        self.pub.publish(filtered_msg)

if __name__ == '__main__':
    try:
        ImuEmaFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

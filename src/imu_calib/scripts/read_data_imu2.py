#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
#prgrammed by duc
class ImuRawSplitterNode:
    def __init__(self):
        rospy.init_node('imu_raw_splitter_node2')

        self.sub = rospy.Subscriber('/imu/imu2', Imu, self.imu_callback)

        # Angular velocity
        self.ang_x_pub = rospy.Publisher('/imu/imu2/angular_velocity/x', Float32, queue_size=10)
        self.ang_y_pub = rospy.Publisher('/imu/imu2/angular_velocity/y', Float32, queue_size=10)
        self.ang_z_pub = rospy.Publisher('/imu/imu2/angular_velocity/z', Float32, queue_size=10)

        # Linear acceleration
        self.acc_x_pub = rospy.Publisher('/imu/imu2/linear_acceleration/x', Float32, queue_size=10)
        self.acc_y_pub = rospy.Publisher('/imu/imu2/linear_acceleration/y', Float32, queue_size=10)
        self.acc_z_pub = rospy.Publisher('/imu/imu2/linear_acceleration/z', Float32, queue_size=10)

        rospy.loginfo("IMU Raw Splitter Node started (no orientation).")

    def imu_callback(self, msg):
        self.ang_x_pub.publish(msg.angular_velocity.x)
        self.ang_y_pub.publish(msg.angular_velocity.y)
        self.ang_z_pub.publish(msg.angular_velocity.z)

        self.acc_x_pub.publish(msg.linear_acceleration.x)
        self.acc_y_pub.publish(msg.linear_acceleration.y)
        self.acc_z_pub.publish(msg.linear_acceleration.z)

if __name__ == '__main__':
    try:
        ImuRawSplitterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

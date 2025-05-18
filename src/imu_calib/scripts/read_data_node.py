#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf.transformations

class ImuPlotPublisher:
    def __init__(self):
        rospy.init_node('imu_plot_publisher')

        # Subscribe to the filtered IMU data
        self.sub = rospy.Subscriber('/imu/complement_filtered', Imu, self.imu_callback)

        # Publishers for orientation (Euler angles)
        self.roll_pub  = rospy.Publisher('/imu/roll',  Float32, queue_size=10)
        self.pitch_pub = rospy.Publisher('/imu/pitch', Float32, queue_size=10)
        self.yaw_pub   = rospy.Publisher('/imu/yaw',   Float32, queue_size=10)

        # Publishers for angular velocity
        self.ang_x_pub = rospy.Publisher('/imu/angular_velocity/x', Float32, queue_size=10)
        self.ang_y_pub = rospy.Publisher('/imu/angular_velocity/y', Float32, queue_size=10)
        self.ang_z_pub = rospy.Publisher('/imu/angular_velocity/z', Float32, queue_size=10)

        # Publishers for linear acceleration
        self.acc_x_pub = rospy.Publisher('/imu/linear_acceleration/x', Float32, queue_size=10)
        self.acc_y_pub = rospy.Publisher('/imu/linear_acceleration/y', Float32, queue_size=10)
        self.acc_z_pub = rospy.Publisher('/imu/linear_acceleration/z', Float32, queue_size=10)

        rospy.loginfo("IMU plot publisher is running...")

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat)

        # Publish orientation
        self.roll_pub.publish(roll)
        self.pitch_pub.publish(pitch)
        self.yaw_pub.publish(yaw)

        # Publish angular velocity
        self.ang_x_pub.publish(msg.angular_velocity.x)
        self.ang_y_pub.publish(msg.angular_velocity.y)
        self.ang_z_pub.publish(msg.angular_velocity.z)

        # Publish linear acceleration
        self.acc_x_pub.publish(msg.linear_acceleration.x)
        self.acc_y_pub.publish(msg.linear_acceleration.y)
        self.acc_z_pub.publish(msg.linear_acceleration.z)

if __name__ == '__main__':
    try:
        ImuPlotPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

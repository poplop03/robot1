#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

def main():
    # Initialize the ROS node
    rospy.init_node('mpu6050_publisher', anonymous=True)
    
    # Create a publisher for sensor_msgs/Imu messages on the /imu topic
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    
    # Initialize the MPU6050 sensor (make sure the correct I2C address is used)
    mpu = mpu6050(0x69)
    
    # Set the loop rate (e.g., 100 Hz)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        # Create an empty Imu message
        imu_msg = Imu()

        # Fill in the header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # Read accelerometer data from the sensor
        accel_data = mpu.get_accel_data()
        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']

        # Read gyroscope data from the sensor
        gyro_data = mpu.get_gyro_data()
        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        # Optional: if you don't provide orientation data,
        # set the first element of orientation_covariance to -1 to indicate unk>
        imu_msg.orientation_covariance[0] = -1

        # Publish the IMU message
        imu_pub.publish(imu_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

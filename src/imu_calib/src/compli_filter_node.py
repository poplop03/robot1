#!/usr/bin/env python3
import rospy
import math
import tf
from sensor_msgs.msg import Imu

deg_to_rad = math.pi / 180.0

class ComplementaryFilterNode(object):
    def __init__(self):
        rospy.init_node('complementary_filter', anonymous=True)

        # Complementary filter constant: alpha near 1 favors gyro integration, lower values favor accelerometer.
        self.alpha = rospy.get_param("~alpha", 0.98)

        # Initialize filtered Euler angles (in radians)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.last_time = rospy.Time.now()

        # Subscribe to the calibrated IMU data on /corrected
        self.sub = rospy.Subscriber("/corrected", Imu, self.imu_callback)
        # Publisher for the filtered orientation
        self.pub = rospy.Publisher("/imu_filtered", Imu, queue_size=10)
        rospy.loginfo("Complementary Filter Node Initialized, subscribing to /corrected.")

    def imu_callback(self, msg):
        # Compute the time delta
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return
        self.last_time = current_time

        # Read calibrated linear acceleration (m/s^2)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Calculate roll and pitch from accelerometer.
        # Note: Assumes that the only acceleration is gravity (i.e., static or near static)
        pitch_acc = math.atan2(ay, az)
        roll_acc  = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # Read calibrated gyroscope data (rad/s)
        gx = msg.angular_velocity.x * deg_to_rad
        gy = msg.angular_velocity.y * deg_to_rad
        gz = msg.angular_velocity.z * deg_to_rad

        # Integrate gyro readings and blend with accelerometer angles.
        # Here, we assume gx affects pitch and gy affects roll.
        self.pitch = self.alpha * (self.pitch + gx * dt) + (1.0 - self.alpha) * pitch_acc
        self.roll  = self.alpha * (self.roll  + gy * dt) + (1.0 - self.alpha) * roll_acc
        # For yaw, we integrate gyro data directly (no accelerometer feedback available)
        self.yaw   = self.yaw + gz * dt

        # Convert the Euler angles (roll, pitch, yaw) to a quaternion.
        quat = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # Create a new IMU message with the filtered orientation.
        filtered_msg = Imu()
        filtered_msg.header = msg.header  # Preserve timestamp and frame_id
        filtered_msg.orientation.x = quat[0]
        filtered_msg.orientation.y = quat[1]
        filtered_msg.orientation.z = quat[2]
        filtered_msg.orientation.w = quat[3]

        # Optionally, include the original gyro and accel data.
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration

        # Publish the filtered IMU message.
        self.pub.publish(filtered_msg)
        
        ###################################################
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (0, 0, 0),  # IMU position (assuming it's at the origin of base_link)
            quat,  # IMU orientation from complementary filter
            rospy.Time.now(),
            "imu_link",  # Child frame
            "base_link"  # Parent frame (change this if needed)
        )
        ###################################################
        


if __name__ == '__main__':
    try:
        node = ComplementaryFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

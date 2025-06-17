#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import serial
import struct
import math

deg_to_rad = math.pi / 180.0
GYRO_SCALE = 131.0  # LSB/(deg/s) for ±250°/s on MPU6050

def parse_teapot_packet(packet):
    # packet: 28 bytes
    # [10:16] gyro, [16:22] accel
    gx = struct.unpack('>h', packet[10:12])[0]
    gy = struct.unpack('>h', packet[12:14])[0]
    gz = struct.unpack('>h', packet[14:16])[0]
    ax = struct.unpack('>h', packet[16:18])[0]
    ay = struct.unpack('>h', packet[18:20])[0]
    az = struct.unpack('>h', packet[20:22])[0]
    return ax, ay, az, gx, gy, gz

def main():
    rospy.init_node('mpu6050_teapot_serial_publisher', anonymous=True)
    imu_pub = rospy.Publisher('/imu/raw', Imu, queue_size=10)

    # Get parameters from ROS param server (settable in launch file)
    port = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 115200)

    ser = serial.Serial(port, baudrate, timeout=1)

    PACKET_SIZE = 28
    HEADER = b'\x24\x03'  # '$', 0x03

    while not rospy.is_shutdown():
        # Find start of teapot packet
        while True:
            c = ser.read(1)
            if not c:
                continue
            if c == HEADER[0:1]:
                next_c = ser.read(1)
                if next_c == HEADER[1:2]:
                    packet = HEADER + ser.read(PACKET_SIZE - 2)
                    if len(packet) == PACKET_SIZE:
                        break

        ax, ay, az, gx, gy, gz = parse_teapot_packet(packet)

        # Convert raw gyro to degree per second
        gx_dps = gx / GYRO_SCALE
        gy_dps = gy / GYRO_SCALE
        gz_dps = gz / GYRO_SCALE

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # If you want to publish in rad/s (as per ROS standard), multiply by deg_to_rad
        imu_msg.angular_velocity.x = gx_dps * deg_to_rad
        imu_msg.angular_velocity.y = gy_dps * deg_to_rad
        imu_msg.angular_velocity.z = gz_dps * deg_to_rad

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.orientation_covariance[0] = -1  # Not provided

        imu_pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
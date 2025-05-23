#!/usr/bin/env python3
import rospy
import serial
import struct
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from tf.transformations import quaternion_from_euler
import tf

# programmed by: duc

class HoverboardControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('hoverboard_control', anonymous=True)

        # Constants from config.h
        self.START_FRAME = 0xABCD
        self.ENCODER_MIN = 0
        self.ENCODER_MAX = 9000
        self.ENCODER_LOW_WRAP_FACTOR = 0.3
        self.ENCODER_HIGH_WRAP_FACTOR = 0.7
        self.TICKS_PER_ROTATION = 90

        # Parameters from controller.yaml and hardware.yaml
        self.wheel_separation = rospy.get_param('~hoverboard_velocity_controller/wheel_separation', 0.45)
        self.wheel_radius = rospy.get_param('~hoverboard_velocity_controller/wheel_radius', 0.0825)
        self.base_frame_id = rospy.get_param('~hoverboard_velocity_controller/base_frame_id', 'base_footprint')
        self.odom_frame_id = rospy.get_param('~hoverboard_velocity_controller/odom_frame_id', '/raw_odom')
        self.port = rospy.get_param('~port', '/dev/ttyTHS1')
        self.loop_hz = rospy.get_param('~robaka/hardware_interface/loop_hz', 50)
        self.direction_correction = rospy.get_param('~robaka/direction', 1)

        # Serial connection
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=0.1)
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port {}: {}".format(self.port, e))
            rospy.signal_shutdown("Serial port error")
            return

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.voltage_pub = rospy.Publisher('hoverboard/battery_voltage', Float64, queue_size=3)
        self.temp_pub = rospy.Publisher('hoverboard/temperature', Float64, queue_size=3)
        self.connected_pub = rospy.Publisher('hoverboard/connected', Bool, queue_size=3)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.get_time()
        self.last_read = rospy.get_time()

        # Encoder state
        self.last_wheelcountR = 0
        self.last_wheelcountL = 0
        self.multR = 0
        self.multL = 0
        self.low_wrap = self.ENCODER_LOW_WRAP_FACTOR * (self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN
        self.high_wrap = self.ENCODER_HIGH_WRAP_FACTOR * (self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN
        self.last_posL = 0.0
        self.last_posR = 0.0
        self.last_pub_posL = 0.0
        self.last_pub_posR = 0.0
        self.node_start_flag = True

        # Feedback storage
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # Initialize protocol-related attributes
        self.prev_byte = 0  # Initialize prev_byte
        self.msg_len = 0    # Initialize message length
        self.msg_buffer = []  # Initialize message buffer

    def cmd_vel_callback(self, msg):
        # Convert cmd_vel to wheel speeds (rad/s)
        linear_v = msg.linear.x
        angular_v = msg.angular.z
        left_speed = (linear_v - angular_v * self.wheel_separation / 2.0) / self.wheel_radius
        right_speed = (linear_v + angular_v * self.wheel_separation / 2.0) / self.wheel_radius

        # Convert rad/s to RPM (0.10472 = 2 * pi / 60)
        left_rpm = left_speed / 0.10472
        right_rpm = right_speed / 0.10472

        # Calculate steer and speed (mimicking hoverboard.cpp)
        speed = (left_rpm + right_rpm) / 2.0
        steer = (left_rpm - speed) * 2.0

        # Send commands
        self.send_speed_commands(steer, speed)

    def send_speed_commands(self, steer, speed):
        # Prepare SerialCommand
        command = struct.pack('<HhhH', self.START_FRAME, int(steer), int(speed), 0)
        checksum = self.START_FRAME ^ int(steer) ^ int(speed)
        command = struct.pack('<HhhH', self.START_FRAME, int(steer), int(speed), checksum)

        try:
            self.serial.write(command)
            rospy.logdebug("Sent: steer={}, speed={}".format(steer, speed))
        except serial.SerialException as e:
            rospy.logerr("Serial write error: {}".format(e))

    def read_feedback(self):
        # Read serial data
        try:
            while self.serial.in_waiting > 0:
                c = self.serial.read(1)
                if not c:
                    break
                self.protocol_recv(c)
        except serial.SerialException as e:
            rospy.logerr("Serial read error: {}".format(e))

        # Check connection timeout
        if (rospy.get_time() - self.last_read) > 1.0:
            rospy.logwarn("Timeout reading from serial")
            connected_msg = Bool()
            connected_msg.data = False
            self.connected_pub.publish(connected_msg)
        else:
            connected_msg = Bool()
            connected_msg.data = True
            self.connected_pub.publish(connected_msg)

    def protocol_recv(self, byte):
        # Convert byte to integer
        byte_val = ord(byte) if isinstance(byte, str) else byte[0]  # Ensure byte_val is an integer

        # Build start frame
        start_frame = (byte_val << 8) | self.prev_byte

        # Read start frame
        if start_frame == self.START_FRAME:
            self.msg_buffer = [self.prev_byte, byte_val]
            self.msg_len = 2
        elif self.msg_len >= 2 and self.msg_len < 20:  # sizeof(SerialFeedback) = 20 bytes
            self.msg_buffer.append(byte_val)
            self.msg_len += 1

        # Process complete message
        if self.msg_len == 20:
            try:
                # Unpack SerialFeedback
                msg = struct.unpack('<HhhhhhhhhhH', bytes(self.msg_buffer))
                start, cmd1, cmd2, speedR_meas, speedL_meas, wheelR_cnt, wheelL_cnt, batVoltage, boardTemp, cmdLed, checksum = msg

                # Verify checksum
                calc_checksum = start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ wheelR_cnt ^ wheelL_cnt ^ batVoltage ^ boardTemp ^ cmdLed
                if start == self.START_FRAME and checksum == calc_checksum:
                    self.last_read = rospy.get_time()

                    # Publish battery and temperature
                    self.voltage_pub.publish(Float64(batVoltage / 100.0))
                    self.temp_pub.publish(Float64(boardTemp / 10.0))

                    # Store velocities (RPM)
                    self.right_rpm = self.direction_correction * abs(speedR_meas)
                    self.left_rpm = self.direction_correction * abs(speedL_meas)

                    # Update odometry
                    self.on_encoder_update(wheelR_cnt, wheelL_cnt)
                else:
                    rospy.logwarn("Checksum mismatch: received={}, calculated={}".format(checksum, calc_checksum))
            except struct.error:
                rospy.logwarn("Failed to unpack SerialFeedback")
            self.msg_len = 0

        self.prev_byte = byte_val

    def on_encoder_update(self, right, left):
        # Calculate wheel position in ticks, factoring in encoder wraps
        if right < self.low_wrap and self.last_wheelcountR > self.high_wrap:
            self.multR += 1
        elif right > self.high_wrap and self.last_wheelcountR < self.low_wrap:
            self.multR -= 1
        posR = right + self.multR * (self.ENCODER_MAX - self.ENCODER_MIN)
        self.last_wheelcountR = right

        if left < self.low_wrap and self.last_wheelcountL > self.high_wrap:
            self.multL += 1
        elif left > self.high_wrap and self.last_wheelcountL < self.low_wrap:
            self.multL -= 1
        posL = left + self.multL * (self.ENCODER_MAX - self.ENCODER_MIN)
        self.last_wheelcountL = left

        # Handle board restarts
        if (rospy.get_time() - self.last_read) > 0.2 and abs(posL) < 5 and abs(posR) < 5:
            self.last_posL = posL
            self.last_posR = posR

        # Compute position differences
        posL_diff = 0.0
        posR_diff = 0.0
        if not self.node_start_flag:
            posL_diff = posL - self.last_posL
            posR_diff = posR - self.last_posR
        self.node_start_flag = False

        self.last_pub_posL += posL_diff
        self.last_pub_posR += posR_diff
        self.last_posL = posL
        self.last_posR = posR

        # Convert ticks to radians
        left_pos_rad = 2.0 * math.pi * self.last_pub_posL / self.TICKS_PER_ROTATION
        right_pos_rad = 2.0 * math.pi * self.last_pub_posR / self.TICKS_PER_ROTATION

        # Update odometry
        current_time = rospy.get_time()
        dt = current_time - self.last_time

        # Convert RPM to rad/s
        left_vel = self.left_rpm * 0.10472
        right_vel = self.right_rpm * 0.10472

        # Differential drive kinematics
        linear_v = (right_vel + left_vel) * self.wheel_radius / 2.0
        angular_v = (right_vel - left_vel) * self.wheel_radius / self.wheel_separation

        # Update pose
        delta_theta = angular_v * dt
        delta_x = linear_v * math.cos(self.theta) * dt
        delta_y = linear_v * math.sin(self.theta) * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_v

        # Covariance from controller.yaml
        odom.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

        # Publish TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )

        self.last_time = current_time

    def run(self):
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            self.read_feedback()
            rate.sleep()

        self.serial.close()

if __name__ == '__main__':
    try:
        node = HoverboardControl()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CmdVelRelay:
    def __init__(self):
        rospy.init_node('cmd_vel_relay', anonymous=True)

        # Subscribers and Publishers
        self.sub = rospy.Subscriber('/cmd_vel_web', Twist, self.cmd_vel_callback, queue_size=1)
        self.pub = rospy.Publisher('/hoverboard_velocity_controller/cmd_vel', Twist, queue_size=1)

        self.latest_cmd = Twist()  # Default zero velocities
        self.rate = rospy.Rate(20)  # 20 Hz publishing

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.latest_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelRelay()
        node.run()
    except rospy.ROSInterruptException:
        pass

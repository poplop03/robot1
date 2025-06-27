#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class CmdVelRelay:
    def __init__(self):
        rospy.init_node("cmd_vel_relay")

        self.enabled = False
        self.latest_cmd = Twist()

        rospy.Subscriber("/cmd_vel_web", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("/hoverboard_velocity_controller/cmd_vel", Twist, queue_size=1)

        # Services to control relay
        rospy.Service("/cmd_relay_enable", SetBool, self.handle_enable)

        self.rate = rospy.Rate(20)  # 20Hz

    def handle_enable(self, req):
        self.enabled = req.data
        msg = "Relay enabled" if req.data else "Relay disabled"
        rospy.loginfo(msg)
        return SetBoolResponse(success=True, message=msg)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.enabled:
                self.cmd_pub.publish(self.latest_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    CmdVelRelay().run()

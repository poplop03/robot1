#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class CmdVelRelay:
    def __init__(self):
        rospy.init_node('cmd_vel_relay', anonymous=True)

        self.sub = rospy.Subscriber('/cmd_vel_web', Twist, self.cmd_vel_callback, queue_size=1)
        self.pub = rospy.Publisher('/hoverboard_velocity_controller/cmd_vel', Twist, queue_size=1)

        self.latest_cmd = Twist()
        self.joystick_active = False     # Is joystick sending commands
        self.relay_enabled = False       # Controlled via service
        self.rate = rospy.Rate(20)

        # Add service
        self.relay_service = rospy.Service('/cmd_relay_enable', SetBool, self.set_relay_status)

    def set_relay_status(self, req):
        self.relay_enabled = req.data
        status = "enabled" if req.data else "disabled"
        rospy.loginfo(f"🔁 Relay manually {status}")
        return SetBoolResponse(success=True, message=f"Relay {status}")

    def cmd_vel_callback(self, msg):
        # If message is all zeros, consider joystick inactive
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            self.joystick_active = False
        else:
            self.joystick_active = True
            self.latest_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.relay_enabled and self.joystick_active:
                self.pub.publish(self.latest_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        CmdVelRelay().run()
    except rospy.ROSInterruptException:
        pass

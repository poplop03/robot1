import rospy
from geometry_msgs.msg import Twist

class CmdVelRelay:
    def __init__(self):
        rospy.init_node('cmd_vel_relay', anonymous=True)

        self.sub = rospy.Subscriber('/cmd_vel_web', Twist, self.cmd_vel_callback, queue_size=1)
        self.pub = rospy.Publisher('/hoverboard_velocity_controller/cmd_vel', Twist, queue_size=1)

        self.latest_cmd = Twist()
        self.should_publish = False
        self.rate = rospy.Rate(20)

    def cmd_vel_callback(self, msg):
        # Check if all components are zero
        if (msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0):
            self.should_publish = False
        else:
            self.should_publish = True
            self.latest_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.should_publish:
                self.pub.publish(self.latest_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelRelay()
        node.run()
    except rospy.ROSInterruptException:
        pass

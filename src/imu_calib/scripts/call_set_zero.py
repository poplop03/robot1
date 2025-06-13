#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def call_service():
    rospy.wait_for_service('/imu/set_zero_orientation')
    try:
        reset_orientation = rospy.ServiceProxy('/imu/set_zero_orientation', Empty)
        reset_orientation()
        rospy.loginfo("Service /imu/set_zero_orientation called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('call_set_zero_orientation_node', anonymous=True)
    rospy.sleep(1)  # Wait a few seconds to ensure the imu node is up
    call_service()

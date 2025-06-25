#!/usr/bin/env python3

import rospy
import tf
import os
import yaml
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class SavePoseSendGoalNode:
    def __init__(self):
        rospy.init_node('save_pose_send_goal')
        self.yaml_path = rospy.get_param('~yaml_path', '/tmp/poses.yaml')
        self.pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()

        # Services
        rospy.Service('/save_pose_send_goal/save_pose', String, self.save_pose_callback)
        rospy.Service('/save_pose_send_goal/send_goal', String, self.send_goal_callback)

        rospy.loginfo("✅ save_pose_send_goal node ready.")

    def get_current_pose(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return {'x': trans[0], 'y': trans[1], 'yaw': yaw}
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"❌ TF lookup failed: {e}")
            return None

    def save_pose_callback(self, req):
        name = req.data.strip()
        if not name:
            rospy.logwarn("⚠️ Cannot save pose with empty name.")
            return String(data="Pose name is empty")

        pose = self.get_current_pose()
        if not pose:
            return String(data="Failed to get current robot pose")

        # Load existing poses
        poses = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                poses = yaml.safe_load(f) or {}

        poses[name] = pose

        # Save to YAML
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        with open(self.yaml_path, 'w') as f:
            yaml.dump(poses, f)

        rospy.loginfo(f"✅ Saved pose '{name}' at ({pose['x']:.2f}, {pose['y']:.2f}, {math.degrees(pose['yaw']):.1f}°)")
        return String(data=f"Saved pose '{name}'")

    def send_goal_callback(self, req):
        name = req.data.strip()
        if not os.path.exists(self.yaml_path):
            rospy.logwarn("⚠️ No saved poses file found.")
            return String(data="No saved poses available")

        with open(self.yaml_path, 'r') as f:
            poses = yaml.safe_load(f) or {}

        if name not in poses:
            rospy.logwarn(f"⚠️ Pose '{name}' not found in saved poses.")
            return String(data=f"Pose '{name}' not found")

        pose = poses[name]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = pose['x']
        goal.pose.position.y = pose['y']
        goal.pose.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0, 0, pose['yaw'])
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.pose_pub.publish(goal)
        rospy.loginfo(f"🚀 Sent goal to '{name}'")
        return String(data=f"Sent goal to '{name}'")

if __name__ == '__main__':
    try:
        SavePoseSendGoalNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

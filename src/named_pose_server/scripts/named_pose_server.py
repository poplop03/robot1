#!/usr/bin/env python3

import rospy
import tf
import os
import yaml
import json
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class NamedPoseServer:
    def __init__(self):
        rospy.init_node("named_pose_server")

        self.pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.listener = tf.TransformListener()

        # Path to store poses
        self.yaml_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "../config/named_poses.yaml"
        ))

        self.saved_poses = {}
        self.load_from_yaml()

        # Services
        rospy.Service("/save_named_pose", String, self.save_named_pose_cb)
        rospy.Service("/get_saved_poses", Trigger, self.get_saved_poses_cb)
        rospy.Service("/send_named_goal", String, self.send_named_goal_cb)

        rospy.loginfo("✅ NamedPoseServer started. Listening for save/send requests.")

    def load_from_yaml(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, "r") as f:
                self.saved_poses = yaml.safe_load(f) or {}
            rospy.loginfo(f"Loaded {len(self.saved_poses)} named poses from YAML.")
        else:
            rospy.loginfo("No saved pose file found. Starting fresh.")
            self.saved_poses = {}

    def save_to_yaml(self):
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        with open(self.yaml_path, "w") as f:
            yaml.dump(self.saved_poses, f)
        rospy.loginfo(f"Saved poses to {self.yaml_path}")

    def get_current_pose(self):
        try:
            self.listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return {"x": trans[0], "y": trans[1], "yaw": yaw}
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"Could not get current robot pose: {e}")
            return None

    def save_named_pose_cb(self, req):
        name = req.data.strip()
        if not name:
            return self._string_response(False, "Pose name cannot be empty.")

        pose = self.get_current_pose()
        if pose:
            self.saved_poses[name] = pose
            self.save_to_yaml()
            rospy.loginfo(f"Saved pose '{name}': {pose}")
            return self._string_response(True, f"Saved pose '{name}'.")
        else:
            return self._string_response(False, "Failed to get current pose.")

    def get_saved_poses_cb(self, req):
        try:
            poses_json = json.dumps(self.saved_poses)
            return TriggerResponse(success=True, message=poses_json)
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def send_named_goal_cb(self, req):
        name = req.data.strip()
        if name not in self.saved_poses:
            return self._string_response(False, f"Pose '{name}' not found.")

        pose = self.saved_poses[name]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = pose["x"]
        goal.pose.position.y = pose["y"]
        goal.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, pose["yaw"])
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.pose_pub.publish(goal)
        rospy.loginfo(f"Published goal for '{name}' to /move_base_simple/goal")
        return self._string_response(True, f"Sent goal '{name}'.")

    def _string_response(self, success, message):
        res = String()
        res.data = message
        return res if success else res

if __name__ == "__main__":
    try:
        NamedPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

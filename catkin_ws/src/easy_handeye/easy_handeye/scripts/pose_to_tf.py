#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "camera_base")
        self.child_frame  = rospy.get_param("~child_frame",  "aruco_marker_frame")
        self.pose_topic   = rospy.get_param("~pose_topic",   "/aruco_single/pose")

        self.br = TransformBroadcaster()
        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb, queue_size=1)

    def cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("pose_to_tf")
    PoseToTF()
    rospy.spin()

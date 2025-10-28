#!/usr/bin/env python3
"""Broadcast a static transform for the world RGB-D camera."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply


def main():
    rospy.init_node("world_camera_tf_broadcaster", anonymous=False)

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "world_camera_optical_frame"

    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 1.0
    transform.transform.translation.z = 1.5

    # Pose specified in liver_sim.world (roll=0, pitch=0.9, yaw=-2.35619449).
    q_world_camera = quaternion_from_euler(0.0, 0.9, -2.35619449)
    # Convert to optical frame convention (+Z forward, +X right, +Y down).
    q_camera_optical = quaternion_from_euler(-1.57079632679, 0.0, -1.57079632679)
    qx, qy, qz, qw = quaternion_multiply(q_world_camera, q_camera_optical)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    broadcaster.sendTransform(transform)
    rospy.loginfo("Published static transform world -> world_camera_optical_frame")
    rospy.spin()


if __name__ == "__main__":
    main()

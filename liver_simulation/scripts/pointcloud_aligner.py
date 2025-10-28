#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2

class PointCloudAligner:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "probe_link_ee")
        self.world_topic = rospy.get_param("~world_topic", "/world_camera/depth/points")
        self.wrist_topic = rospy.get_param("~wrist_topic", "/wrist_camera/depth/points")
        self.world_aligned_topic = rospy.get_param("~world_aligned_topic", "world_camera/depth/aligned")
        self.wrist_aligned_topic = rospy.get_param("~wrist_aligned_topic", "wrist_camera/depth/aligned")

        # Keep a longer TF buffer so we can use the transform that matches the
        # point cloud timestamp (especially important when the probe is moving).
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)        
        self.world_pub = rospy.Publisher(self.world_aligned_topic, PointCloud2, queue_size=1)
        self.wrist_pub = rospy.Publisher(self.wrist_aligned_topic, PointCloud2, queue_size=1)

        rospy.Subscriber(self.world_topic, PointCloud2, self._handle_world, queue_size=1)
        rospy.Subscriber(self.wrist_topic, PointCloud2, self._handle_wrist, queue_size=1)

    def _handle_world(self, cloud):
        self._transform_and_publish(cloud, self.world_pub)

    def _handle_wrist(self, cloud):
        self._transform_and_publish(cloud, self.wrist_pub)

    def _transform_and_publish(self, cloud, publisher):
        if not cloud.header.frame_id:
            rospy.logwarn_throttle(2.0, "PointCloud2 without frame_id, skipping")
            return
        try:
            transform = self.buffer.lookup_transform(
                self.target_frame,
                cloud.header.frame_id,
                cloud.header.stamp,
                rospy.Duration(0.5),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                1.0,
                "PointCloudAligner waiting for transform %s -> %s (%s)",
                self.target_frame,
                cloud.header.frame_id,
                exc,
            )
            return

        aligned = tf2_sensor_msgs.do_transform_cloud(cloud, transform)
        publisher.publish(aligned)

if __name__ == "__main__":
    rospy.init_node("pointcloud_aligner")
    PointCloudAligner()
    rospy.spin()
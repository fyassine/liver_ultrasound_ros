#!/usr/bin/env python3
"""
Surface Generator Node
Generates a simulated rough abdominal surface as a PointCloud2 message.
This simulates the output of a 3D sensor.
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

class SurfaceGenerator:
    def __init__(self):
        rospy.init_node('surface_generator', anonymous=True)

        # Publisher for the point cloud
        self.cloud_pub = rospy.Publisher('/simulated_surface/points', PointCloud2, queue_size=1, latch=True)

        rospy.loginfo("Generating simulated abdomen surface...")
        self.generate_and_publish_surface()
        rospy.loginfo("Surface published to /simulated_surface/points")

    def generate_and_publish_surface(self):
        # Parameters for the surface
        width = 0.4  # meters (Y-axis)
        length = 0.3  # meters (X-axis)
        resolution = 0.01  # 1 cm between points
        noise_amplitude = 0.02  # 2 cm max height variation

        # Position of the surface in the world (on top of the patient bed)
        # Bed top is at z=0.6 in world frame. Phantom is slightly above.
        center_x, center_y, center_z = 1.0, 0.05, 0.62

        # Create a grid of points
        x_points = np.arange(-length / 2, length / 2, resolution)
        y_points = np.arange(-width / 2, width / 2, resolution)
        xx, yy = np.meshgrid(x_points, y_points)

        # Create a bumpy surface using sinusoidal functions
        zz = noise_amplitude * (np.sin(xx * 20) * np.cos(yy * 15))

        # Combine into a single array of points
        points = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

        # Translate points to their world position
        points[:, 0] += center_x
        points[:, 1] += center_y
        points[:, 2] += center_z

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'  # Use the world frame

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # Create point cloud data
        import struct
        point_step = 12  # 3 floats * 4 bytes/float
        cloud_data = []
        for p in points:
            cloud_data.append(struct.pack('fff', p[0], p[1], p[2]))
        
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=point_step,
            row_step=point_step * len(points),
            data=b"".join(cloud_data)
        )

        self.cloud_pub.publish(cloud_msg)

if __name__ == '__main__':
    try:
        SurfaceGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

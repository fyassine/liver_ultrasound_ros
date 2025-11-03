#!/usr/bin/python3
"""
Ultrasound Scan Simulator
Publishes a static ultrasound image continuously (no contact detection).
"""

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ANSI color codes for terminal output
GREEN = '\033[92m'
RESET = '\033[0m'

def log_success(msg):
    """Log success messages in green"""
    rospy.loginfo(f"{GREEN}{msg}{RESET}")

class ScanSimulator:
    def __init__(self):
        rospy.init_node('scan_simulator', anonymous=False)
        
        # Get robot namespace
        robot_name = rospy.get_param('~robot_name', 'iiwa')
        
        # Load ultrasound image
        pkg_path = rospy.get_param('~package_path', None)
        if pkg_path is None:
            # Try to find the package path
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('liver_simulation')
        
        image_path = os.path.join(pkg_path, 'images', 'ultrasound.png')
        
        if not os.path.exists(image_path):
            rospy.logerr(f"Ultrasound image not found at: {image_path}")
            rospy.signal_shutdown("Missing ultrasound image")
            return
        
        self.ultrasound_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if self.ultrasound_img is None:
            rospy.logerr(f"Failed to load ultrasound image from: {image_path}")
            rospy.signal_shutdown("Failed to load ultrasound image")
            return
        
        rospy.loginfo(f"Loaded ultrasound image: {image_path} ({self.ultrasound_img.shape})")
        
        # CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Publisher for ultrasound image
        self.image_pub = rospy.Publisher(
            '/ultrasound/image_raw',
            Image,
            queue_size=10
        )
        
        # Publish rate
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)  # Hz
        
        # Timer for publishing at fixed rate
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self._publish_timer_callback
        )
        
        log_success("Scan simulator initialized. Publishing ultrasound images...")
    
    def _publish_timer_callback(self, event):
        """Publish ultrasound image at fixed rate."""
        # Convert OpenCV image to ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self.ultrasound_img, encoding="mono8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "probe_link_ee"
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Failed to publish ultrasound image: {e}")

if __name__ == '__main__':
    try:
        simulator = ScanSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

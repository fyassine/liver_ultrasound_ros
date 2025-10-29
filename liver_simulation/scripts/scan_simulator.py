#!/usr/bin/env python3
"""
Ultrasound Scan Simulator
Publishes a static ultrasound image when the probe is in contact with the phantom.
"""

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ContactsState
from cv_bridge import CvBridge
from collections import deque

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
        
        # Track contact state - simple immediate detection
        self.in_contact = False
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)  # Hz
        
        # Subscribe to force/torque sensor contact topic
        wrench_topic = f"/{robot_name}/ft_sensor/wrench"
        rospy.loginfo(f"Subscribing to contact topic: {wrench_topic}")
        rospy.loginfo(f"Using immediate contact detection (force > 0.1 N)")
        
        self.wrench_sub = rospy.Subscriber(
            wrench_topic,
            ContactsState,
            self._contact_callback
        )
        
        # Timer for publishing at fixed rate
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self._publish_timer_callback
        )
        
        log_success("Scan simulator initialized. Waiting for contact...")
    
    def _contact_callback(self, msg):
        """Detect contact based on ContactsState from Gazebo F/T sensor - simple immediate detection."""
        
        # Calculate current force magnitude
        has_contact_states = len(msg.states) > 0
        if not has_contact_states or not msg.states[0].total_wrench:
            current_force = 0.0
        else:
            total_wrench = msg.states[0].total_wrench
            force = total_wrench.force
            current_force = (force.x**2 + force.y**2 + force.z**2)**0.5
        
        # Simple threshold: any force above minimum threshold = contact
        min_force_threshold = 0.1  # Very low threshold to detect any contact
        
        # Update contact state immediately based on current force
        new_contact_state = current_force > min_force_threshold
        
        # Log state changes
        if new_contact_state and not self.in_contact:
            log_success(f"Contact detected! Force: {current_force:.2f} N")
            self.in_contact = True
        elif not new_contact_state and self.in_contact:
            rospy.loginfo(f"Contact lost! Force: {current_force:.2f} N")
            self.in_contact = False
    
    def _publish_timer_callback(self, event):
        """Publish ultrasound image at fixed rate when in contact."""
        if not self.in_contact:
            return
        
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

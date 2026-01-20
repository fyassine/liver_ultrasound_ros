#!/usr/bin/env python3
"""
Ultrasound image processor - crops, rotates, and calculates pixel size.
Removes UI elements (top/bottom bars) and corrects orientation.
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class UltrasoundProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.crop_top = rospy.get_param('~crop_top', 325)
        self.crop_bottom = rospy.get_param('~crop_bottom', 595)
        self.crop_left = rospy.get_param('~crop_left', 0)
        self.crop_right = rospy.get_param('~crop_right', 0)
        
        self.rotation = rospy.get_param('~rotation', 270)
        
        self.depth_cm = rospy.get_param('~depth_cm', 16.0)
        
        self.image_width = rospy.get_param('~image_width', 1920)
        self.image_height = rospy.get_param('~image_height', 1080)
        
        if self.rotation in [90, 270]:
            # After 90/270 rotation: width and height swap
            self.rotated_width = self.image_height   # 1080
            self.rotated_height = self.image_width   # 1920
        else:
            self.rotated_width = self.image_width
            self.rotated_height = self.image_height
            
        # Crop is applied after rotation (top/bottom crop the tall axis)
        self.cropped_width = self.rotated_width - self.crop_left - self.crop_right
        self.cropped_height = self.rotated_height - self.crop_top - self.crop_bottom
        
        if self.cropped_height > 0:
            self.pixel_size_cm = self.depth_cm / self.cropped_height
        else:
            self.pixel_size_cm = 0.0
            
        rospy.set_param('~pixel_size_cm', self.pixel_size_cm)
        rospy.set_param('~cropped_height_pixels', self.cropped_height)
        
        input_topic = rospy.get_param('~input_topic', '/ultrasound_camera/image_raw')
        output_topic = rospy.get_param('~output_topic', '/ultrasound/image_raw')
        
        self.pub = rospy.Publisher(output_topic, Image, queue_size=2)
        self.sub = rospy.Subscriber(input_topic, Image, self.callback, queue_size=2)
        
        rospy.loginfo("="*50)
        rospy.loginfo("Ultrasound Processor Configuration:")
        rospy.loginfo(f"  Original: {self.image_width}x{self.image_height}")
        rospy.loginfo(f"  After rotation ({self.rotation}°): {self.rotated_width}x{self.rotated_height}")
        rospy.loginfo(f"  After crop: {self.cropped_width}x{self.cropped_height}")
        rospy.loginfo(f"  Depth setting: {self.depth_cm} cm")
        rospy.loginfo(f"  Pixel size: {self.pixel_size_cm:.6f} cm/pixel")
        rospy.loginfo("="*50)
        
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.rotation == 90:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation == 180:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            elif self.rotation == 270:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            h, w = cv_image.shape[:2]
            y1 = self.crop_top
            y2 = h - self.crop_bottom
            x1 = self.crop_left
            x2 = w - self.crop_right
            
            if y2 > y1 and x2 > x1:
                cv_image = cv_image[y1:y2, x1:x2]
            
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


def main():
    rospy.init_node('ultrasound_processor')
    processor = UltrasoundProcessor()
    rospy.spin()


if __name__ == '__main__':
    main()


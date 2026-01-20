#!/usr/bin/env python3
"""
Ultrasound image processor - crops and rotates ultrasound image.
Removes UI elements (top/bottom bars) and corrects orientation.
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class UltrasoundProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.crop_top = rospy.get_param('~crop_top', 500)
        self.crop_bottom = rospy.get_param('~crop_bottom', 200)
        self.crop_left = rospy.get_param('~crop_left', 0)
        self.crop_right = rospy.get_param('~crop_right', 0)
        
        self.rotation = rospy.get_param('~rotation', 90)
        
        input_topic = rospy.get_param('~input_topic', '/ultrasound_camera/image_raw')
        output_topic = rospy.get_param('~output_topic', '/ultrasound/image_raw')
        
        self.pub = rospy.Publisher(output_topic, Image, queue_size=2)
        self.sub = rospy.Subscriber(input_topic, Image, self.callback, queue_size=2)
        
        rospy.loginfo(f"Ultrasound processor: {input_topic} -> {output_topic}")
        rospy.loginfo(f"Crop: top={self.crop_top}, bottom={self.crop_bottom}")
        rospy.loginfo(f"Rotation: {self.rotation}°")
        
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

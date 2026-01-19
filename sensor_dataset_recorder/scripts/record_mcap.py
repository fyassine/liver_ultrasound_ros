#!/usr/bin/env python3
"""
MCAP recorder for robot data with TF support.
Records IIWA robot joint states and TF transforms to MCAP format for Foxglove visualization.
"""

import os
import sys
import time
import signal
import argparse
from datetime import datetime

import rospy
from mcap_ros1.writer import Writer as McapWriter
from iiwa_msgs.msg import JointPosition, JointTorque
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


class McapRecorder:
    """Records ROS topics to MCAP format."""
    
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.writer = None
        self.mcap_file = None
        self.recording = False
        self.message_count = 0
        
        self.topics = {
            '/iiwa/state/JointPosition': JointPosition,
            '/iiwa/state/JointTorque': JointTorque,
            '/iiwa/joint_states': JointState,
            '/joint_states': JointState,
            '/tf': TFMessage,
            '/tf_static': TFMessage,
        }
        
        self.subscribers = []
        
    def start_recording(self, episode_name: str = None):
        """Start recording to a new MCAP file."""
        if self.recording:
            rospy.logwarn("Already recording!")
            return
            
        if episode_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            episode_name = f"episode_{timestamp}"
            
        filepath = os.path.join(self.output_dir, f"{episode_name}.mcap")
        
        self.mcap_file = open(filepath, 'wb')
        self.writer = McapWriter(self.mcap_file)
        
        for topic, msg_type in self.topics.items():
            sub = rospy.Subscriber(
                topic, 
                msg_type, 
                self._message_callback,
                callback_args=topic
            )
            self.subscribers.append(sub)
            
        self.recording = True
        self.message_count = 0
        rospy.loginfo(f"Started recording to: {filepath}")
        
    def _message_callback(self, msg, topic):
        """Write incoming message to MCAP."""
        if not self.recording or self.writer is None:
            return
            
        try:
            self.writer.write_message(topic, msg)
            self.message_count += 1
            
            if self.message_count % 100 == 0:
                rospy.loginfo(f"Recorded {self.message_count} messages...")
        except Exception as e:
            rospy.logerr(f"Error writing message: {e}")
            
    def stop_recording(self):
        """Stop recording and close file."""
        if not self.recording:
            return
            
        self.recording = False
        
        for sub in self.subscribers:
            sub.unregister()
        self.subscribers.clear()
        
        if self.writer:
            self.writer.finish()
            self.writer = None
            
        if self.mcap_file:
            self.mcap_file.close()
            self.mcap_file = None
            
        rospy.loginfo(f"Recording stopped. Total messages: {self.message_count}")


def main():
    parser = argparse.ArgumentParser(description='Record robot joints to MCAP')
    parser.add_argument('--output-dir', '-o', 
                        default=os.path.expanduser('~/datasets/liver_ultrasound'),
                        help='Output directory for recordings')
    parser.add_argument('--episode', '-e',
                        default=None,
                        help='Episode name (default: auto-generated timestamp)')
    parser.add_argument('--duration', '-d',
                        type=float, default=None,
                        help='Recording duration in seconds (default: until Ctrl+C)')
    
    args, _ = parser.parse_known_args()
    
    rospy.init_node('mcap_recorder', anonymous=True)
    
    recorder = McapRecorder(args.output_dir)
    
    def signal_handler(sig, frame):
        rospy.loginfo("Stopping recording...")
        recorder.stop_recording()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    recorder.start_recording(args.episode)
    
    if args.duration:
        rospy.loginfo(f"Recording for {args.duration} seconds...")
        time.sleep(args.duration)
        recorder.stop_recording()
    else:
        rospy.loginfo("Recording until Ctrl+C...")
        rospy.spin()
        

if __name__ == '__main__':
    main()

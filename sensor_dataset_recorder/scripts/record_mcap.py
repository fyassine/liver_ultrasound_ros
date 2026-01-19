#!/usr/bin/env python3
"""
MCAP recorder for robot data with TF and camera support.
Records IIWA robot, TF transforms, and camera to MCAP for Foxglove visualization.
"""

import os
import sys
import time
import signal
import argparse
import threading
from datetime import datetime

import rospy
from mcap_ros1.writer import Writer as McapWriter
from iiwa_msgs.msg import JointPosition, JointTorque
from sensor_msgs.msg import JointState, Image
from tf2_msgs.msg import TFMessage


class McapRecorder:
    """Records ROS topics to MCAP format."""
    
    def __init__(self, output_dir: str, recording_fps: float = 30.0):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.writer = None
        self.mcap_file = None
        self.recording = False
        self.message_count = 0
        self.lock = threading.Lock()
        
        self.recording_fps = recording_fps
        self.min_interval = 1.0 / recording_fps
        self.last_write_time = {}
        
        self.topics = {
            '/iiwa/state/JointPosition': JointPosition,
            '/iiwa/state/JointTorque': JointTorque,
            '/joint_states': JointState,

            '/tf': TFMessage,
            '/tf_static': TFMessage,

            '/rgb/image_raw': Image,
            '/depth_to_rgb/image_raw': Image,
        }
        
        self.subscribers = []
        
    def start_recording(self, episode_name: str = None):
        """Start recording to a new MCAP file."""
        with self.lock:
            if self.recording:
                rospy.logwarn("Already recording!")
                return
            
            if episode_name is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                episode_name = f"episode_{timestamp}"
                
            filepath = os.path.join(self.output_dir, f"{episode_name}.mcap")
            
            self.mcap_file = open(filepath, 'wb')
            self.writer = McapWriter(self.mcap_file)
            self.writer.__enter__()
            
            self.recording = True
            self.message_count = 0
            
        rospy.loginfo(f"Started recording to: {filepath}")
        
        # Subscribe to topics
        for topic, msg_type in self.topics.items():
            queue_size = 2 if 'image' in topic else 10
            sub = rospy.Subscriber(
                topic, msg_type, self._message_callback,
                callback_args=topic, queue_size=queue_size
            )
            self.subscribers.append(sub)
        
    def _message_callback(self, msg, topic):
        """Write incoming message to MCAP."""
        now = rospy.get_time()
        if now - self.last_write_time.get(topic, 0) < self.min_interval:
            return
        self.last_write_time[topic] = now

        with self.lock:
            if not self.recording or self.writer is None:
                return
            try:
                current_time = rospy.Time.now().to_nsec()
                publish_time = msg.header.stamp.to_nsec() if hasattr(msg, 'header') else current_time
                self.writer.write_message(topic, msg, log_time=current_time, publish_time=publish_time)
                self.message_count += 1
                if self.message_count % 100 == 0:
                    rospy.loginfo(f"Recorded {self.message_count} messages...")
            except Exception as e:
                rospy.logerr(f"Error writing {topic}: {e}")
            
    def stop_recording(self):
        """Stop recording and close file."""
        with self.lock:
            if not self.recording:
                return
            self.recording = False
            
            for sub in self.subscribers:
                sub.unregister()
            self.subscribers.clear()
            
            if self.writer:
                try:
                    self.writer.__exit__(None, None, None)
                except Exception as e:
                    rospy.logerr(f"Error closing writer: {e}")
                self.writer = None
                
            if self.mcap_file:
                self.mcap_file.close()
                self.mcap_file = None
            
        rospy.loginfo(f"Recording stopped. Total messages: {self.message_count}")


def main():
    parser = argparse.ArgumentParser(description='Record robot and camera data to MCAP')
    parser.add_argument('--output-dir', '-o', 
                        default=os.path.expanduser('~/datasets/liver_ultrasound'))
    parser.add_argument('--episode', '-e', default=None)
    parser.add_argument('--duration', '-d', type=float, default=None)
    
    args, _ = parser.parse_known_args()
    rospy.init_node('mcap_recorder', anonymous=True)
    
    recording_fps = rospy.get_param('~recording_fps', 30.0)
    duration = rospy.get_param('~duration', args.duration)
    
    recorder = McapRecorder(args.output_dir, recording_fps)
    
    def signal_handler(sig, frame):
        rospy.loginfo("Stopping recording...")
        recorder.stop_recording()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    recorder.start_recording(args.episode)
    
    if duration:
        rospy.loginfo(f"Recording for {duration} seconds...")
        time.sleep(duration)
        recorder.stop_recording()
    else:
        rospy.loginfo("Recording until Ctrl+C...")
        rospy.spin()


if __name__ == '__main__':
    main()

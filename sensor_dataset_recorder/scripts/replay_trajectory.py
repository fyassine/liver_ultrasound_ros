#!/usr/bin/env python3
"""
Trajectory replay from MCAP files.
Reads recorded joint positions and publishes them as commands.
"""

import os
import sys
import rospy
from mcap.reader import make_reader
from mcap_ros1.decoder import DecoderFactory
from iiwa_msgs.msg import JointPosition


def replay_trajectory(mcap_file, rate_hz=30.0, speed=1.0):
    """Replay trajectory from MCAP file."""
    
    mcap_path = os.path.expanduser(mcap_file)
    
    if not os.path.exists(mcap_path):
        rospy.logerr(f"MCAP file not found: {mcap_path}")
        return False
    
    rospy.loginfo(f"Loading trajectory from: {mcap_path}")
    
    joint_positions = []
    timestamps = []
    
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        
        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            if channel.topic == '/iiwa/state/JointPosition':
                joint_positions.append(ros_msg)
                timestamps.append(message.log_time / 1e9)
    
    if not joint_positions:
        rospy.logerr("No joint positions found in MCAP file!")
        return False
    
    duration = timestamps[-1] - timestamps[0]
    rospy.loginfo(f"Loaded {len(joint_positions)} positions, duration: {duration:.2f}s")
    
    cmd_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    rospy.sleep(0.5)  # Wait for publisher to connect
    
    rospy.loginfo(f"Starting replay at {rate_hz} Hz, speed {speed}x...")
    
    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()
    trajectory_start = timestamps[0]
    trajectory_end = timestamps[-1]
    idx = 0
    last_logged_idx = -100  # For progress logging
    
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec() * speed
        target_time = trajectory_start + elapsed
        
        if target_time >= trajectory_end:
            cmd_pub.publish(joint_positions[-1])
            rospy.loginfo("Replay: 100.0%")
            break
        
        while idx < len(timestamps) - 1 and timestamps[idx + 1] <= target_time:
            idx += 1
        
        cmd_pub.publish(joint_positions[idx])
        
        if idx >= last_logged_idx + 100:
            progress = (idx / len(joint_positions)) * 100
            rospy.loginfo(f"Replay: {progress:.1f}%")
            last_logged_idx = idx
        
        rate.sleep()
    
    rospy.loginfo("Trajectory replay complete!")
    return True


def main():
    rospy.init_node('trajectory_replayer')
    
    mcap_file = rospy.get_param('~mcap_file', '')
    rate_hz = rospy.get_param('~rate', 30.0)
    speed = rospy.get_param('~speed', 1.0)
    
    if not mcap_file:
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('mcap_file', nargs='?', default='')
        parser.add_argument('--rate', '-r', type=float, default=30.0)
        parser.add_argument('--speed', '-s', type=float, default=1.0)
        args, _ = parser.parse_known_args()
        mcap_file = args.mcap_file
        rate_hz = args.rate
        speed = args.speed
    
    if not mcap_file:
        rospy.logerr("No MCAP file specified! Use mcap_file param or command line arg.")
        sys.exit(1)
    
    success = replay_trajectory(mcap_file, rate_hz, speed)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

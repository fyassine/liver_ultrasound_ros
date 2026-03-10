#!/usr/bin/env python3

import os
import sys
import subprocess

def replay_trajectory(bag_file):
    bags_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'bags'
    )
    bag_path = os.path.join(bags_dir, bag_file)
    
    if not os.path.exists(bag_path):
        print(f"Bag file not found: {bag_path}")
        sys.exit(1)

    print(f"Replaying trajectory from: {bag_path}")
    print("Remapping /iiwa/state/* topics to /iiwa/command/* topics...")
    
    try:
        remap_args = [
            '/iiwa/state/CartesianPose:=/iiwa/command/CartesianPose',
            '/iiwa/state/CartesianWrench:=/iiwa/command/CartesianWrench',
            '/iiwa/state/JointPosition:=/iiwa/command/JointPosition',
            '/iiwa/state/JointVelocity:=/iiwa/command/JointVelocity',
            '/iiwa/state/JointTorque:=/iiwa/command/JointTorque'
        ]
        
        cmd = ['rosbag', 'play', bag_path] + remap_args
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nReplay stopped.")
    except subprocess.CalledProcessError as e:
        print(f"Error during replay: {e}")
        sys.exit(1)

if __name__ == '__main__':
    replay_trajectory("recorded_trajectory_iiwa.bag")


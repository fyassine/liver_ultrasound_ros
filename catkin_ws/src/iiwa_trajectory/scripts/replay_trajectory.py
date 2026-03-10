#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

from storage_paths import default_bag_path, resolve_bag_path

def replay_trajectory(bag_file):
    bag_path = resolve_bag_path(bag_file)
    
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
    parser = argparse.ArgumentParser(description='Replay a recorded iiwa trajectory bag.')
    parser.add_argument('--bag', '-b', default=default_bag_path('recorded_trajectory_iiwa.bag'), help='Bag file path or filename inside the project data folder')
    args, _ = parser.parse_known_args()
    replay_trajectory(args.bag)


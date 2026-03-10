#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import time

from storage_paths import default_bag_path, ensure_recordings_dir


def record_joint_positions(iiwa_output_path=None, standard_output_path=None):
    republisher_script = os.path.join(os.path.dirname(__file__), 'topics_republisher.py')
    republisher_process = subprocess.Popen(['python3', republisher_script])

    time.sleep(2)
    try:
        iiwa_topics = ['/iiwa/state/JointPosition']
        standard_topics = ['/iiwa/state/JointPosition_standard']

        ensure_recordings_dir()
        print(f"Recording joint positions. Press Ctrl+C to stop recording...")

        iiwa_output_path = iiwa_output_path or default_bag_path('recorded_joint_positions_iiwa.bag')
        standard_output_path = standard_output_path or default_bag_path('recorded_joint_positions_standard.bag')

        iiwa_cmd = ['rosbag', 'record', '-O', iiwa_output_path] + iiwa_topics
        standard_cmd = ['rosbag', 'record', '-O', standard_output_path] + standard_topics

        iiwa_process = subprocess.Popen(iiwa_cmd)
        standard_process = subprocess.Popen(standard_cmd)

        iiwa_process.wait()
        standard_process.wait()
    except KeyboardInterrupt:
        print("\nRecording stopped.")
    finally:
        republisher_process.terminate()
        if 'iiwa_process' in locals():
            iiwa_process.terminate()
        if 'standard_process' in locals():
            standard_process.terminate()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record iiwa joint-position bags.')
    parser.add_argument('--iiwa-bag', default=None, help='Output path for raw iiwa joint bag')
    parser.add_argument('--standard-bag', default=None, help='Output path for standard joint bag')
    args, _ = parser.parse_known_args()
    record_joint_positions(args.iiwa_bag, args.standard_bag)

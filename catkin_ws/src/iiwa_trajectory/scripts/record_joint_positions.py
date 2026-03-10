#!/usr/bin/env python3

import os
import subprocess
import sys
import time

def record_joint_positions():
    republisher_script = os.path.join(os.path.dirname(__file__), 'topics_republisher.py')
    republisher_process = subprocess.Popen(['python3', republisher_script])

    time.sleep(2)
    try:
        iiwa_topics = ['/iiwa/state/JointPosition']
        standard_topics = ['/iiwa/state/JointPosition_standard']

        bags_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'bags'
        )
        os.makedirs(bags_dir, exist_ok=True)
        print(f"Recording joint positions. Press Ctrl+C to stop recording...")

        iiwa_output_path = os.path.join(bags_dir, 'recorded_joint_positions_iiwa.bag')
        standard_output_path = os.path.join(bags_dir, 'recorded_joint_positions_standard.bag')

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
    record_joint_positions()

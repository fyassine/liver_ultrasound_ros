#!/usr/bin/env python3

import os
import subprocess
import sys

def record_trajectory():


    republisher_script = os.path.join(os.path.dirname(__file__), 'topics_republisher.py')
    republisher_process = subprocess.Popen(['python3', republisher_script])

    import time
    time.sleep(2)
    try:
        iiwa_topics = [
            '/iiwa/state/CartesianPose',
            '/iiwa/state/CartesianWrench',
            '/iiwa/state/JointPosition',
            '/iiwa/state/JointVelocity',
            '/iiwa/state/JointTorque'
        ]
        standard_topics = [
            '/iiwa/state/CartesianPose_standard',
            '/iiwa/state/CartesianWrench_standard',
            '/iiwa/state/JointPosition_standard',
            '/iiwa/state/JointVelocity_standard',
            '/iiwa/state/JointTorque_standard'
        ]

        bags_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'bags'
        )
        os.makedirs(bags_dir, exist_ok=True)
        print(f"Recording. Press Ctrl+C to stop recording...")

        iiwa_output_path = os.path.join(bags_dir, 'recorded_trajectory_iiwa.bag')
        standard_output_path = os.path.join(bags_dir, 'recorded_trajectory_standard.bag')

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
    record_trajectory()

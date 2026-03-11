#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rospy
from iiwa_msgs.msg import JointPosition
from mcap_ros1.writer import Writer as McapWriter
from sensor_msgs.msg import Image, JointState


REPOSITORY_ROOT = os.path.normpath(
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            '..',
            '..',
            '..',
            '..',
        )
    )
)

DEFAULT_OUTPUT_DIR = os.path.join(REPOSITORY_ROOT, 'data', 'mcap_recordings')


def resolve_output_dir(path_value=None):
    if not path_value:
        return DEFAULT_OUTPUT_DIR
    expanded = os.path.expandvars(os.path.expanduser(path_value))
    if os.path.isabs(expanded):
        return os.path.normpath(expanded)
    return os.path.normpath(os.path.join(REPOSITORY_ROOT, expanded))


def build_output_path(output_dir=None, filename_prefix='dataset_recording', filename=None):
    resolved_output_dir = resolve_output_dir(output_dir)
    os.makedirs(resolved_output_dir, exist_ok=True)
    if filename:
        resolved_name = filename
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        resolved_name = f'{filename_prefix}_{timestamp}.mcap'
    if not resolved_name.endswith('.mcap'):
        resolved_name = f'{resolved_name}.mcap'
    return os.path.join(resolved_output_dir, resolved_name)


class McapRecorder:
    def __init__(self, output_path, recording_fps=0.0, spawn_joint_republisher=True):
        self.output_path = output_path
        self.recording_interval = 0.0 if recording_fps <= 0.0 else 1.0 / float(recording_fps)
        self.spawn_joint_republisher = spawn_joint_republisher
        self.writer = None
        self.mcap_file = None
        self.is_recording = False
        self.message_count = 0
        self.lock = threading.Lock()
        self.last_write_time = {}
        self.subscribers = []
        self.republisher_process = None
        self.topics = {
            '/base_camera/rgb/image_raw': Image,
            '/base_camera/depth_to_rgb/image_raw': Image,
            '/hand_camera/rgb/image_raw': Image,
            '/hand_camera/depth_to_rgb/image_raw': Image,
            '/clarius/bmode': Image,
            '/iiwa/state/JointPosition': JointPosition,
            '/iiwa/state/JointPosition_standard': JointState,
        }

    def start(self):
        if self.spawn_joint_republisher:
            republisher_script = os.path.join(
                os.path.dirname(__file__),
                '..',
                '..',
                'iiwa_trajectory',
                'scripts',
                'topics_republisher.py',
            )
            republisher_script = os.path.normpath(republisher_script)
            self.republisher_process = subprocess.Popen(['python3', republisher_script])
            time.sleep(2)

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.mcap_file = open(self.output_path, 'wb')
        self.writer = McapWriter(self.mcap_file)
        self.writer.__enter__()
        self.is_recording = True

        for topic, msg_type in self.topics.items():
            queue_size = 2 if msg_type is Image else 10
            subscriber = rospy.Subscriber(
                topic,
                msg_type,
                self._message_callback,
                callback_args=topic,
                queue_size=queue_size,
            )
            self.subscribers.append(subscriber)

        rospy.loginfo('Recording MCAP dataset to %s', self.output_path)

    def _message_callback(self, msg, topic):
        if self.recording_interval > 0.0:
            now = rospy.get_time()
            if now - self.last_write_time.get(topic, 0.0) < self.recording_interval:
                return
            self.last_write_time[topic] = now

        with self.lock:
            if not self.is_recording or self.writer is None:
                return
            current_time = rospy.Time.now().to_nsec()
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                publish_time = msg.header.stamp.to_nsec()
                if publish_time == 0:
                    publish_time = current_time
            else:
                publish_time = current_time
            try:
                self.writer.write_message(topic, msg, log_time=current_time, publish_time=publish_time)
                self.message_count += 1
                if self.message_count % 100 == 0:
                    rospy.loginfo('Recorded %d messages', self.message_count)
            except Exception as error:
                rospy.logerr('Failed to write %s: %s', topic, str(error))

    def stop(self):
        with self.lock:
            if not self.is_recording:
                return
            self.is_recording = False

            for subscriber in self.subscribers:
                subscriber.unregister()
            self.subscribers = []

            if self.writer is not None:
                self.writer.__exit__(None, None, None)
                self.writer = None

            if self.mcap_file is not None:
                self.mcap_file.close()
                self.mcap_file = None

        if self.republisher_process is not None:
            self.republisher_process.terminate()
            try:
                self.republisher_process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                self.republisher_process.kill()
            self.republisher_process = None

        rospy.loginfo('Stopped MCAP recording after %d messages', self.message_count)


def main():
    parser = argparse.ArgumentParser(description='Record camera, ultrasound, and joint data to MCAP.')
    parser.add_argument('--output-dir', default=None)
    parser.add_argument('--filename-prefix', default=None)
    parser.add_argument('--filename', default=None)
    parser.add_argument('--recording-fps', type=float, default=None)
    parser.add_argument('--spawn-joint-republisher', dest='spawn_joint_republisher', action='store_true')
    parser.add_argument('--no-spawn-joint-republisher', dest='spawn_joint_republisher', action='store_false')
    parser.set_defaults(spawn_joint_republisher=None)
    args, _ = parser.parse_known_args()

    rospy.init_node('mcap_dataset_recorder')

    output_dir = rospy.get_param('~output_dir', args.output_dir)
    filename_prefix = rospy.get_param('~filename_prefix', args.filename_prefix or 'mcap_dataset')
    filename = rospy.get_param('~filename', args.filename)
    recording_fps = float(rospy.get_param('~recording_fps', args.recording_fps if args.recording_fps is not None else 0.0))
    spawn_joint_republisher = rospy.get_param(
        '~spawn_joint_republisher',
        True if args.spawn_joint_republisher is None else args.spawn_joint_republisher,
    )

    output_path = build_output_path(output_dir=output_dir, filename_prefix=filename_prefix, filename=filename)
    recorder = McapRecorder(
        output_path=output_path,
        recording_fps=recording_fps,
        spawn_joint_republisher=bool(spawn_joint_republisher),
    )

    def shutdown_handler(*_args):
        recorder.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    rospy.on_shutdown(recorder.stop)

    try:
        recorder.start()
        rospy.spin()
    except FileNotFoundError as error:
        rospy.logerr('Missing recorder dependency: %s', str(error))
        recorder.stop()
        sys.exit(1)
    except Exception as error:
        rospy.logerr('Recorder failed: %s', str(error))
        recorder.stop()
        sys.exit(1)


if __name__ == '__main__':
    main()
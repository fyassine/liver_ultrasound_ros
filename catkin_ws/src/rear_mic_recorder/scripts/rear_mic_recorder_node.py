#!/usr/bin/env python3

import os
import signal
import subprocess
from datetime import datetime

import rospy


class RearMicRecorder:
    def __init__(self):
        self.device = rospy.get_param("~device", "default")
        self.sample_rate = int(rospy.get_param("~sample_rate", 44100))
        self.channels = int(rospy.get_param("~channels", 2))
        self.sample_format = rospy.get_param("~sample_format", "S16_LE")
        self.duration = int(rospy.get_param("~duration", 0))
        self.output_dir = rospy.get_param("~output_dir", "data/mic_recordings")
        self.filename_prefix = rospy.get_param("~filename_prefix", "rear_mic")
        self.filename = rospy.get_param("~filename", "")
        self.process = None

    def _find_workspace_root(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        while True:
            if os.path.isdir(os.path.join(current_dir, "catkin_ws")) and os.path.isdir(os.path.join(current_dir, "data")):
                return current_dir
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:
                return os.getcwd()
            current_dir = parent_dir

    def _resolve_output_dir(self):
        expanded_output_dir = os.path.expanduser(os.path.expandvars(self.output_dir))
        if os.path.isabs(expanded_output_dir):
            return expanded_output_dir
        return os.path.normpath(os.path.join(self._find_workspace_root(), expanded_output_dir))

    def _build_output_path(self):
        output_dir = self._resolve_output_dir()
        if self.filename:
            filename = self.filename
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.filename_prefix}_{timestamp}.wav"
        if not filename.lower().endswith(".wav"):
            filename = f"{filename}.wav"
        output_path = os.path.join(output_dir, filename)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        return output_path

    def start(self):
        output_path = self._build_output_path()
        cmd = [
            "arecord",
            "-D",
            self.device,
            "-f",
            self.sample_format,
            "-c",
            str(self.channels),
            "-r",
            str(self.sample_rate),
            "-q",
        ]
        if self.duration > 0:
            cmd.extend(["-d", str(self.duration)])
        cmd.append(output_path)

        rospy.loginfo("Starting rear mic recording: %s", output_path)
        rospy.loginfo("Command: %s", " ".join(cmd))

        self.process = subprocess.Popen(cmd)

        if self.duration > 0:
            code = self.process.wait()
            if code == 0:
                rospy.loginfo("Recording finished: %s", output_path)
            else:
                rospy.logerr("arecord exited with code %d", code)
            rospy.signal_shutdown("Recording completed")
            return

        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            if self.process.poll() is not None:
                rospy.logerr("arecord exited unexpectedly with code %d", self.process.returncode)
                rospy.signal_shutdown("arecord exited")
                break
            rate.sleep()

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is not None:
            return

        rospy.loginfo("Stopping rear mic recording")
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()


def main():
    rospy.init_node("rear_mic_recorder")
    recorder = RearMicRecorder()
    rospy.on_shutdown(recorder.stop)

    try:
        recorder.start()
    except FileNotFoundError:
        rospy.logerr("arecord not found. Install alsa-utils.")
        rospy.signal_shutdown("missing arecord")
    except Exception as error:
        rospy.logerr("Failed to record audio: %s", str(error))
        rospy.signal_shutdown("recorder error")


if __name__ == "__main__":
    main()

#!/usr/bin/env python

import os
import argparse
import ctypes
import sys
import threading
import time
from dataclasses import dataclass
from typing import Final, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
DEFAULT_CONDA_PREFIX = "/home/aorta-scan/miniconda3/envs/clarius_py39"
DEFAULT_VENV_PYTHON = os.path.join(REPO_ROOT, ".venv", "bin", "python")


def _ensure_supported_python() -> None:
    if sys.version_info[:2] == (3, 9):
        return
    if os.environ.get("CLARIUS_DRIVER_REEXEC") == "1":
        return
    target_python = os.environ.get("CLARIUS_DRIVER_PYTHON", DEFAULT_VENV_PYTHON)
    if not os.path.exists(target_python):
        return
    os.environ["CLARIUS_DRIVER_REEXEC"] = "1"
    os.execv(target_python, [target_python, os.path.abspath(__file__), *sys.argv[1:]])


def _candidate_conda_prefixes():
    prefixes = []
    current = os.environ.get("CONDA_PREFIX")
    if current:
        prefixes.append(current)
    prefixes.append(DEFAULT_CONDA_PREFIX)
    seen = set()
    for prefix in prefixes:
        if prefix and prefix not in seen:
            seen.add(prefix)
            yield prefix


def _load_linux_runtime() -> None:
    for prefix in _candidate_conda_prefixes():
        libstdcpp = os.path.join(prefix, "lib", "libstdc++.so.6")
        libgcc = os.path.join(prefix, "lib", "libgcc_s.so.1")
        if os.path.exists(libstdcpp):
            ctypes.CDLL(libstdcpp, ctypes.RTLD_GLOBAL)
        if os.path.exists(libgcc):
            ctypes.CDLL(libgcc, ctypes.RTLD_GLOBAL)

    libcast_path = os.path.join(SCRIPT_DIR, "libcast.so")
    if not os.path.exists(libcast_path):
        raise FileNotFoundError(f"libcast.so not found at {libcast_path}")

    if SCRIPT_DIR not in sys.path:
        sys.path.insert(0, SCRIPT_DIR)

    ctypes.CDLL(libcast_path, ctypes.RTLD_GLOBAL)

_ensure_supported_python()

if sys.platform.startswith("linux"):
    _load_linux_runtime()

import pyclariuscast


import rospy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Float32, Empty

# ----------------------------------------------------------------------
# Clarius user function IDs (from vendor examples/headers)
# ----------------------------------------------------------------------

CMD_FREEZE: Final = 1  # userFunction ID for freeze toggle
CMD_CAPTURE_IMAGE: Final = 2
CMD_CAPTURE_CINE: Final = 3
CMD_DEPTH_DEC: Final = 4
CMD_DEPTH_INC: Final = 5
CMD_GAIN_DEC: Final = 6
CMD_GAIN_INC: Final = 7
CMD_B_MODE: Final = 12
CMD_CFI_MODE: Final = 14


@dataclass
class Frame:
    """Grayscale frame data suitable for ROS publishing and UI display"""

    width: int
    height: int
    step: int  # bytes per line (stride)
    data: bytes  # grayscale 8-bit image (height * step bytes)
    timestamp: rospy.Time  # seconds (from Clarius or system)
    microns_per_pixel: float
    encoding: str  # e.g. "mono8", "mono16", "rgba8"
    is_bigendian: int = 0  # ROS Image field (0 = little-endian)


class FrameStore:
    """Thread-safe storage for the most recent frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[Frame] = None

    def update(self, frame: Frame) -> Frame:
        with self._lock:  # TODO: Could cause problems in the cast callback
            self._frame = frame
        return frame

    def get_latest(self) -> Optional[Frame]:
        with self._lock:
            return self._frame


# Global frame store
frame_store = FrameStore()


class RosClariusService:

    def __init__(self, cast: pyclariuscast.Caster) -> None:
        self._cast = cast
        self._lock = threading.Lock()

        self._pub = rospy.Publisher("/clarius/bmode", RosImage, queue_size=10)

        rospy.Subscriber("/clarius/tx_freq", Float32, self._cb_tx_freq)
        rospy.Subscriber("/clarius/tx_focus", Float32, self._cb_tx_focus)
        rospy.Subscriber("/clarius/toggle_freeze",
                         Empty, self._cb_toggle_freeze)
        rospy.Subscriber("/clarius/request_image",
                         Empty, self._cb_request_image)

    def _cb_tx_freq(self, msg: Float32):
        with self._lock:
            cast = self._cast
            if cast is None:
                rospy.logwarn("Clarius not connected - cannot set txFreq")
                return
            val = float(msg.data)
            ok = cast.setParam("txFreq", val)
        if not ok:
            rospy.logwarn(f"Failed to set txFreq to {val:.2f} MHz")
        else:
            rospy.loginfo(f"Set txFreq to {val:.2f} MHz")

    def _cb_tx_focus(self, msg: Float32):
        with self._lock:
            cast = self._cast
            if cast is None:
                rospy.logwarn("Clarius not connected - cannot set txFocus")
                return
            val = float(msg.data)
            ok = cast.setParam("txFocus", val)
        if not ok:
            rospy.logwarn(f"Failed to set txFocus to {val:.2f} cm")
        else:
            rospy.loginfo(f"Set txFocus to {val:.2f} cm")

    def _cb_toggle_freeze(self, msg: Empty):
        with self._lock:
            cast = self._cast
            connected = cast is not None and cast.isConnected()
            if not connected:
                rospy.logwarn("Clarius not connected - cannot toggle freeze")
                return
            cast.userFunction(CMD_FREEZE, 0)

    def _cb_request_image(self, msg: Empty):
        rospy.loginfo("Frame capture triggered. Publishing...")
        self.publish_latest_frame()

    def publish_latest_frame(self):
        frame = frame_store.get_latest()
        if frame is None:
            rospy.logwarn("No frame available to publish.")
            return
        self._publish_frame(frame)

    def _publish_frame(self, frame: Frame):
        if rospy.is_shutdown():
            return

        msg = RosImage()

        msg.header.stamp = frame.timestamp

        msg.height = frame.height
        msg.width = frame.width
        msg.encoding = frame.encoding
        msg.is_bigendian = frame.is_bigendian
        msg.step = frame.step
        msg.data = frame.data

        self._pub.publish(msg)
        rospy.loginfo(
            f"Published frame ({frame.encoding}, {frame.width:d}x{frame.height:d}).")


def newProcessedImage(image, width, height, sz, micronsPerPixel, timestamp, angle, imu):
    # integer bytes-per-pixel
    bpp = sz // (width * height)

    if bpp == 1:
        # 8-bit grayscale
        encoding = "mono8"
        step = width  # 1 byte per pixel

        # Ensure we own the buffer: make a copy
        data = bytes(image)

    elif bpp == 4:
        encoding = "rgba8"
        step = width * 4

        # Option 1: keep RGBA and just copy
        data = bytes(image)

        # Option 2 conversion to mono:
        # pil_img = Image.frombytes("RGBA", (width, height), image)
        # pil_img = pil_img.convert("L")
        # encoding = width
        # step = width
        # data = pil_img.tobytes()

    else:
        rospy.logwarn(f"Unsupported bytes-per-pixel: {bpp:d}")
        return

    # Timestamp: prefer probe timestamp if valid
    ts = rospy.Time.now()

    frame = Frame(
        width=width,
        height=height,
        step=step,
        data=data,
        timestamp=ts,
        microns_per_pixel=micronsPerPixel,
        encoding=encoding,
        is_bigendian=0
    )

    frame_store.update(frame)


def newRawImage(image, lines, samples, bps, axial, lateral, timestamp, jpg, rf, angle):
    return


def newSpectrumImage(
    image, lines, samples, bps, period, micronsPerSample, velocityPerSample, pw
):
    return


def newImuData(imu):
    return


def freezeFn(frozen):
    if frozen:
        rospy.loginfo("Clarius imaging frozen")
    else:
        rospy.loginfo("Clarius imaging running")


def buttonsFn(button, clicks):
    return


def _destroy_cast(cast) -> None:
    if cast is None:
        return
    try:
        if cast.isConnected():
            cast.disconnect()
    except Exception:
        pass
    try:
        cast.destroy()
    except Exception:
        pass


def _fatal_exit(cast, message: str, code: int = 1) -> None:
    rospy.logerr(message)
    _destroy_cast(cast)
    os._exit(code)


def _connect_with_retry(
    cast,
    ip: str,
    port: int,
    certificate: str,
    retries: int,
    retry_delay: float,
    settle_timeout: float,
) -> bool:
    attempts = max(1, retries)
    for attempt in range(1, attempts + 1):
        try:
            ok = cast.connect(ip, port, certificate)
        except Exception as exc:
            rospy.logwarn(f"Connect attempt {attempt}/{attempts} raised: {exc}")
            ok = False

        if ok:
            deadline = time.time() + max(0.0, settle_timeout)
            while time.time() < deadline:
                try:
                    if cast.isConnected():
                        rospy.loginfo(f"Connected to probe on attempt {attempt}/{attempts}")
                        return True
                except Exception:
                    break
                time.sleep(0.25)

            rospy.loginfo(
                f"Connect request accepted on attempt {attempt}/{attempts}; continuing without immediate isConnected() confirmation"
            )
            return True

        if attempt < attempts:
            rospy.logwarn(
                f"Connect attempt {attempt}/{attempts} failed for {ip}:{port}. Retrying in {retry_delay:.1f}s"
            )
            time.sleep(retry_delay)

    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ip", "-a", dest="ip", default="192.168.1.1", help="IP address of the probe"
    )
    parser.add_argument("--port", "-p", type=int,
                        required=True, help="Port of the probe")
    parser.add_argument("--connect-retries", type=int, default=20)
    parser.add_argument("--connect-retry-delay", type=float, default=1.0)
    parser.add_argument("--connect-settle-timeout", type=float, default=5.0)
    args = parser.parse_args()

    cast = pyclariuscast.Caster(
        newProcessedImage,
        newRawImage,
        newSpectrumImage,
        newImuData,
        freezeFn,
        buttonsFn,
    )

    rospy.init_node("clarius_bmode_node", anonymous=True, disable_signals=True)
    rospy.loginfo("Clarius node initialized")

    ros_service = RosClariusService(cast)
    rospy.loginfo("ROS B-mode service initialized")

    keys_dir = os.path.expanduser("~/")
    if not cast.init(keys_dir, 640, 480):
        _fatal_exit(cast, "Failed to initialize Clarius.")

    rospy.loginfo("Clarius initialized. Connecting...")

    ok = _connect_with_retry(
        cast,
        args.ip,
        args.port,
        "research",
        args.connect_retries,
        args.connect_retry_delay,
        args.connect_settle_timeout,
    )
    if not ok:
        _fatal_exit(cast, f"Failed to connect to probe at {args.ip}:{args.port:d}")

    cast.separateOverlays(True)
    rospy.loginfo("Connected to probe. Ready to use.")

    try:
        rospy.spin()
    finally:
        _destroy_cast(cast)


if __name__ == "__main__":
    main()

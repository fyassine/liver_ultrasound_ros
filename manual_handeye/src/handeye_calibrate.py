#!/usr/bin/env python3
import os
import numpy as np
import rospy
import tf2_ros
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_matrix


def tfmsg_to_T(transform_stamped):
    q = transform_stamped.transform.rotation
    t = transform_stamped.transform.translation
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T


def invert_T(T):
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


class HandEyeCapture:
    def __init__(self):
        self.bridge = CvBridge()

        # Frames
        self.world_frame = rospy.get_param("~world_frame", "iiwa_link_0")  # base/world frame
        self.hand_frame  = rospy.get_param("~hand_frame",  "iiwa_link_ee")  # end-effector/hand frame

        # Topics
        self.image_topic = rospy.get_param("~image_topic", "/rgb/image_raw")
        self.info_topic  = rospy.get_param("~info_topic",  "/rgb/camera_info")

        # ArUco params
        self.marker_length = float(rospy.get_param("~marker_length_m", 0.1))  # meters
        self.marker_id     = int(rospy.get_param("~marker_id", 582))
        dict_name          = rospy.get_param("~aruco_dict", "DICT_ARUCO_ORIGINAL")

        # Output
        self.out_file = rospy.get_param("~out_file", os.path.expanduser("~/handeye_samples.npz"))

        # Intrinsics
        self.K = None
        self.D = None

        # TF
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # ArUco init
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco not available. Install opencv-contrib-python.")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if not hasattr(cv2.aruco, "ArucoDetector"):
            raise RuntimeError("OpenCV build missing ArucoDetector (unexpected for 4.12).")

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.samples = []

        rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)

        rospy.loginfo("OpenCV version: %s", cv2.__version__)
        rospy.loginfo("HandEyeCapture ready. Press 's' to save, 'q' to quit.")
        rospy.loginfo("Saving to: %s", self.out_file)

    def cb_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.D, dtype=np.float64).reshape(1, -1)
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            rospy.loginfo("Got intrinsics: %dx%d, fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                          msg.width, msg.height, fx, fy, cx, cy)

    def estimate_marker_pose_pnp(self, img_corners_4x2):
        """Estimate marker pose using PnP."""
        half = self.marker_length / 2.0
        obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        img_pts = np.array(img_corners_4x2, dtype=np.float32).reshape(4, 2)
        flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D, flags=flag)
        return ok, rvec, tvec

    def cb_image(self, msg: Image):
        if self.K is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _rej = self.aruco_detector.detectMarkers(gray)

        detected = False
        rvec = None
        tvec = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            rospy.loginfo_throttle(1.0, f"Detected ArUco IDs: {ids_flat.tolist()}")

            if self.marker_id in ids_flat:
                idx = int(np.where(ids_flat == self.marker_id)[0][0])
                c = corners[idx].reshape(4, 2)
                ok, rvec, tvec = self.estimate_marker_pose_pnp(c)
                if ok:
                    detected = True
                    cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.marker_length * 0.75, 2)
            
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

        cv2.putText(img, f"samples: {len(self.samples)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img, f"marker_id: {self.marker_id} detected: {detected}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("handeye_capture", img)
        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            rospy.loginfo_throttle(1.0, f"Key pressed: {key} ({chr(key) if 32 <= key <= 126 else ''})")

        if key == ord('q'):
            self.save_npz()
            rospy.signal_shutdown("Quit")
            return

        if key == ord('s'):
            if not detected:
                rospy.logwarn("Marker not detected / pose not estimated. Not saving.")
                return

            stamp = msg.header.stamp
            
            # Get robot forward kinematics: world -> hand (NOT hand -> world)
            try:
                tf_wh = self.tf_buf.lookup_transform(
                    self.world_frame, self.hand_frame, stamp, rospy.Duration(0.5))
            except Exception as e:
                rospy.logwarn("TF lookup failed: %s", str(e))
                try:
                    tf_wh = self.tf_buf.lookup_transform(
                        self.world_frame, self.hand_frame, rospy.Time(0), rospy.Duration(0.5))
                except Exception as e2:
                    rospy.logwarn("TF latest failed: %s", str(e2))
                    return
            
            # lookup_transform(world, hand) returns World->Hand transform (forward kinematics)
            T_WH = tfmsg_to_T(tf_wh)
            
            # For OpenCV calibrateHandEye, we need Hand->World, so invert it
            T_HW = invert_T(T_WH)
            
            # solvePnP gives Camera->Marker transform
            R_CM, _ = cv2.Rodrigues(rvec)
            t_CM = tvec.reshape(3, 1)
            
            # Invert to get Marker->Camera (needed for OpenCV)
            R_MC = R_CM.T
            t_MC = -R_MC @ t_CM
            
            sample = {
                "R_hand2world": T_HW[:3, :3],      # Hand->World (for OpenCV)
                "t_hand2world": T_HW[:3, 3].reshape(3, 1),
                "R_marker2cam": R_MC,              # Marker->Camera (for OpenCV)
                "t_marker2cam": t_MC,
                "R_cam2marker": R_CM,              # Camera->Marker (from solvePnP)
                "t_cam2marker": t_CM,
                "stamp":        float(stamp.to_sec())
            }
            self.samples.append(sample)
            rospy.loginfo("Saved sample %d at t=%.6f", len(self.samples), sample["stamp"])
            self.save_npz()

    def save_npz(self):
        if len(self.samples) == 0:
            rospy.logwarn("No samples to save.")
            return

        Rs_h2w = np.stack([s["R_hand2world"] for s in self.samples], axis=0)
        ts_h2w = np.stack([s["t_hand2world"] for s in self.samples], axis=0)
        Rs_m2c = np.stack([s["R_marker2cam"] for s in self.samples], axis=0)
        ts_m2c = np.stack([s["t_marker2cam"] for s in self.samples], axis=0)
        Rs_c2m = np.stack([s["R_cam2marker"] for s in self.samples], axis=0)
        ts_c2m = np.stack([s["t_cam2marker"] for s in self.samples], axis=0)
        stamps = np.array([s["stamp"] for s in self.samples], dtype=np.float64)

        np.savez(self.out_file,
                 R_hand2world=Rs_h2w, t_hand2world=ts_h2w,
                 R_marker2cam=Rs_m2c, t_marker2cam=ts_m2c,
                 R_cam2marker=Rs_c2m, t_cam2marker=ts_c2m,
                 stamp=stamps)
        rospy.loginfo("Wrote %d samples to %s", len(self.samples), self.out_file)
        rospy.loginfo("Ready for calibration. Run: rosrun manual_handeye handeye_solve.py --npz %s", self.out_file)


if __name__ == "__main__":
    rospy.init_node("handeye_capture")
    HandEyeCapture()
    rospy.spin()

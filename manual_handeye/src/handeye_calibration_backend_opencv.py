#!/usr/bin/env python3
"""
Standalone hand-eye calibration using the easy_handeye backend logic.
This version works directly with NPZ data without ROS dependencies.
"""
import cv2
import numpy as np
import sys
from scipy.spatial.transform import Rotation


class HandeyeCalibrationBackendOpenCV(object):
    MIN_SAMPLES = 2
    """Minimum samples required for a successful calibration."""

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def _get_opencv_samples_from_npz(data):
        """
        Returns the sample list from NPZ data as rotation matrices and translation vectors.
        Matches the format that easy_handeye uses.
        
        :param data: NPZ file data with keys R_hand2world, t_hand2world, R_marker2cam, t_marker2cam
        :rtype: ((list, list), (list, list))
        """
        hand_world_rot = []
        hand_world_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        n_samples = data["R_hand2world"].shape[0]
        
        for i in range(n_samples):
            # Robot poses: Hand->World (already correct for OpenCV, no inversion needed)
            R_HW = data["R_hand2world"][i]
            t_HW = data["t_hand2world"][i]
            
            hand_world_rot.append(R_HW)
            hand_world_tr.append(t_HW)
            
            # Marker poses: Marker->Camera (already correct for OpenCV)
            R_MC = data["R_marker2cam"][i]
            t_MC = data["t_marker2cam"][i]
            
            marker_camera_rot.append(R_MC)
            marker_camera_tr.append(t_MC)

        return (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr)

    def compute_calibration_from_npz(self, npz_file, algorithm='Tsai-Lenz'):
        """
        Computes the calibration from NPZ file using the same logic as easy_handeye.

        :param npz_file: Path to NPZ file with calibration samples
        :param algorithm: Algorithm name (Tsai-Lenz, Park, Horaud, Andreff, Daniilidis)
        :rtype: numpy.ndarray (4x4 transformation matrix)
        """
        try:
            data = np.load(npz_file)
        except FileNotFoundError:
            print(f"Error: File not found: {npz_file}")
            return None
        except Exception as e:
            print(f"Error loading NPZ: {e}")
            return None

        print(f'Calibrating with algorithm: {algorithm}')

        # Get samples in OpenCV format
        opencv_samples = self._get_opencv_samples_from_npz(data)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        n_samples = len(hand_world_rot)
        if n_samples < self.MIN_SAMPLES:
            print(f"Error: Need at least {self.MIN_SAMPLES} samples, got {n_samples}")
            return None

        print(f"Computing from {n_samples} poses...")

        method = self.AVAILABLE_ALGORITHMS[algorithm]

    
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
            hand_world_rot, hand_world_tr, 
            marker_camera_rot, marker_camera_tr, 
            method=method
        )
        
        # Build 4x4 matrix
        result = np.eye(4)
        result[:3, :3] = hand_camera_rot
        result[:3, 3] = hand_camera_tr.ravel()

        print("Computed calibration:")
        print(result)

        return result


def print_transform_info(T, name):
    """Print transformation details."""
    R, t = T[:3, :3], T[:3, 3]
    rot = Rotation.from_matrix(R)
    quat = rot.as_quat()
    euler = rot.as_euler('xyz', degrees=True)
    
    print(f"\n{name}:")
    print("=" * 70)
    print(f"Translation [m]: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    print(f"Distance: {np.linalg.norm(t):.4f} m")
    print(f"Quaternion [x,y,z,w]: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
    print(f"Euler [deg]: [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]")
    print(f"\n{T}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Solve hand-eye calibration using easy_handeye backend")
    parser.add_argument("--npz", required=True, help="NPZ file with samples")
    parser.add_argument("--algorithm", default="Tsai-Lenz", 
                        choices=['Tsai-Lenz', 'Park', 'Horaud', 'Andreff', 'Daniilidis'],
                        help="Calibration algorithm")
    args = parser.parse_args()
    
    backend = HandeyeCalibrationBackendOpenCV()
    T_gripper_to_camera = backend.compute_calibration_from_npz(args.npz, args.algorithm)
    
    if T_gripper_to_camera is not None:
        print("\n" + "=" * 70)
        print("HAND-EYE CALIBRATION RESULT (using easy_handeye backend)")
        print("=" * 70)
        
        T_camera_to_gripper = np.linalg.inv(T_gripper_to_camera)
        
        print_transform_info(T_gripper_to_camera, "Gripper to Camera (hand_camera)")
        print_transform_info(T_camera_to_gripper, "Camera to Gripper (Inverse)")
    else:
        print("\nCalibration failed!")
        sys.exit(1)

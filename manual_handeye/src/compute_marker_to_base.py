#!/usr/bin/env python3
"""Compute marker-to-base: ^B T_M = ^B T_E * ^E T_C * ^C T_M"""
import numpy as np
from scipy.spatial.transform import Rotation
import argparse


def invert_transformation(T):
    """Invert 4x4 transformation matrix."""
    T_inv = np.eye(4)
    R, t = T[:3, :3], T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def transformation_to_matrix(quat, trans):
    """Convert quaternion (x,y,z,w) and translation to 4x4 matrix."""
    R = Rotation.from_quat(quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = trans
    return T


def print_transform(T, name="Transform"):
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


def main():
    parser = argparse.ArgumentParser(description="Compute ^B T_M = ^B T_E * ^E T_C * ^C T_M")
    args = parser.parse_args()
    
    print("\n" + "=" * 70)
    print("MARKER-TO-BASE TRANSFORMATION")
    print("=" * 70)
    
    # Camera -> End-Effector (from EasyHandEye)
    # NOTE: `Rotation.from_quat` expects quaternion order [x, y, z, w].
    quat_EC = ...
    trans_EC = ...
    E_T_C = transformation_to_matrix(quat_EC, trans_EC)
    
    # Marker -> Camera (from vision)
    quat_CM = ...
    trans_CM = ...
    C_T_M = transformation_to_matrix(quat_CM, trans_CM)
    
    # Base to End-Effector (home position)
    B_T_E = ...
    
    # Compute: ^B T_M = ^B T_E * ^E T_C * ^C T_M
    B_T_M = B_T_E @ E_T_C @ C_T_M
    
    print_transform(E_T_C, "^E T_C (Camera to End-Effector)")
    print_transform(C_T_M, "^C T_M (Marker to Camera)")
    print_transform(B_T_E, "^B T_E (Base to End-Effector)")
    print_transform(B_T_M, "^B T_M (Marker to Base) = ^B T_E * ^E T_C * ^C T_M")


if __name__ == '__main__':
    main()

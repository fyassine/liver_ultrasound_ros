#!/usr/bin/env python3
"""
View hand-eye calibration samples from NPZ file in quaternion format.
This script reads the samples collected by handeye_calibrate.py and displays them
using quaternions instead of rotation matrices.
"""
import os
import sys
import argparse
import numpy as np
from scipy.spatial.transform import Rotation


def format_pose_quaternion(R, t, indent="  "):
    """Format rotation matrix (as quaternion) and translation vector as readable text."""
    lines = []
    lines.append(f"{indent}Translation (x, y, z) [m]:")
    lines.append(f"{indent}  [{t[0, 0]:9.6f}, {t[1, 0]:9.6f}, {t[2, 0]:9.6f}]")
    
    # Convert rotation matrix to quaternion
    rot = Rotation.from_matrix(R)
    quat = rot.as_quat()  # [x, y, z, w]
    
    lines.append(f"{indent}Quaternion (x, y, z, w):")
    lines.append(f"{indent}  [{quat[0]:9.6f}, {quat[1]:9.6f}, {quat[2]:9.6f}, {quat[3]:9.6f}]")
    
    # Also show Euler angles for reference
    euler = rot.as_euler('xyz', degrees=True)
    lines.append(f"{indent}Euler angles (roll, pitch, yaw) [deg]:")
    lines.append(f"{indent}  [{euler[0]:8.3f}, {euler[1]:8.3f}, {euler[2]:8.3f}]")
    
    return "\n".join(lines)


def print_samples(npz_file, output_file=None):
    """Print or export calibration samples from NPZ file."""
    if not os.path.exists(npz_file):
        print(f"Error: File '{npz_file}' not found.")
        return False
    
    try:
        data = np.load(npz_file)
    except Exception as e:
        print(f"Error loading NPZ file: {e}")
        return False
    
    # Check what keys are available
    print(f"\nLoading: {npz_file}")
    print(f"Available keys: {list(data.keys())}")
    
    # Extract data (support both old and new key names)
    if 'R_hand2world' in data.keys():
        # New naming convention
        R_hand2world = data['R_hand2world']
        t_hand2world = data['t_hand2world']
        R_marker2cam = data['R_marker2cam']
        t_marker2cam = data['t_marker2cam']
        has_cam2marker = 'R_cam2marker' in data.keys()
        if has_cam2marker:
            R_cam2marker = data['R_cam2marker']
            t_cam2marker = data['t_cam2marker']
        else:
            has_cam2marker = False
        robot_label = "Robot Pose (Hand -> World)"
        marker_label = "Marker Pose (Marker -> Camera)"
        cam_label = "Camera Pose (Camera -> Marker)"
    elif 'R_world2hand' in data.keys():
        # Old naming (World->Hand, needs inversion for display)
        R_WH = data['R_world2hand']
        t_WH = data['t_world2hand']
        R_hand2world = np.zeros_like(R_WH)
        t_hand2world = np.zeros_like(t_WH)
        for i in range(R_WH.shape[0]):
            R_hand2world[i] = R_WH[i].T
            t_hand2world[i] = (-R_hand2world[i] @ t_WH[i]).reshape(3, 1)
        R_marker2cam = data['R_marker2cam']
        t_marker2cam = data['t_marker2cam']
        has_cam2marker = 'R_cam2marker' in data.keys()
        if has_cam2marker:
            R_cam2marker = data['R_cam2marker']
            t_cam2marker = data['t_cam2marker']
        else:
            has_cam2marker = False
        robot_label = "Robot Pose (Hand -> World)"
        marker_label = "Marker Pose (Marker -> Camera)"
        cam_label = "Camera Pose (Camera -> Marker)"
    elif 'R_gripper2base' in data.keys():
        # Very old naming
        R_hand2world = data['R_gripper2base']
        t_hand2world = data['t_gripper2base']
        R_marker2cam = data['R_target2cam']
        t_marker2cam = data['t_target2cam']
        has_cam2marker = False
        robot_label = "Robot Pose (Gripper -> Base)"
        marker_label = "Marker Pose (Target -> Camera)"
        cam_label = ""
    else:
        print(f"Error: Unrecognized data format. Available keys: {list(data.keys())}")
        return False
    
    stamps = data['stamp']
    n_samples = R_hand2world.shape[0]
    print(f"\nNumber of samples: {n_samples}")
    
    if n_samples == 0:
        print("No samples found in file.")
        return False
    
    # Prepare output
    output_lines = []
    output_lines.append("=" * 80)
    output_lines.append(f"Hand-Eye Calibration Samples (Quaternion Format): {os.path.basename(npz_file)}")
    output_lines.append("=" * 80)
    output_lines.append(f"Total samples: {n_samples}")
    output_lines.append(f"File: {npz_file}")
    output_lines.append("=" * 80)
    output_lines.append("")
    
    # Print each sample
    for i in range(n_samples):
        output_lines.append(f"Sample {i + 1}/{n_samples}")
        output_lines.append(f"Timestamp: {stamps[i]:.6f} seconds")
        output_lines.append("")
        
        output_lines.append(f"  {robot_label}:")
        output_lines.append(format_pose_quaternion(R_hand2world[i], t_hand2world[i], "    "))
        output_lines.append("")
        
        output_lines.append(f"  {marker_label}:")
        output_lines.append(format_pose_quaternion(R_marker2cam[i], t_marker2cam[i], "    "))
        output_lines.append("")
        
        if has_cam2marker:
            output_lines.append(f"  {cam_label}:")
            output_lines.append(format_pose_quaternion(R_cam2marker[i], t_cam2marker[i], "    "))
            output_lines.append("")
        output_lines.append("-" * 80)
        output_lines.append("")
    
    # Statistics
    output_lines.append("\nSample Statistics:")
    output_lines.append("-" * 80)
    
    # Translation ranges for robot
    t_h2w_min = t_hand2world.min(axis=0).flatten()
    t_h2w_max = t_hand2world.max(axis=0).flatten()
    output_lines.append("Robot Hand Position Range (Hand->World translation) [m]:")
    output_lines.append(f"  X: [{t_h2w_min[0]:7.4f}, {t_h2w_max[0]:7.4f}] (range: {t_h2w_max[0]-t_h2w_min[0]:7.4f})")
    output_lines.append(f"  Y: [{t_h2w_min[1]:7.4f}, {t_h2w_max[1]:7.4f}] (range: {t_h2w_max[1]-t_h2w_min[1]:7.4f})")
    output_lines.append(f"  Z: [{t_h2w_min[2]:7.4f}, {t_h2w_max[2]:7.4f}] (range: {t_h2w_max[2]-t_h2w_min[2]:7.4f})")
    output_lines.append("")
    
    # Translation ranges for marker
    t_m2c_min = t_marker2cam.min(axis=0).flatten()
    t_m2c_max = t_marker2cam.max(axis=0).flatten()
    output_lines.append("Marker Position Range (Marker->Camera translation) [m]:")
    output_lines.append(f"  X: [{t_m2c_min[0]:7.4f}, {t_m2c_max[0]:7.4f}] (range: {t_m2c_max[0]-t_m2c_min[0]:7.4f})")
    output_lines.append(f"  Y: [{t_m2c_min[1]:7.4f}, {t_m2c_max[1]:7.4f}] (range: {t_m2c_max[1]-t_m2c_min[1]:7.4f})")
    output_lines.append(f"  Z: [{t_m2c_min[2]:7.4f}, {t_m2c_max[2]:7.4f}] (range: {t_m2c_max[2]-t_m2c_min[2]:7.4f})")
    output_lines.append("")
    
    # Time span
    if n_samples > 1:
        time_span = stamps[-1] - stamps[0]
        output_lines.append(f"Time span: {time_span:.3f} seconds ({time_span/60:.2f} minutes)")
    
    output_lines.append("=" * 80)
    
    # Output to screen and/or file
    output_text = "\n".join(output_lines)
    print(output_text)
    
    if output_file:
        try:
            with open(output_file, 'w') as f:
                f.write(output_text)
            print(f"\nOutput saved to: {output_file}")
        except Exception as e:
            print(f"Error writing to file: {e}")
            return False
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description="View hand-eye calibration samples from NPZ file in quaternion format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # View samples from default file
  %(prog)s
  
  # View samples from specific file
  %(prog)s /home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz
  
  # Export to text file
  %(prog)s /home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz -o samples_quat.txt
  
  # Export to same directory with .txt extension
  %(prog)s /home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz -o auto
        """
    )
    
    parser.add_argument(
        'npz_file',
        nargs='?',
        default=os.path.expanduser('/home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz'),
        help='Path to NPZ file (default: /home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz)'
    )
    
    parser.add_argument(
        '-o', '--output',
        dest='output_file',
        help='Output text file path. Use "auto" to create .txt file with same name as input'
    )
    
    args = parser.parse_args()
    
    # Handle auto output file naming
    if args.output_file == 'auto':
        base = os.path.splitext(args.npz_file)[0]
        args.output_file = base + '_quaternion.txt'
    
    # Process the file
    success = print_samples(args.npz_file, args.output_file)
    
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()

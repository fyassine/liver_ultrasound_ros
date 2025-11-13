#!/usr/bin/env python3

import os
import glob
import argparse
import math
import subprocess
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_csvs(cartesian_csv, joint_csv):
    if cartesian_csv is None or not os.path.exists(cartesian_csv):
        raise FileNotFoundError(f"Cartesian CSV not found: {cartesian_csv}")
    if joint_csv is None or not os.path.exists(joint_csv):
        raise FileNotFoundError(f"Joint CSV not found: {joint_csv}")

    cart = pd.read_csv(cartesian_csv)
    joints = pd.read_csv(joint_csv)
    return cart, joints


def plot_3d(cart_df, out_path):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    t = cart_df['time'].values
    x = cart_df['pos_x'].values
    y = cart_df['pos_y'].values
    z = cart_df['pos_z'].values
    # color by time normalized
    tn = (t - t.min()) / (t.max() - t.min() + 1e-12)
    ax.scatter(x, y, z, c=tn, cmap='viridis', s=8)
    ax.plot(x, y, z, color='gray', alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Cartesian 3D Path (colored by time)')
    fig.colorbar(plt.cm.ScalarMappable(cmap='viridis'), ax=ax, label='normalized time')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_cartesian_components(cart_df, out_path):
    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    t = cart_df['time'].values
    x = cart_df['pos_x'].values
    y = cart_df['pos_y'].values
    z = cart_df['pos_z'].values

    mins = np.array([np.nanmin(x), np.nanmin(y), np.nanmin(z)])
    maxs = np.array([np.nanmax(x), np.nanmax(y), np.nanmax(z)])
    ranges = maxs - mins
    max_range = float(np.nanmax(ranges))
    if max_range <= 0:
        max_range = 1e-6

    centers = 0.5 * (mins + maxs)
    half_span = 0.5 * max_range

    axs[0].plot(t, x, '-b')
    axs[0].set_ylabel('pos_x (m)')
    axs[0].set_ylim(centers[0] - half_span, centers[0] + half_span)

    axs[1].plot(t, y, '-g')
    axs[1].set_ylabel('pos_y (m)')
    axs[1].set_ylim(centers[1] - half_span, centers[1] + half_span)

    axs[2].plot(t, z, '-r')
    axs[2].set_ylabel('pos_z (m)')
    axs[2].set_ylim(centers[2] - half_span, centers[2] + half_span)
    axs[2].set_xlabel('time (s)')

    fig.suptitle('Cartesian components vs time (shared span)')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_joints_timeseries(joints_df, out_path):
    time = joints_df['time'].values
    joint_cols = [c for c in joints_df.columns if c != 'time']
    n = len(joint_cols)
    fig, axs = plt.subplots(n, 1, figsize=(10, 1.8 * n), sharex=True)
    if n == 1:
        axs = [axs]
    joint_values = joints_df[joint_cols].values

    mins = np.nanmin(joint_values, axis=0)
    maxs = np.nanmax(joint_values, axis=0)
    ranges = maxs - mins
    max_range = float(np.nanmax(ranges))
    if max_range <= 0:
        max_range = 1e-6  # fallback small span in original units

    for i, col in enumerate(joint_cols):
        axs[i].plot(time, joint_values[:, i], '-k')
        axs[i].set_ylabel(col)
        center = 0.5 * (mins[i] + maxs[i])
        half_span = 0.5 * max_range
        axs[i].set_ylim(center - half_span, center + half_span)
    axs[-1].set_xlabel('time (s)')
    fig.suptitle('Joint positions vs time')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_joint_intervals(joints_df, out_path, threshold=1e-5):
    time = joints_df['time'].values
    dt = np.diff(time)
    joint_cols = [c for c in joints_df.columns if c != 'time']
    n = len(joint_cols)
    movements = np.abs(np.diff(joints_df[joint_cols].values, axis=0))
    moving = movements > threshold

    fig, ax = plt.subplots(figsize=(10, 1.2 * n))
    y_ticks = []
    y_labels = []
    for i, col in enumerate(joint_cols):
        row = moving[:, i]
        intervals = []
        start_idx = None
        for k, val in enumerate(row):
            if val and start_idx is None:
                start_idx = k
            elif not val and start_idx is not None:
                intervals.append((start_idx, k))
                start_idx = None
        if start_idx is not None:
            intervals.append((start_idx, len(row)))

        bars = []
        for s, e in intervals:
            t0 = time[s]
            t1 = time[e]
            bars.append((t0, t1 - t0))

        ax.broken_barh(bars, (i - 0.4, 0.8), facecolors='tab:blue')
        y_ticks.append(i)
        y_labels.append(col)

    ax.set_ylim(-1, n)
    ax.set_yticks(y_ticks)
    ax.set_yticklabels(y_labels)
    ax.set_xlabel('time (s)')
    ax.set_title(f'Joint movement intervals (threshold={threshold})')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description='Plot cartesian and joint CSVs.')
    p.add_argument('--cartesian-csv', help='Path to cartesian CSV')
    p.add_argument('--joint-csv', help='Path to joint CSV')
    p.add_argument('--bag', help='Bag path (if CSVs missing, run inspect_bag to generate them)')
    p.add_argument('--threshold', type=float, default=1e-5, help='Movement threshold for intervals')
    p.add_argument('--out-dir', help='Output directory for PNGs (defaults to CSV dir)')
    p.add_argument('--show', action='store_true', help='Show plots (not recommended in headless)')
    args = p.parse_args()

    # Prefer using provided CSVs. If not provided, try to auto-discover CSVs in
    # the current directory and in the usual `src/iiwa_trajectory/bags` folder.
    cart_csv = args.cartesian_csv
    joint_csv = args.joint_csv

    def discover_csvs():
        candidates = []
        # search cwd
        candidates += glob.glob(os.path.join(os.getcwd(), '*_cartesian_pose.csv'))
        candidates += glob.glob(os.path.join(os.getcwd(), '*_joint_positions.csv'))
        # search repo's bags directory
        repo_bags = os.path.join(os.path.dirname(__file__), '..', 'bags')
        repo_bags = os.path.normpath(repo_bags)
        candidates += glob.glob(os.path.join(repo_bags, '*_cartesian_pose.csv'))
        candidates += glob.glob(os.path.join(repo_bags, '*_joint_positions.csv'))
        return list(sorted(set(candidates)))

    # If no explicit CSVs, try to find matching files
    if cart_csv is None or joint_csv is None:
        found = discover_csvs()
        # map found names
        found_cart = [f for f in found if f.endswith('_cartesian_pose.csv')]
        found_joint = [f for f in found if f.endswith('_joint_positions.csv')]
        if not cart_csv and found_cart:
            cart_csv = found_cart[0]
        if not joint_csv and found_joint:
            joint_csv = found_joint[0]

    # If still missing CSVs, but --bag provided, run inspect_bag to generate them
    if (cart_csv is None or joint_csv is None) and args.bag:
        print('CSV files not found — invoking inspect_bag to generate CSVs from bag...')
        inspect_script = os.path.join(os.path.dirname(__file__), 'inspect_bag.py')
        try:
            subprocess.run([inspect_script, args.bag], check=True)
        except Exception as e:
            print(f'Failed to run inspect_bag: {e}')
        # after running, try to infer CSV paths from bag
        bag_base = os.path.splitext(os.path.basename(args.bag))[0]
        bag_dir = os.path.dirname(os.path.abspath(args.bag)) or os.getcwd()
        if cart_csv is None:
            candidate = os.path.join(bag_dir, f"{bag_base}_cartesian_pose.csv")
            if os.path.exists(candidate):
                cart_csv = candidate
        if joint_csv is None:
            candidate = os.path.join(bag_dir, f"{bag_base}_joint_positions.csv")
            if os.path.exists(candidate):
                joint_csv = candidate

    # Final check
    if cart_csv is None or joint_csv is None:
        print('CSV files not found. Provide --bag to generate them or place CSVs in working dir.')
        return 1

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(cart_csv))
    os.makedirs(out_dir, exist_ok=True)

    cart_df, joints_df = load_csvs(cart_csv, joint_csv)

    base = os.path.splitext(os.path.basename(cart_csv))[0]
    out_3d = os.path.join(out_dir, f"{base}_cartesian_3d.png")
    out_components = os.path.join(out_dir, f"{base}_cartesian_components.png")
    out_joints = os.path.join(out_dir, f"{base}_joints_timeseries.png")
    out_intervals = os.path.join(out_dir, f"{base}_joints_intervals.png")

    print(f"Writing: {out_3d}")
    plot_3d(cart_df, out_3d)
    print(f"Writing: {out_components}")
    plot_cartesian_components(cart_df, out_components)
    print(f"Writing: {out_joints}")
    plot_joints_timeseries(joints_df, out_joints)
    print(f"Writing: {out_intervals}")
    plot_joint_intervals(joints_df, out_intervals, threshold=args.threshold)

    print('Plots written to', out_dir)
    if args.show:
        try:
            import subprocess
            if os.name == 'posix':
                subprocess.run(['xdg-open', out_3d])
        except Exception:
            pass


if __name__ == '__main__':
    main()

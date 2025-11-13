#!/usr/bin/env python3

import argparse
import os
import sys
import time

import subprocess
import shutil

try:
    import rospy
    import rosbag
except Exception:
    rospy = None
    rosbag = None



DEFAULT_BAG = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'bags', 'recorded_trajectory_iiwa.bag'))
DEFAULT_READ_TOPIC = '/iiwa/state/JointPosition'
DEFAULT_PUBLISH_TOPIC = '/iiwa/command/JointPosition'


def publish_only_joints(bag_path, read_topic, publish_topic, speed=1.0, loop=False, verbose=False):
    if rospy is None or rosbag is None:
        print('This script requires a ROS Python environment (rospy, rosbag).')
        print('Source your workspace (e.g. `source devel/setup.bash`) and retry.')
        return 2

    rospy.init_node('publish_joint_positions_simple', anonymous=True)

    if verbose:
        print(f'Opening bag: {bag_path}')
        print(f'Reading topic: {read_topic}')
        print(f'Publishing to: {publish_topic}')

    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f'Failed to open bag: {e}')
        return 1

    pub = None

    try:
        while not rospy.is_shutdown():
            prev_t = None
            for topic, msg, t in bag.read_messages(topics=[read_topic]):
                if pub is None:
                    try:
                        msg_type = msg.__class__
                        pub = rospy.Publisher(publish_topic, msg_type, queue_size=10)
                        # small pause to allow registration
                        rospy.sleep(0.05)
                    except Exception as e:
                        print('Failed to create publisher for message type:', e)
                        bag.close()
                        return 1

                cur_t = t.to_sec() if hasattr(t, 'to_sec') else float(t)
                if prev_t is not None:
                    dt = (cur_t - prev_t) / float(speed)
                    if dt > 0:
                        try:
                            time.sleep(dt)
                        except KeyboardInterrupt:
                            bag.close()
                            return 0
                prev_t = cur_t

                try:
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        msg.header.stamp = rospy.Time.now()
                except Exception:
                    pass

                try:
                    pub.publish(msg)
                except Exception as e:
                    print('Publish failed:', e)
                    bag.close()
                    return 1

                if verbose:
                    print(f'Published joint message at {prev_t:.6f}')

            if not loop:
                break
            if verbose:
                print('Loop requested — restarting bag')
    finally:
        try:
            bag.close()
        except Exception:
            pass

    return 0


def main():
    p = argparse.ArgumentParser(description='Publish only joint-position messages from bag to command topic (simple).')
    p.add_argument('--bag', '-b', help='Path to bag file', default=DEFAULT_BAG)
    p.add_argument('--topic', '-t', help='Joint topic to read from bag', default=DEFAULT_READ_TOPIC)
    p.add_argument('--target', '-m', help='Topic to publish joint messages to', default=DEFAULT_PUBLISH_TOPIC)
    p.add_argument('--speed', '-s', type=float, default=1.0, help='Playback speed multiplier')
    p.add_argument('--loop', '-l', action='store_true', help='Loop playback')
    p.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    args = p.parse_args()

    bag_path = args.bag
    if not os.path.exists(bag_path):
        print('Default bag not found at', bag_path)
        print('Provide a bag path with --bag or place the default bag in src/iiwa_trajectory/bags/')
        sys.exit(1)

    if rospy is not None and rosbag is not None:
        rc = publish_only_joints(bag_path, args.topic, args.target, speed=args.speed, loop=args.loop, verbose=args.verbose)
        sys.exit(rc)

    rosbag_bin = shutil.which('rosbag')
    if not rosbag_bin:
        print('Neither Python rospy/rosbag imports succeeded nor rosbag CLI found.')
        print("If you're missing the python 'gnupg' dependency, install it (e.g. `pip install python-gnupg`),")
        print("or install 'rosbag' by installing ROS and sourcing your workspace.")
        sys.exit(2)

    cmd = [rosbag_bin, 'play', bag_path, f"{args.topic}:={args.target}"]
    if args.speed is not None and float(args.speed) != 1.0:
        cmd += ['-r', str(float(args.speed))]
    if args.loop:
        cmd += ['-l']

    if args.verbose:
        print('Using rosbag play with remap. Running:')
        print(' '.join(cmd))

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print('\nPlayback stopped.')
        sys.exit(0)
    except subprocess.CalledProcessError as e:
        print('rosbag play failed:', e)
        sys.exit(1)
    sys.exit(0)


if __name__ == '__main__':
    main()

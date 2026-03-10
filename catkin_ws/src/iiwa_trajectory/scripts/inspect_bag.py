#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys
from textwrap import indent
import os
import csv

try:
    import rosbag
except Exception as e:
    print("This script requires ROS (rosbag). Ensure you run it in a ROS environment.")
    raise

try:
    from genmsg.msg_loader import MsgNotFound
except Exception:
    class MsgNotFound(Exception):
        pass


def parse_rosbag_info(bag_path):
    # Prefer using rosbag API to avoid fragile text parsing of `rosbag info` output.
    info = {'duration': None, 'messages': None, 'topics': {}}
    try:
        bag = rosbag.Bag(bag_path)
        try:
            # get_type_and_topic_info may return (types, topics) or a TopicInfo mapping
            ti = bag.get_type_and_topic_info()
            topics_map = ti[1] if isinstance(ti, (list, tuple)) and len(ti) > 1 else getattr(ti, 'topics', None) or ti
            for topic, tinfo in topics_map.items():
                ttype = getattr(tinfo, 'type', None) or (tinfo[0] if isinstance(tinfo, (list, tuple)) and len(tinfo) > 0 else None)
                count = getattr(tinfo, 'message_count', None) or getattr(tinfo, 'messages', None) or (tinfo[1] if isinstance(tinfo, (list, tuple)) and len(tinfo) > 1 else None)
                try:
                    count = int(count) if count is not None else None
                except Exception:
                    count = None
                info['topics'][topic] = {'count': count, 'type': ttype}

            try:
                info['duration'] = bag.get_end_time() - bag.get_start_time()
            except Exception:
                info['duration'] = None
            try:
                info['messages'] = sum(v['count'] for v in info['topics'].values() if v.get('count') is not None)
            except Exception:
                info['messages'] = None
        finally:
            bag.close()
        return info
    except Exception:
        out = subprocess.check_output(["rosbag", "info", bag_path], text=True, stderr=subprocess.DEVNULL)
        m = re.search(r'duration:\s*([\d.]+)s', out)
        info['duration'] = float(m.group(1)) if m else None
        m = re.search(r'messages:\s*(\d+)', out)
        info['messages'] = int(m.group(1)) if m else None
        topics = {}
        for line in out.splitlines():
            line = line.rstrip()
            m = re.match(r"^\s*(/[^\s]+)\s+(\d+)\s+msgs\s+:\s+(.+)$", line)
            if m:
                topic = m.group(1)
                count = int(m.group(2))
                ttype = m.group(3).strip()
                topics[topic] = {'count': count, 'type': ttype}
        info['topics'] = topics
        return info


def try_print_first_messages(bag_path, topic, n=1):
    bag = rosbag.Bag(bag_path)
    printed = 0
    try:
        for t, msg, ts in bag.read_messages(topics=[topic]):
            print(f"--- message {printed+1} (topic={t}, time={ts.to_sec()}) ---")
            try:
                s = str(msg)
            except Exception:
                s = repr(msg)
            print(indent(s, '  '))
            printed += 1
            if printed >= n:
                break
        if printed == 0:
            print(f"No messages found for topic {topic} in bag.")
    except MsgNotFound as e:
        print(f"Cannot deserialize messages for topic {topic}: missing message definitions (MsgNotFound).")
        print("Tip: source the workspace where the custom message package is built, e.g. 'source devel/setup.bash', or inspect the bag with only 'rosbag info' to see types.")
    except Exception as e:
        # catch genmsg errors too
        msg = str(e)
        if 'Cannot locate message' in msg or 'unknown package' in msg:
            print(f"Cannot deserialize messages for topic {topic}: missing message/package definitions in PYTHONPATH.")
            print("Tip: source the workspace where the message package is built (e.g. 'source devel/setup.bash') so Python/genmsg can find them.")
        else:
            print(f"Error while reading messages for topic {topic}: {e}")
    finally:
        bag.close()


def _extract_pose_from_msg(msg):
    """
    Extract position and orientation from a message.
    Returns (pos_tuple, ori_tuple) or (None, None) on failure.
    """
    try:
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'position') and hasattr(msg.pose, 'orientation'):
            p = msg.pose.position
            o = msg.pose.orientation
            return (float(p.x), float(p.y), float(p.z)), (float(o.x), float(o.y), float(o.z), float(o.w))
    except Exception:
        pass

    try:
        if hasattr(msg, 'position') and hasattr(msg, 'orientation'):
            p = msg.position
            o = msg.orientation
            return (float(p.x), float(p.y), float(p.z)), (float(o.x), float(o.y), float(o.z), float(o.w))
    except Exception:
        pass

    try:
        if hasattr(msg, 'translation') and hasattr(msg, 'rotation'):
            p = msg.translation
            o = msg.rotation
            if all(hasattr(o, a) for a in ('x', 'y', 'z', 'w')):
                return (float(p.x), float(p.y), float(p.z)), (float(o.x), float(o.y), float(o.z), float(o.w))
    except Exception:
        pass

    return None, None


def _get_joint_names_and_positions(msg):
    """
    Extract joint names and positions from a message. Return (names, positions) or (None, None).
    """
    try:
        if hasattr(msg, 'name') and hasattr(msg, 'position'):
            names = list(msg.name)
            positions = list(msg.position)
            return names, positions
    except Exception:
        pass

    try:
        if hasattr(msg, 'joint_names') and hasattr(msg, 'positions'):
            names = list(msg.joint_names)
            positions = list(msg.positions)
            return names, positions
    except Exception:
        pass

    try:
        pos = getattr(msg, 'position', None) or getattr(msg, 'positions', None)
        if pos is not None and hasattr(pos, '__len__') and not isinstance(pos, (str, bytes)):
            n = len(pos)
            names = [f'joint_{i+1}' for i in range(n)]
            positions = list(pos)
            return names, positions
    except Exception:
        pass

    return None, None


def write_cartesian_csv(bag_path, topic, out_csv_path):
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f"Failed to open bag for CartesianPose extraction: {e}")
        return
    wrote_any = False
    try:
        with open(out_csv_path, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(['time', 'pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
            for t, msg, ts in bag.read_messages(topics=[topic]):
                try:
                    pos, ori = _extract_pose_from_msg(msg)
                    if pos is None or ori is None:
                        continue
                    row = [f"{ts.to_sec():.6f}", f"{pos[0]:.9f}", f"{pos[1]:.9f}", f"{pos[2]:.9f}",
                           f"{ori[0]:.9f}", f"{ori[1]:.9f}", f"{ori[2]:.9f}", f"{ori[3]:.9f}"]
                    writer.writerow(row)
                    wrote_any = True
                except MsgNotFound:
                    print("Cannot deserialize CartesianPose messages: missing message definitions.")
                    return
                except Exception:
                    continue
        if wrote_any:
            print(f"Wrote CartesianPose CSV: {out_csv_path}")
        else:
            try:
                os.remove(out_csv_path)
            except Exception:
                pass
            print("No usable CartesianPose messages found; CSV not written.")
    finally:
        bag.close()


def write_joint_csv(bag_path, topic, out_csv_path):
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f"Failed to open bag for JointPosition extraction: {e}")
        return
    wrote_any = False
    header_names = None
    try:
        with open(out_csv_path, 'w', newline='') as fh:
            writer = None
            for t, msg, ts in bag.read_messages(topics=[topic]):
                try:
                    names, positions = _get_joint_names_and_positions(msg)
                    if names is None or positions is None:
                        continue
                    if header_names is None:
                        header_names = list(names)
                        writer = csv.writer(fh)
                        writer.writerow(['time'] + header_names)
                    row = [f"{ts.to_sec():.6f}"]
                    for i in range(len(header_names)):
                        if i < len(positions):
                            try:
                                row.append(f"{float(positions[i]):.9f}")
                            except Exception:
                                row.append('')
                        else:
                            row.append('')
                    writer.writerow(row)
                    wrote_any = True
                except MsgNotFound:
                    print("Cannot deserialize JointPosition messages: missing message definitions.")
                    return
                except Exception:
                    continue
        if wrote_any:
            print(f"Wrote JointPosition CSV: {out_csv_path}")
        else:
            try:
                os.remove(out_csv_path)
            except Exception:
                pass
            print("No usable JointPosition messages found; CSV not written.")
    finally:
        bag.close()


def main():
    p = argparse.ArgumentParser(description='Inspect a rosbag without replaying it.')
    p.add_argument('bag', help='Path to bag file')
    p.add_argument('--topic', help='Inspect only this topic (default: list all topics)')
    p.add_argument('--first', type=int, default=5, help='Print first N messages per topic (default 5)')
    p.add_argument('--summary-only', action='store_true', help='Only print summary and topics, skip printing messages')
    args = p.parse_args()

    bag_path = args.bag

    try:
        info = parse_rosbag_info(bag_path)
    except subprocess.CalledProcessError:
        print(f"Failed to run 'rosbag info' on '{bag_path}'. Is the path correct and 'rosbag' available in PATH?")
        sys.exit(1)

    print(f"Bag: {bag_path}")
    print(f"Duration: {info.get('duration')} s")
    if info.get('messages') is not None:
        print(f"Total messages (report): {info.get('messages')}")
    print("Topics:")
    for topic, meta in sorted(info['topics'].items()):
        freq = None
        if info.get('duration') and meta.get('count') is not None:
            try:
                freq = meta['count'] / info['duration']
            except Exception:
                freq = None
        freq_str = f"{freq:.2f} Hz" if freq else "?"
        print(f"  - {topic}: {meta['count']} msgs, type: {meta['type']} ({freq_str})")

    if args.summary_only:
        return

    # CSV export
    cart_topic = None
    joint_topic = None
    for t in sorted(info['topics'].keys()):
        if cart_topic is None and re.search(r'CartesianPose', t, re.IGNORECASE):
            cart_topic = t
        if joint_topic is None and re.search(r'JointPosition', t, re.IGNORECASE):
            joint_topic = t
        if cart_topic and joint_topic:
            break

    bag_dir = os.path.dirname(os.path.abspath(bag_path))
    base = os.path.splitext(os.path.basename(bag_path))[0]

    if cart_topic is None:
        print("No CartesianPose topic found; skipping CartesianPose CSV.")
    else:
        out_cart = os.path.join(bag_dir, f"{base}_cartesian_pose.csv")
        print(f"Exporting CartesianPose from topic '{cart_topic}' to '{out_cart}'...")
        write_cartesian_csv(bag_path, cart_topic, out_cart)

    if joint_topic is None:
        print("No JointPosition topic found; skipping JointPosition CSV.")
    else:
        out_joint = os.path.join(bag_dir, f"{base}_joint_positions.csv")
        print(f"Exporting JointPosition from topic '{joint_topic}' to '{out_joint}'...")
        write_joint_csv(bag_path, joint_topic, out_joint)

    if args.topic:
        topics_to_inspect = [args.topic]
    else:
        # only inspect CartesianPose and JointPosition topics by default
        pattern = re.compile(r'CartesianPose|JointPosition', re.IGNORECASE)
        topics_to_inspect = [t for t in sorted(info['topics'].keys()) if pattern.search(t)]
        if not topics_to_inspect:
            print("No CartesianPose or JointPosition topics found to inspect.")
            return

    print('\nAttempting to print first messages (may fail if message defs are missing)...')
    for topic in topics_to_inspect:
        print(f"\n== Topic: {topic} ==")
        try_print_first_messages(bag_path, topic, n=args.first)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
teach_pose.py — capture scan waypoints or delivery poses into scan_config.yaml.

WHY THIS EXISTS
---------------
Hand-guiding (admittance mode via wrist button / webapp) and the ROS 2 driver
(Terminal 1, kortex_bringup) both demand exclusive control of the arm, so they
can't run at the same time. The arm HOLDS position when you let go, so we split
the two phases in time:

    1. Terminal 1 DOWN  -> hand-guide arm to a good pose -> let go (arm holds)
    2. Terminal 1 UP    -> press Enter here -> pose is captured & written
    3. Terminal 1 DOWN  -> hand-guide to next pose -> repeat

This script never needs the driver and admittance mode up simultaneously, needs
no RViz, and needs no Kinova raw SDK. It reads the held pose straight off the
ROS topics the driver publishes once it's up.

TWO MODES
---------
  --profile NAME   capture SCAN WAYPOINTS (7 joint angles, radians) and append
                   them to scan_profiles[NAME] in the YAML. Press Enter once per
                   waypoint; press q+Enter to finish the profile.

  --pose NAME      capture ONE DELIVERY POSE (Cartesian world -> end_effector)
                   and write delivery_poses[NAME] = {xyz, quat}. For this you
                   position the EE where you want it to RELEASE the object.

USAGE
-----
  # add scan waypoints to the 'work_area' profile
  python3 teach_pose.py --profile work_area

  # capture the 'return_zone' delivery drop pose
  python3 teach_pose.py --pose return_zone

  # point at a different config file
  python3 teach_pose.py --profile work_area --config /path/to/scan_config.yaml

WORKFLOW PER POSE
-----------------
  - Bring Terminal 1 (kortex_bringup) UP. For --pose you also need the TF tree,
    i.e. whatever publishes world->end_effector_link (the driver does this).
  - When this script says "Ready", press Enter to capture.
  - It averages a few samples for a clean reading, shows you the values, and
    writes them. Then bring Terminal 1 down and hand-guide to the next pose.

NOTES
-----
  - Joint values are read by NAME (joint_1..joint_7), so the gripper joint in
    /joint_states never contaminates a waypoint.
  - /joint_states is already in your stack's signed-radian convention, so no
    deg->rad / wrapTo360 conversion is needed (that's only for the raw SDK).
  - The YAML is rewritten preserving your existing content; new entries are
    appended under the named profile / pose.
"""

import os
import sys
import time
import math
import argparse
import threading

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros
import tf_transformations as tft


ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3',
              'joint_4', 'joint_5', 'joint_6', 'joint_7']

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'scan_config.yaml')

SAMPLES_TO_AVG = 5          # average this many readings for a clean capture
SAMPLE_TIMEOUT_SEC = 5.0    # give up waiting for data after this long


# ════════════════════════════════════════════════════════════════════════════
# YAML I/O  (append without clobbering existing content)
# ════════════════════════════════════════════════════════════════════════════

def load_yaml(path):
    if os.path.exists(path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
    data.setdefault('scan_profiles', {})
    data.setdefault('delivery_poses', {})
    data.setdefault('actions', {})
    return data


def save_yaml(path, data):
    # Back up once per run so a bad edit is recoverable.
    if os.path.exists(path):
        bak = path + '.bak'
        if not os.path.exists(bak):
            with open(path, 'r') as f:
                original = f.read()
            with open(bak, 'w') as f:
                f.write(original)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def append_waypoint(path, profile, waypoint):
    data = load_yaml(path)
    profiles = data['scan_profiles']
    profiles.setdefault(profile, [])
    profiles[profile].append([round(float(v), 3) for v in waypoint])
    save_yaml(path, data)
    return len(profiles[profile])


def write_pose(path, name, xyz, quat):
    data = load_yaml(path)
    data['delivery_poses'][name] = {
        'xyz':  [round(float(v), 3) for v in xyz],
        'quat': [round(float(v), 4) for v in quat],
    }
    save_yaml(path, data)


# ════════════════════════════════════════════════════════════════════════════
# CAPTURE NODE
# ════════════════════════════════════════════════════════════════════════════

class TeachNode(Node):
    def __init__(self):
        super().__init__('teach_pose')
        self._latest_joints = None     # dict name -> position
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _joint_cb(self, msg):
        if not msg.position:
            return
        self._latest_joints = dict(zip(msg.name, msg.position))

    # ── joints (scan waypoint) ──
    def capture_joints(self):
        """Average a few /joint_states readings; return [j1..j7] or None."""
        readings = []
        deadline = time.time() + SAMPLE_TIMEOUT_SEC
        while len(readings) < SAMPLES_TO_AVG and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._latest_joints is None:
                continue
            try:
                vec = [float(self._latest_joints[n]) for n in ARM_JOINTS]
            except KeyError:
                # arm joints not all present yet (driver still starting)
                continue
            readings.append(vec)
            time.sleep(0.05)
        if not readings:
            return None
        n = len(readings)
        return [sum(r[i] for r in readings) / n for i in range(7)]

    # ── Cartesian (delivery pose) ──
    def capture_pose(self):
        """Look up world->end_effector_link; return (xyz, quat) or None."""
        deadline = time.time() + SAMPLE_TIMEOUT_SEC
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'world', 'end_effector_link', rclpy.time.Time())
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        return None


# ════════════════════════════════════════════════════════════════════════════
# CLI / MAIN LOOP
# ════════════════════════════════════════════════════════════════════════════

def prompt(msg):
    """Blocking input that also lets us print cleanly."""
    try:
        return input(msg)
    except EOFError:
        return 'q'


def run_profile_mode(node, path, profile):
    print(f"\n=== TEACH SCAN PROFILE: '{profile}'  ->  {path} ===")
    print("For each waypoint:")
    print("  1) bring Terminal 1 (kortex_bringup) DOWN")
    print("  2) hand-guide arm to the pose, let go (it holds)")
    print("  3) bring Terminal 1 UP, wait for /joint_states")
    print("  4) press Enter here to capture")
    print("Press  q + Enter  to finish.\n")

    count = 0
    while True:
        ans = prompt(f"[waypoint {count + 1}] Enter to capture, q to finish: ")
        if ans.strip().lower() == 'q':
            break
        print("  capturing... (averaging joint_states)")
        wp = node.capture_joints()
        if wp is None:
            print("  x no joint data — is Terminal 1 up and publishing "
                  "/joint_states? Try again.")
            continue
        total = append_waypoint(path, profile, wp)
        pretty = '[' + ', '.join(f'{v:+.3f}' for v in wp) + ']'
        print(f"  ok captured: {pretty}")
        print(f"  ok profile '{profile}' now has {total} waypoint(s).\n")
        count += 1

    print(f"\nDone. Added {count} waypoint(s) to '{profile}'.")
    if count:
        print("Preview them against fake hardware with waypoint_preview.py "
              "before running on the real arm.")


def run_pose_mode(node, path, name):
    print(f"\n=== TEACH DELIVERY POSE: '{name}'  ->  {path} ===")
    print("Position the END-EFFECTOR where it should RELEASE the object.")
    print("  1) bring Terminal 1 (kortex_bringup) DOWN")
    print("  2) hand-guide the EE to the release pose, let go (it holds)")
    print("  3) bring Terminal 1 UP, wait for the TF tree")
    print("  4) press Enter here to capture (or q to abort)\n")

    while True:
        ans = prompt("Enter to capture this delivery pose, q to abort: ")
        if ans.strip().lower() == 'q':
            print("Aborted — nothing written.")
            return
        print("  capturing... (world -> end_effector_link)")
        result = node.capture_pose()
        if result is None:
            print("  x no TF — is Terminal 1 up and publishing "
                  "world->end_effector_link? Try again.")
            continue
        xyz, quat = result
        write_pose(path, name, xyz, quat)
        print(f"  ok xyz  = [{xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}]")
        print(f"  ok quat = [{quat[0]:+.4f}, {quat[1]:+.4f}, "
              f"{quat[2]:+.4f}, {quat[3]:+.4f}]")
        # Sanity hint: is it roughly top-down? roll~=pi means tool-Z down.
        roll, pitch, yaw = tft.euler_from_quaternion(list(quat))
        downness = abs(abs(roll) - math.pi)
        if downness > 0.35:   # ~20 deg
            print(f"  ! note: orientation is not strongly top-down "
                  f"(roll={math.degrees(roll):+.0f} deg). Delivery uses this quat "
                  f"as-is — fine if intentional.")
        print(f"  ok wrote delivery_poses['{name}'].")
        return


def main():
    ap = argparse.ArgumentParser(description="Teach scan waypoints / delivery poses into scan_config.yaml")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--profile', metavar='NAME',
                   help='capture scan waypoints (joints) into this profile')
    g.add_argument('--pose', metavar='NAME',
                   help='capture one delivery pose (Cartesian) under this name')
    ap.add_argument('--config', default=DEFAULT_CONFIG,
                    help=f'path to scan_config.yaml (default: {DEFAULT_CONFIG})')
    args = ap.parse_args()

    rclpy.init()
    node = TeachNode()
    try:
        if args.profile:
            run_profile_mode(node, args.config, args.profile)
        else:
            run_pose_mode(node, args.config, args.pose)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

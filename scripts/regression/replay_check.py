#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Replay the baseline bag through the failsafe and the localizer and compare (#142).

Nothing reaches a chair: the ROS graph is a private localhost domain that must be
empty at start, /cmd_vel_safety is remapped, and no command topic is played.

Limits, on purpose: the bag has no /velodyne_points_no_ground, so failsafe Layer C
is not exercised; Nav2 and the planner are not run.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from compare import compare
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_ros import TransformBroadcaster
import yaml

HERE = Path(__file__).resolve().parent
# Time from the start of the bag. Nav2 drove 73-814 s; before and after it stood still.
NAV_WINDOW_S = (73.0, 814.0)
LOCALIZER_START_S = 73.0
LOCALIZER_DURATION_S = 240.0
FAILSAFE_RATE = 2.0
# Longer than the node's stop-command period, and short enough to split engagements.
ENGAGEMENT_GAP_S = 0.5
FITNESS_KEY = 'fitness_score'


def bag_start_ns(bag):
    info = yaml.safe_load((Path(bag) / 'metadata.yaml').read_text())
    return info['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']


def spawn(cmd, log):
    # Own process group: `ros2 run` and `ros2 launch` do not always pass signals on.
    return subprocess.Popen(cmd, stdout=open(log, 'w'), stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop(proc):
    if proc.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(wait_s)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


class Harness(Node):
    def __init__(self):
        super().__init__('regression_harness',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])

    def sim_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def spin_in_background(node):
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    thread = threading.Thread(target=ex.spin, daemon=True)
    thread.start()
    return ex


def check_failsafe(args, out):
    node = Harness()
    stamps = []
    node.create_subscription(Twist, '/regression/cmd_vel_safety',
                             lambda _: stamps.append(node.sim_s()), 50)
    ex = spin_in_background(node)
    log = out / 'failsafe_node.log'
    fs = spawn(['ros2', 'run', 'whill_safety', 'failsafe_node', '--ros-args',
                '-p', 'use_sim_time:=true',
                '-r', '/cmd_vel_safety:=/regression/cmd_vel_safety'], log)
    time.sleep(5)
    play = spawn(['ros2', 'bag', 'play', args.bag, '--clock', '100',
                  '--rate', str(FAILSAFE_RATE), '--disable-keyboard-controls',
                  '--topics', '/scan', '/pcl_pose', '/alignment_status'],
                 out / 'failsafe_play.log')
    play.wait()
    time.sleep(2)
    stop(fs)
    ex.shutdown()
    node.destroy_node()
    if play.returncode != 0:
        raise RuntimeError(f'bag play failed: {out}/failsafe_play.log')
    t0 = bag_start_ns(args.bag) * 1e-9
    rel = np.array(sorted(stamps)) - t0
    starts = rel[np.r_[True, np.diff(rel) > ENGAGEMENT_GAP_S]] if len(rel) else rel
    ends = rel[np.r_[np.diff(rel) > ENGAGEMENT_GAP_S, True]] if len(rel) else rel
    in_nav = ((starts >= NAV_WINDOW_S[0]) & (starts <= NAV_WINDOW_S[1])) if len(rel) else rel
    text = log.read_text(errors='replace')
    # The node's own transitions are the engagement count. Stop commands merged by
    # ENGAGEMENT_GAP_S only locate them in time and measure how long they lasted.
    return {'engagements': text.count('failsafe ENGAGED'),
            'engaged_s': round(float((ends - starts).sum()), 2),
            'stops_while_driving': int(np.sum(in_nav))}


def reference_poses(bag, start_s, duration_s):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/pcl_pose']))
    t0 = bag_start_ns(bag)
    lo, hi = t0 + start_s * 1e9, t0 + (start_s + duration_s) * 1e9
    rows, first = [], None
    while reader.has_next():
        _, data, t = reader.read_next()
        if t < lo or t > hi:
            continue
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        first = first or msg
        rows.append(pose_row(msg))
    return np.array(rows), first


def pose_row(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw]


def check_localizer(args, out):
    ref, first = reference_poses(args.bag, LOCALIZER_START_S, LOCALIZER_DURATION_S)
    if first is None:
        raise RuntimeError('no /pcl_pose in the replay window')
    node = Harness()
    poses, fitness = [], []

    def on_status(msg):
        for status in msg.status:
            for kv in status.values:
                if kv.key == FITNESS_KEY:
                    try:
                        fitness.append(float(kv.value))
                    except ValueError:
                        pass

    node.create_subscription(PoseWithCovarianceStamped, '/pcl_pose',
                             lambda m: poses.append(pose_row(m)), qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/alignment_status', on_status,
                             qos_profile_sensor_data)
    # The EKF does not run here. Its recorded output supplies odom->base_link, which
    # the localizer needs to publish map->odom.
    broadcaster = TransformBroadcaster(node)

    def on_odom(msg):
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        broadcaster.sendTransform(tf)

    node.create_subscription(Odometry, '/odometry/filtered', on_odom, 50)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    ex = spin_in_background(node)
    loc = spawn(['ros2', 'launch', str(HERE / 'replay_localizer_launch.py'), 'site:=' + args.site],
                out / 'localizer.log')
    # The map PCD takes several seconds to load.
    time.sleep(20)
    play = spawn(['ros2', 'bag', 'play', args.bag, '--clock', '100',
                  '--start-offset', str(LOCALIZER_START_S), '--disable-keyboard-controls',
                  '--topics', '/velodyne_points', '/odometry/filtered', '/imu/data_rep145'],
                 out / 'localizer_play.log')
    # Stamp from the bag clock: the localizer drops scans older than the initial pose.
    deadline = time.time() + 40
    while not poses and time.time() < deadline:
        if node.sim_s() > 0:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose = first.pose
            init_pub.publish(msg)
        time.sleep(2)
    time.sleep(max(0, LOCALIZER_DURATION_S - 2))
    stop(play)
    stop(loc)
    ex.shutdown()
    node.destroy_node()
    if len(poses) < 2:
        raise RuntimeError(f'the localizer produced no poses: {out}/localizer.log')
    got = np.array(poses)
    # Pair by nearest stamp. A pose the replay never produced counts through `poses`.
    idx = np.clip(np.searchsorted(ref[:, 0], got[:, 0]), 1, len(ref) - 1)
    idx -= (got[:, 0] - ref[idx - 1, 0]) < (ref[idx, 0] - got[:, 0])
    near = np.abs(ref[idx, 0] - got[:, 0]) < 0.2
    xy = np.hypot(*(got[near, 1:3] - ref[idx[near], 1:3]).T)
    dyaw = np.abs(np.arctan2(np.sin(got[near, 4] - ref[idx[near], 4]),
                             np.cos(got[near, 4] - ref[idx[near], 4])))
    fit = np.array(fitness) if fitness else np.array([float('nan')])
    return {'poses': int(len(got)), 'reference_poses': int(len(ref)),
            'max_gap_s': round(float(np.diff(got[:, 0]).max()), 3),
            'xy_err_p50_m': round(float(np.percentile(xy, 50)), 4),
            'xy_err_p95_m': round(float(np.percentile(xy, 95)), 4),
            'xy_err_max_m': round(float(xy.max()), 4),
            'yaw_err_p95_deg': round(math.degrees(float(np.percentile(dyaw, 95))), 3),
            'fitness_p50': round(float(np.nanpercentile(fit, 50)), 4),
            'fitness_p95': round(float(np.nanpercentile(fit, 95)), 4),
            'fitness_max': round(float(np.nanmax(fit)), 4)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--site', default='campus')
    ap.add_argument('--baseline', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--only', choices=['failsafe', 'localizer'])
    ap.add_argument('--no-compare', action='store_true', help='only measure')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    listing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True)
    if listing.stdout.strip():
        sys.exit(f'ROS domain {os.environ.get("ROS_DOMAIN_ID")} is not empty:\n{listing.stdout}')

    rclpy.init()
    result = {}
    if args.only != 'localizer':
        result['failsafe'] = check_failsafe(args, args.out)
    if args.only != 'failsafe':
        result['localizer'] = check_localizer(args, args.out)
    rclpy.shutdown()
    (args.out / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if args.no_compare:
        return 0
    failures = compare(result, json.loads(args.baseline.read_text()))
    for failure in failures:
        print('REGRESSION:', failure)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

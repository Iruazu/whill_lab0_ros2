#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Replay a driving bag through the demo localizer on a chosen map and TF (#155).

The regression check (replay_check.py) asks "does the branch reproduce the
recorded /pcl_pose". A rebuilt map (#153) and a corrected LiDAR TF (#145) are
*meant* to change the pose, so that question no longer applies. This script asks
instead: on this map, with this TF and these clouds, does NDT converge and how
well does it fit (fitness, rejects, pose gaps) — so the old and the new
combination can be measured on the same bag with the same harness.

The recorded /pcl_pose is in the old map frame. With --map-transform (a 4x4
T_new_old, e.g. from map_align.py) it is carried into the new frame twice: as
the initial pose, and as a reference to report how far the new pose is from the
old one (this mixes the map change and the TF change; it is a consistency
figure, not an accuracy figure — neither map is ground truth).

Nothing reaches a chair: private localhost domain that must be empty, no
command topic is played (same harness as replay_check.py).
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from replay_check import (FITNESS_KEY, Harness, bag_start_ns, pose_row,  # noqa: E402
                          spawn, spin_in_background, stop)

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402


def recorded_poses(bag, start_s, duration_s):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/pcl_pose']))
    t0 = bag_start_ns(bag)
    lo = t0 + start_s * 1e9
    hi = t0 + (start_s + duration_s) * 1e9 if duration_s > 0 else float('inf')
    msgs = []
    while reader.has_next():
        _, data, t = reader.read_next()
        if lo <= t <= hi:
            msgs.append(deserialize_message(data, PoseWithCovarianceStamped))
    return msgs


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def transform_pose(msg, T):
    out = PoseWithCovarianceStamped()
    out.header = msg.header
    out.pose.covariance = msg.pose.covariance
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    P = np.eye(4)
    P[:3, :3] = quat_to_R(q)
    P[:3, 3] = [p.x, p.y, p.z]
    N = T @ P
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, N[:3, 3])
    o = out.pose.pose.orientation
    o.x, o.y, o.z, o.w = R_to_quat(N[:3, :3])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--site', required=True)
    ap.add_argument('--maps-root', help='WHILL_MAPS_ROOT for this run (directory holding <site>/)')
    ap.add_argument('--lidar-tf', default='',
                    help='"x y z roll pitch yaw" [m, deg] for base_link->velodyne; empty = branch TF')
    ap.add_argument('--map-transform', type=Path,
                    help='4x4 T_new_old (text) applied to the recorded /pcl_pose')
    ap.add_argument('--start', type=float, default=73.0, help='seconds from bag start')
    ap.add_argument('--duration', type=float, default=240.0)
    ap.add_argument('--out', required=True, type=Path)
    a = ap.parse_args()
    if a.out.exists():
        sys.exit(f'{a.out} exists; pick a new output directory')
    a.out.mkdir(parents=True)
    if a.maps_root:
        os.environ['WHILL_MAPS_ROOT'] = str(Path(a.maps_root).expanduser().resolve())

    import subprocess
    listing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True)
    if listing.stdout.strip():
        sys.exit(f'ROS domain {os.environ.get("ROS_DOMAIN_ID")} is not empty:\n{listing.stdout}')

    T = np.loadtxt(a.map_transform) if a.map_transform else np.eye(4)
    rec = recorded_poses(a.bag, a.start, a.duration)
    if not rec:
        sys.exit('no /pcl_pose in the window')
    ref = np.array([pose_row(transform_pose(m, T)) for m in rec])
    first = transform_pose(rec[0], T)

    rclpy.init()
    node = Harness()
    poses, fitness = [], []

    def on_status(msg):
        for st in msg.status:
            for kv in st.values:
                if kv.key == FITNESS_KEY:
                    try:
                        fitness.append((node.sim_s(), float(kv.value)))
                    except ValueError:
                        pass

    node.create_subscription(PoseWithCovarianceStamped, '/pcl_pose',
                             lambda m: poses.append(pose_row(m)), qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/alignment_status', on_status,
                             qos_profile_sensor_data)
    br = TransformBroadcaster(node)

    def on_odom(msg):
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        br.sendTransform(tf)

    node.create_subscription(Odometry, '/odometry/filtered', on_odom, 50)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    ex = spin_in_background(node)
    cmd = ['ros2', 'launch', str(HERE / 'replay_localizer_launch.py'), 'site:=' + a.site]
    if a.lidar_tf.strip():
        cmd.append('lidar_tf:=' + a.lidar_tf)
    loc = spawn(cmd, a.out / 'localizer.log')
    time.sleep(20)
    play_cmd = ['ros2', 'bag', 'play', a.bag, '--clock', '100', '--start-offset', str(a.start),
                '--disable-keyboard-controls',
                '--topics', '/velodyne_points', '/odometry/filtered', '/imu/data_rep145']
    play = spawn(play_cmd, a.out / 'play.log')
    deadline = time.time() + 40
    while not poses and time.time() < deadline:
        if node.sim_s() > 0:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose = first.pose
            init_pub.publish(msg)
        time.sleep(2)
    if a.duration > 0:
        time.sleep(max(0, a.duration - 2))
        stop(play)
    else:
        play.wait()
        time.sleep(3)
    stop(loc)
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    log = (a.out / 'localizer.log').read_text(errors='replace')
    if len(poses) < 2:
        sys.exit(f'no poses: {a.out}/localizer.log')
    got = np.array(poses)
    np.savetxt(a.out / 'poses.txt', got, header='t x y z yaw')
    fit = np.array([f for _, f in fitness]) if fitness else np.array([np.nan])
    idx = np.clip(np.searchsorted(ref[:, 0], got[:, 0]), 1, len(ref) - 1)
    idx -= (got[:, 0] - ref[idx - 1, 0]) < (ref[idx, 0] - got[:, 0])
    near = np.abs(ref[idx, 0] - got[:, 0]) < 0.2
    xy = np.hypot(*(got[near, 1:3] - ref[idx[near], 1:3]).T)
    dz = got[near, 3] - ref[idx[near], 3]
    dyaw = np.degrees(np.arctan2(np.sin(got[near, 4] - ref[idx[near], 4]),
                                 np.cos(got[near, 4] - ref[idx[near], 4])))
    res = {
        'site': a.site, 'lidar_tf': a.lidar_tf or 'branch static_tf_launch.py',
        'map_transform': str(a.map_transform) if a.map_transform else None,
        'window_s': [a.start, a.duration],
        'poses': int(len(got)), 'recorded_poses': int(len(ref)),
        'max_gap_s': round(float(np.diff(got[:, 0]).max()), 3),
        'fitness_n': int(np.isfinite(fit).sum()),
        'fitness_p50': round(float(np.nanpercentile(fit, 50)), 4),
        'fitness_p95': round(float(np.nanpercentile(fit, 95)), 4),
        'fitness_max': round(float(np.nanmax(fit)), 4),
        # Messages of lidar_localization_component.cpp: the measurement gate
        # warns "The fitness score is over <threshold>." and a failed NDT
        # warns "The registration didn't converge.".
        'fitness_over_threshold_warnings': log.count('The fitness score is over'),
        'not_converged_warnings': log.count("The registration didn't converge"),
        'vs_recorded_xy_p50_m': round(float(np.percentile(xy, 50)), 3),
        'vs_recorded_xy_p95_m': round(float(np.percentile(xy, 95)), 3),
        'vs_recorded_xy_max_m': round(float(xy.max()), 3),
        'vs_recorded_dz_p50_m': round(float(np.percentile(dz, 50)), 3),
        'vs_recorded_dyaw_p50_deg': round(float(np.percentile(dyaw, 50)), 3),
        'vs_recorded_dyaw_p95_abs_deg': round(float(np.percentile(np.abs(dyaw), 95)), 3),
    }
    (a.out / 'result.json').write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()

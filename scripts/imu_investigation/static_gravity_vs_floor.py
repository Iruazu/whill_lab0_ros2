#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Compare IMU gravity direction with the LiDAR floor normal on a static window.

For a window [start, start+dur] (bag-relative seconds) in which the robot is
standing still: IMU 'up' = mean specific force (REP-145: +g up) normalised;
LiDAR 'up' = RANSAC floor-plane normal of the accumulated scans (with/without
Hi-Res re-projection). Then reports the tilt of each sensor and the angle
between R_lidar_imu * up_imu and up_lidar for a given T_lidar_imu quaternion.
"""
import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu, PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lidar_icp_odom import load_points  # noqa: E402
from velodyne_reproject_bag import build_tables  # noqa: E402


def tilt(u):
    # pitch, roll of a frame whose measured 'up' is u (yaw-free ZYX convention)
    return np.degrees(np.arcsin(-u[0])), np.degrees(np.arctan2(u[1], u[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_db3')
    ap.add_argument('--start', type=float, default=2.0)
    ap.add_argument('--dur', type=float, default=25.0)
    ap.add_argument('--src-calib', required=True)
    ap.add_argument('--dst-calib', required=True)
    ap.add_argument('--imu-topic', default='/imu/data_rep145')
    ap.add_argument('--q-lidar-imu', nargs=4, type=float, default=[-0.035606, -0.066319, -0.002368, 0.997163])
    ap.add_argument('--max-range', type=float, default=12.0)
    a = ap.parse_args()
    con = sqlite3.connect(f'file:{a.bag_db3}?mode=ro', uri=True)
    tid = {n: i for i, n in con.execute('select id,name from topics')}
    t0 = con.execute('select min(timestamp) from messages').fetchone()[0]
    lo, hi = t0 + int(a.start * 1e9), t0 + int((a.start + a.dur) * 1e9)
    acc = []
    for _, d in con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ?',
                            (tid[a.imu_topic], lo, hi)):
        m = deserialize_message(d, Imu)
        acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    acc = np.array(acc)
    up_i = acc.mean(0) / np.linalg.norm(acc.mean(0))
    print(f'IMU: n={len(acc)} mean f={acc.mean(0)} |f|={np.linalg.norm(acc.mean(0)):.4f}  up={up_i}  '
          f'pitch={tilt(up_i)[0]:+.2f} roll={tilt(up_i)[1]:+.2f} deg')
    R_li = Rot.from_quat(a.q_lidar_imu).as_matrix()
    tables = build_tables(a.src_calib, a.dst_calib)
    for label, tb in (('original(VLP16db)', None), ('reprojected(Hi-Res)', tables)):
        pts = []
        for k, (_, d) in enumerate(con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ?',
                                               (tid['/velodyne_points'], lo, hi))):
            if k % 5:
                continue
            p, _ = load_points(deserialize_message(d, PointCloud2), tb)
            r = np.hypot(p[:, 0], p[:, 1])
            pts.append(p[(r < a.max_range) & (p[:, 2] < -0.3)])
        p = np.concatenate(pts)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p)).voxel_down_sample(0.05)
        (A, B, C, D), inl = pc.segment_plane(0.04, 3, 2000)
        n = np.array([A, B, C])
        if n[2] < 0:
            n, D = -n, -D
        nn = np.linalg.norm(n)
        n, D = n / nn, D / nn
        q = np.asarray(pc.points)[inl]
        res = q @ n + D
        # per-ring sanity: floor height in front/back/left/right sectors
        print(f'{label}: floor normal (lidar)={n}  height={D:.3f} m  inliers={len(inl)} resid std={res.std():.3f}  '
              f'LiDAR pitch={tilt(n)[0]:+.2f} roll={tilt(n)[1]:+.2f} deg')
        az = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
        for nm, m in (('front', np.abs(az) < 30), ('back', np.abs(az) > 150), ('left', np.abs(az - 90) < 30),
                      ('right', np.abs(az + 90) < 30)):
            if m.sum():
                print(f'    {nm:5s}: median z of inliers = {np.median(q[m, 2]):+.3f} m')
        pred = R_li @ up_i
        ang = np.degrees(np.arccos(np.clip(pred @ n, -1, 1)))
        # rotation needed (2-DoF) from IMU up to LiDAR up
        print(f'    angle(R_lidar_imu(config) * up_imu, up_lidar) = {ang:.2f} deg; '
              f'config predicts LiDAR pitch={tilt(pred)[0]:+.2f} roll={tilt(pred)[1]:+.2f}')


if __name__ == '__main__':
    main()

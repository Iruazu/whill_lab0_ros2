#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""LiDAR-only scan-to-local-map odometry (open3d point-to-plane ICP), IMU-free.

Purpose: an IMU-independent reference motion to test the IMU (gyro/accel axes,
T_lidar_imu rotation, time offset). GLIM's own trajectory cannot be used for
this because its LiDAR pose is IMU pose * fixed T_imu_lidar.

Reads clouds straight from the sqlite bag (read-only), optionally re-projects
Hi-Res elevations with the tables from velodyne_reproject_bag.py, deskews each
scan with the previous constant-velocity estimate using the per-point `time`
field, and registers it to a sliding local map of the last K keyframes.
Output: npz with stamp (header stamp = scan end), T_world_lidar (N,4,4),
fitness, rmse.
"""
import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from velodyne_reproject_bag import build_tables  # noqa: E402

DT = np.dtype({'names': ['x', 'y', 'z', 'ring', 'time'], 'formats': ['<f4', '<f4', '<f4', '<u2', '<f4'],
               'offsets': [0, 4, 8, 16, 18], 'itemsize': 22})


def load_points(msg, tables):
    a = np.frombuffer(bytes(msg.data), DT)[: msg.width * msg.height]
    p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    ok = np.isfinite(p).all(1)
    p, ring, tt = p[ok], a['ring'][ok].astype(int), a['time'][ok].astype(np.float64)
    if tables is not None:
        sv, so, dv, do, _ = tables
        xy = np.hypot(p[:, 0], p[:, 1])
        d = xy * np.cos(sv[ring]) + p[:, 2] * np.sin(sv[ring])
        xy2 = d * np.cos(dv[ring]) - do[ring] * np.sin(dv[ring])
        z2 = d * np.sin(dv[ring]) + do[ring] * np.cos(dv[ring])
        s = xy2 / np.maximum(xy, 1e-6)
        p = np.stack([p[:, 0] * s, p[:, 1] * s, z2], 1)
    r = np.linalg.norm(p, axis=1)
    keep = (r > 1.5) & (r < 60.0)
    return p[keep], tt[keep]


def se3_log(T):
    rv = Rot.from_matrix(T[:3, :3]).as_rotvec()
    return np.r_[rv, T[:3, 3]]


def se3_exp(xi):
    T = np.eye(4)
    T[:3, :3] = Rot.from_rotvec(xi[:3]).as_matrix()
    T[:3, 3] = xi[3:]
    return T


def deskew(p, tt, T_delta, scan_dt):
    """Move every point to the scan-end frame (time 0 = header stamp, tt<=0).
    T_delta is the motion over one scan period in the scan-end frame
    (pose_end^-1 * pose_prev_end); point at time tt is at fraction (tt/scan_dt)."""
    xi = se3_log(np.linalg.inv(T_delta))  # end->start direction
    f = np.clip(-tt / scan_dt, 0, 1.2)
    R = Rot.from_rotvec(f[:, None] * xi[None, :3]).as_matrix()
    return np.einsum('nij,nj->ni', R, p) + f[:, None] * xi[None, 3:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_db3')
    ap.add_argument('out_npz')
    ap.add_argument('--topic', default='/velodyne_points')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--src-calib')
    ap.add_argument('--dst-calib')
    ap.add_argument('--voxel', type=float, default=0.3)
    ap.add_argument('--keyframes', type=int, default=10)
    ap.add_argument('--no-deskew', action='store_true')
    a = ap.parse_args()
    tables = build_tables(a.src_calib, a.dst_calib) if a.src_calib else None
    con = sqlite3.connect(f'file:{a.bag_db3}?mode=ro', uri=True)
    tid = {n: i for i, n in con.execute('select id,name from topics')}
    t0 = con.execute('select min(timestamp) from messages').fetchone()[0]
    lo, hi = t0 + int(a.start * 1e9), t0 + int((a.start + a.duration) * 1e9)
    rows = con.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<=? '
                       'order by timestamp', (tid[a.topic], lo, hi))
    stamps, poses, fit, rmse = [], [], [], []
    T = np.eye(4)
    T_delta = np.eye(4)
    kfs = []  # list of (T_world, pcd_world)
    last_kf_T = None
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
    for k, (t, d) in enumerate(rows):
        msg = deserialize_message(d, PointCloud2)
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, tt = load_points(msg, tables)
        scan_dt = 0.1014
        if not a.no_deskew and k > 0:
            p = deskew(p, tt, T_delta, scan_dt)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p)).voxel_down_sample(a.voxel)
        if not kfs:
            T_new = np.eye(4)
            f_, r_ = 1.0, 0.0
        else:
            target = o3d.geometry.PointCloud()
            for _, kp in kfs:
                target += kp
            if not target.has_normals():
                target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=20))
            init = T @ T_delta  # constant velocity prediction (T_delta in body frame)
            res = o3d.pipelines.registration.registration_icp(
                pc, target, 1.0, init,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(), crit)
            T_new, f_, r_ = res.transformation, res.fitness, res.inlier_rmse
        if k > 0:
            T_delta = np.linalg.inv(T) @ T_new
        T = T_new
        stamps.append(st)
        poses.append(T.copy())
        fit.append(f_)
        rmse.append(r_)
        if last_kf_T is None or np.linalg.norm(np.linalg.inv(last_kf_T)[:3, :3] @ (T[:3, 3] - last_kf_T[:3, 3])) > 1.0 \
                or Rot.from_matrix(np.linalg.inv(last_kf_T)[:3, :3] @ T[:3, :3]).magnitude() > np.radians(10):
            kp = o3d.geometry.PointCloud(pc).transform(T)
            kp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=20))
            kfs.append((T.copy(), kp))
            kfs = kfs[-a.keyframes:]
            last_kf_T = T.copy()
        if k % 500 == 0:
            print(k, f'{st:.2f}', np.round(T[:3, 3], 2), f'fit={f_:.2f} rmse={r_:.3f}', file=sys.stderr, flush=True)
    np.savez(a.out_npz, stamp=np.array(stamps), T=np.array(poses), fitness=np.array(fit), rmse=np.array(rmse),
             reprojected=tables is not None)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check where placed static objects (and the floor) land in base_link.

Used for the avoidance baseline's C1 outdoor static check and as the C2
acceptance test (docs/ja/plans/2026-10-28-avoidance-baseline.md). Each bag
is a ~10 s stationary recording with one object on the chassis centreline;
an empty-scene bag is the background.

Sources compared:
  lidar_current  recorded /velodyne_points + the bag's base_link->velodyne TF
                 (what the running stack sees today)
  lidar_hires    the same cloud re-projected to Puck Hi-Res elevations
                 (exact: ring r -> -10 + 4/3 r deg, VLP16db vert offset
                 removed; matches velodyne_transform_node with
                 VLP16_hires_db.yaml to 0.00 mm) + --lidar-tf
  depth          D435 depth image (per-pixel median over frames) + the bag's
                 base_link->camera_link TF, or --camera-tf

Object points = crop box around the object minus anything within 7 cm of
the empty-scene cloud; LiDAR points must also recur in >= 20 % of frames.

notes CSV columns: bag,object,x_m,height_m[,y_m,half_width_m]
  x_m      object's near face along base_link x (tape from the rear axle)
  height_m true object height

Example:
  scripts/check_object_placement.py --bag-root ~/bags/20261030 \\
      --notes ~/bags/20261030/notes.csv --empty empty_0m_101500 --out /tmp/chk
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

# 2026-10-28 tape + floor-fit values (plan §velodyne_hires).
DEFAULT_LIDAR_TF = (0.42, 0.36, 0.777, 0.24, 0.97, -0.317)
VLP16DB_VERT_OFFSET_M = 0.0112
BG_RADIUS_M = 0.07
VOXEL_M = 0.05
VOXEL_FRAC = 0.2
DEPTH_FRAMES = 90
SOURCES = ('lidar_current', 'lidar_hires', 'depth')


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def tf_from_args(vals):
    x, y, z, r, p, yaw = vals
    return rpy_to_R(*map(math.radians, (r, p, yaw))), np.array([x, y, z])


def to_hires(P):
    """VLP16db-computed points (x, y, z, ring) -> Hi-Res geometry."""
    ring = P[:, 3].astype(int)
    v = np.radians(-15.0 + 2.0 * ring)
    vh = np.radians(-10.0 + (4.0 / 3.0) * ring)
    xy = np.hypot(P[:, 0], P[:, 1])
    # Invert rawdata.cpp: xy = d cos v - o sin v, z = d sin v + o cos v.
    d = xy * np.cos(v) + P[:, 2] * np.sin(v)
    s = d * np.cos(vh) / np.maximum(xy, 1e-9)
    return np.c_[P[:, 0] * s, P[:, 1] * s, d * np.sin(vh), P[:, 3]]


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[
        '/tf_static', '/velodyne_points', '/camera/camera/depth/image_rect_raw',
        '/camera/camera/depth/camera_info']))
    tf, clouds, depth, K = {}, [], [], None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/tf_static':
            for t in deserialize_message(data, TFMessage).transforms:
                q, p = t.transform.rotation, t.transform.translation
                tf[(t.header.frame_id, t.child_frame_id)] = (
                    quat_to_R(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))
        elif topic == '/velodyne_points':
            m = deserialize_message(data, PointCloud2)
            # Structured array (sensor_msgs_py humble); field access is also
            # far faster than iterating tuples for ~25k points per frame.
            a = pc2.read_points(m, ('x', 'y', 'z', 'ring'), skip_nans=True)
            clouds.append(np.c_[a['x'], a['y'], a['z'], a['ring']].astype(float))
        elif topic.endswith('camera_info'):
            if K is None:
                K = np.array(deserialize_message(data, CameraInfo).k).reshape(3, 3)
        elif len(depth) < DEPTH_FRAMES:
            m = deserialize_message(data, Image)
            depth.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width))
    return tf, clouds, depth, K


def chain(tf, *frames):
    R, t = np.eye(3), np.zeros(3)
    for a, b in zip(frames, frames[1:]):
        Ri, ti = tf[(a, b)]
        R, t = R @ Ri, R @ ti + t
    return R, t


def build_sources(bag, args):
    tf, clouds, depth, K = read_bag(bag)
    out = {}
    if clouds:
        R, t = tf[('base_link', 'velodyne')]
        out['lidar_current'] = [np.c_[c[:, :3] @ R.T + t, c[:, 3]] for c in clouds]
        Rh, th = tf_from_args(args.lidar_tf)
        out['lidar_hires'] = [np.c_[h[:, :3] @ Rh.T + th, h[:, 3]]
                              for h in map(to_hires, clouds)]
    if depth and K is not None:
        st = np.stack(depth).astype(float)
        st[st == 0] = np.nan
        dep = np.nanmedian(st, 0) / 1000.0
        v, u = np.nonzero(np.isfinite(dep) & (dep > 0.2))
        z = dep[v, u]
        P = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
        Ro, to = chain(tf, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
        Pl = P @ Ro.T + to
        Rc, tc = (tf_from_args(args.camera_tf) if args.camera_tf
                  else tf[('base_link', 'camera_link')])
        out['depth'] = [np.c_[Pl @ Rc.T + tc, np.full(len(Pl), -1.0)]]
    return out


def persistent(frames):
    """Keep points whose voxel recurs in >= VOXEL_FRAC of the frames."""
    if len(frames) == 1:
        return frames[0]
    allp = np.vstack(frames)
    if not len(allp):
        return allp
    keys = np.floor(allp[:, :3] / VOXEL_M).astype(np.int64)
    fid = np.concatenate([np.full(len(f), i) for i, f in enumerate(frames)])
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    hits = np.zeros(len(uniq), int)
    pairs = np.unique(np.c_[inv, fid], axis=0)
    np.add.at(hits, pairs[:, 0], 1)
    return allp[hits[inv] >= VOXEL_FRAC * len(frames)]


def object_metrics(frames, bg_tree, row):
    x0 = float(row['x_m'])
    y0 = float(row.get('y_m') or 0.0)
    hw = float(row.get('half_width_m') or 0.4)
    h = float(row['height_m'])
    lo, hi = x0 - 0.3, x0 + max(0.4, 1.2 * h)
    crops = []
    for f in frames:
        c = f[(f[:, 0] > lo) & (f[:, 0] < hi) & (np.abs(f[:, 1] - y0) < hw)]
        if len(c):
            c = c[bg_tree.query(c[:, :3])[0] > BG_RADIUS_M]
        crops.append(c)
    P = persistent(crops)
    if len(P) < 5:
        return {'detected': False}
    rec = {
        'detected': True,
        'n_per_frame': round(len(P) / len(frames), 1),
        'x_front': round(float(np.percentile(P[:, 0], 3)), 3),
        'z_max': round(float(np.percentile(P[:, 2], 99)), 3),
        'z_min': round(float(np.percentile(P[:, 2], 1)), 3),
        'y_center': round(float((np.percentile(P[:, 1], 5) + np.percentile(P[:, 1], 95)) / 2), 3),
    }
    rec['dx'] = round(rec['x_front'] - x0, 3)
    rec['dz'] = round(rec['z_max'] - h, 3)
    rec['dy'] = round(rec['y_center'] - y0, 3)
    if P[0, 3] >= 0:
        rings, cnt = np.unique(P[:, 3].astype(int), return_counts=True)
        rec['rings'] = {int(r): round(c / len(frames), 1) for r, c in zip(rings, cnt)}
    return rec, P


def floor_profile(frames, rng=np.random.default_rng(0)):
    """Floor plane (RANSAC, near-horizontal normals only) and its height
    spread vs distance. roll_deg is the road cross slope seen by the sensor;
    z_sigma per bin feeds the minimum obstacle height."""
    P = np.vstack(frames)[:, :3]
    P = P[(np.abs(P[:, 1]) < 1.5) & (P[:, 0] > 0.5) & (P[:, 0] < 8.0)
          & (np.abs(P[:, 2]) < 0.5)]
    if len(P) < 200:
        return {'bins': [], 'plane': None}
    # Fit on the lowest point of each 0.25 m cell (standard ground-candidate
    # step): object and wall points sit above the floor, so cell minima are
    # floor wherever the sensor sees floor at all (LiDAR: beyond ~4 m only).
    cell = np.floor(P[:, :2] / 0.25).astype(np.int64)
    order = np.lexsort((P[:, 2], cell[:, 1], cell[:, 0]))
    _, first = np.unique(cell[order], axis=0, return_index=True)
    G = P[order[first]]
    if len(G) < 20:
        return {'bins': [], 'plane': None}
    best = None
    for _ in range(400):
        a, b, c = G[rng.choice(len(G), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        # Reject walls / object faces: a road surface is within ~15 deg of level.
        if nn < 1e-9 or abs(n[2]) / nn < 0.966:
            continue
        n /= nn
        inl = np.abs((G - a) @ n) < 0.03
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        return {'bins': [], 'plane': None}
    Q = G[best]
    coef = np.linalg.lstsq(np.c_[Q[:, 0], Q[:, 1], np.ones(len(Q))], Q[:, 2], rcond=None)[0]
    plane = {'pitch_deg': round(math.degrees(math.atan(coef[0])), 2),
             'roll_deg': round(math.degrees(math.atan(coef[1])), 2),
             'z0': round(float(coef[2]), 3), 'inlier_ratio': round(best.mean(), 2)}
    # Spread: points within 0.2 m of the fitted plane, measured as base_link z,
    # so both a TF height error (median) and noise (sigma) show up per bin.
    res = P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
    F = P[np.abs(res) < 0.2]
    bins = []
    for x in np.arange(1.0, 8.01, 0.5):
        z = F[np.abs(F[:, 0] - x) < 0.25, 2]
        if len(z) > 30:
            med = float(np.median(z))
            bins.append({'x': float(x), 'z_median': round(med, 3),
                         'z_sigma': round(float(1.4826 * np.median(np.abs(z - med))), 3),
                         'n': int(len(z))})
    return {'bins': bins, 'plane': plane}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag-root', required=True)
    ap.add_argument('--notes', required=True)
    ap.add_argument('--empty', required=True, help='empty-scene bag name')
    ap.add_argument('--out', required=True)
    ap.add_argument('--lidar-tf', nargs=6, type=float, default=DEFAULT_LIDAR_TF,
                    metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='base_link->velodyne_hires, metres and degrees')
    ap.add_argument('--camera-tf', nargs=6, type=float, default=None,
                    metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='override base_link->camera_link (default: bag /tf_static)')
    ap.add_argument('--tol', type=float, default=0.05,
                    help='acceptance tolerance on |dx| and |dz| [m]')
    ap.add_argument('--accept', choices=SOURCES, action='append', default=[],
                    help='exit non-zero if a detected object in this source misses --tol')
    args = ap.parse_args()

    from scipy.spatial import cKDTree
    os.makedirs(args.out, exist_ok=True)
    rows = list(csv.DictReader(open(args.notes)))
    empty = build_sources(os.path.join(args.bag_root, args.empty), args)
    trees = {k: cKDTree(np.vstack(v)[:, :3]) for k, v in empty.items()}
    report = {'floor': {k: floor_profile(v) for k, v in empty.items()}, 'objects': []}
    plots = {}
    failed = []
    for row in rows:
        if row['bag'] == args.empty:
            continue
        src = build_sources(os.path.join(args.bag_root, row['bag']), args)
        res = {'bag': row['bag'], 'object': row['object'],
               'x_m': float(row['x_m']), 'height_m': float(row['height_m'])}
        for k, frames in src.items():
            out = object_metrics(frames, trees[k], row)
            rec, P = out if isinstance(out, tuple) else (out, None)
            res[k] = rec
            if P is not None:
                plots.setdefault(row['object'], []).append((row, k, P))
            if k in args.accept and rec['detected'] \
                    and max(abs(rec['dx']), abs(rec['dz'])) > args.tol:
                failed.append(f"{row['bag']}:{k}")
        report['objects'].append(res)
        print(json.dumps(res, ensure_ascii=False))
    report['accept_failed'] = failed
    json.dump(report, open(os.path.join(args.out, 'results.json'), 'w'),
              ensure_ascii=False, indent=1)
    print(json.dumps({'floor': report['floor']}, ensure_ascii=False))
    _plot(plots, args.out)
    if failed:
        print(f'ACCEPT FAILED (tol {args.tol} m): {failed}', file=sys.stderr)
        sys.exit(1)


def _plot(plots, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    col = {'lidar_current': 'tab:red', 'lidar_hires': 'tab:blue', 'depth': 'tab:green'}
    for obj, items in plots.items():
        bags = sorted({(r['bag'], float(r['x_m']), float(r['height_m'])) for r, _, _ in items},
                      key=lambda b: (b[1], b[0]))
        fig, axs = plt.subplots(1, len(bags), figsize=(4.2 * len(bags), 4.2), squeeze=False)
        for ax, (bag, x0, h) in zip(axs[0], bags):
            for r, k, P in items:
                if r['bag'] != bag:
                    continue
                Q = P[np.random.default_rng(0).choice(len(P), min(len(P), 3000), replace=False)]
                ax.scatter(Q[:, 0], Q[:, 2], s=2, c=col[k], label=k, alpha=0.5)
            ax.axhline(h, ls='--', c='k', lw=1)
            ax.axhline(0, c='gray', lw=0.8)
            ax.axvline(x0, ls=':', c='k', lw=1)
            ax.set(title=f'{bag}\n(x={x0} m)', xlabel='base_link x [m]', ylabel='z [m]',
                   xlim=(x0 - 0.5, x0 + 1.0), ylim=(-1.1, max(1.0, h + 0.3)))
            ax.grid(alpha=0.3)
        handles = {}
        for ax in axs[0]:
            for hdl, lab in zip(*ax.get_legend_handles_labels()):
                handles.setdefault(lab, hdl)
        fig.legend(list(handles.values()), list(handles.keys()), loc='lower center',
                   ncol=3, markerscale=4)
        fig.suptitle(f'{obj}: side view (dashed = true height, dotted = tape)')
        fig.tight_layout(rect=(0, 0.08, 1, 0.95))
        fig.savefig(os.path.join(out, f'side_{obj}.png'), dpi=110)


if __name__ == '__main__':
    main()

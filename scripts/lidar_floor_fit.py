#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""静止 bag の床面から base_link→velodyne の z・roll・pitch を求める。

用途: 校正台帳 §2.1 の LiDAR の高さと姿勢の出典。WHILL を止めて撮った bag
の /velodyne_points から床の平面を当てはめ、その平面が base_link の
z = 0 の水平面になるような z・roll・pitch を出す。yaw は床からは観測できない
ので出さない (--yaw-deg は確認用の変換にだけ使う)。

方法:
  1. 先頭 --frames フレームの /velodyne_points (velodyne 系) を読む。
     記録時の校正表のままの点群と、Hi-Res の校正表へ投影し直した点群の
     両方を処理する。投影し直しは velodyne_reproject_bag.py の
     build_tables / reproject をそのまま使う (bag の補正と同じ式にするため)。
  2. 床の候補: 水平距離 1〜8 m、z < −0.3 m の点を 0.25 m のセルに分け、
     各セルの最低点を取る。
  3. RANSAC (1000 回、しきい値 3 cm、法線は鉛直から 15° 以内) で床の
     インライアを選び、SVD で平面を当てはめる。
  4. 法線 n (上向き) と、原点から平面までの距離 z を出す。

符号の規約: tf2 の RPY と同じく R = Rz(yaw)·Ry(pitch)·Rx(roll)
(固定軸 X→Y→Z、R は velodyne 系の点を base_link 系へ移す)。base_link の
上向き e_z を velodyne 系で表すと n = Rᵀ e_z = (−sin p, sin r cos p,
cos r cos p) なので、pitch = −asin(n_x)、roll = atan2(n_y, n_z)。
確認として、求めた RPY と z でインライアを base_link 系へ移し、床の傾きが
0 になることを表示する。--check で与えた TF でも同じ傾きを表示する。

例:
  python3 scripts/lidar_floor_fit.py \\
      ~/bags/sensor_check_20261028/chk_empty_0m_160726 \\
      --check 0.777 0.24 0.97 -0.317

床の候補が足りない、またはインライアが無いときは終了コード 1。
"""
import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from velodyne_reproject_bag import build_tables, reproject  # noqa: E402

# 床の候補の範囲。1 m 未満は WHILL の車体と、ほぼ真下を見る ring がない
# (Hi-Res の最下 ring −10° では高さ 0.78 m の床に届くのが約 4.4 m 先からで、
# 1 m 未満の点は車体か床以外)。8 m を超えると ring の間隔が粗く、床の
# 起伏の影響が大きくなる。
MIN_RANGE_M = 1.0
MAX_RANGE_M = 8.0
# センサより 0.3 m 以上下だけを候補にする。センサ高 (約 0.78 m) の半分より
# 上には床がないので、壁や物の側面を初めから外せる。
MAX_Z_M = -0.3
# 物や壁の点は床より上にあるので、セルの最低点はほぼ床になる。
CELL_M = 0.25
RANSAC_ITERS = 1000
RANSAC_THRESH_M = 0.03
# 床の法線が鉛直から 15° (cos 0.966) を超える組は壁や物の面とみなして捨てる。
MIN_NORMAL_Z = 0.966
MIN_CANDIDATES = 20


def read_clouds(bag, nmax):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/velodyne_points']))
    msgs = []
    while r.has_next() and len(msgs) < nmax:
        _, data, _ = r.read_next()
        msgs.append(deserialize_message(data, PointCloud2))
    return msgs


def to_xyz(msgs):
    out = []
    for m in msgs:
        a = pc2.read_points(m, ('x', 'y', 'z'), skip_nans=True)
        out.append(np.c_[a['x'], a['y'], a['z']].astype(float))
    return np.vstack(out) if out else np.zeros((0, 3))


def rpy_to_R(r, p, y):
    # tf2 の setRPY と同じ Rz(y)·Ry(p)·Rx(r)
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def floor_candidates(P):
    xy = np.hypot(P[:, 0], P[:, 1])
    P = P[(xy > MIN_RANGE_M) & (xy < MAX_RANGE_M) & (P[:, 2] < MAX_Z_M)]
    cell = np.floor(P[:, :2] / CELL_M).astype(np.int64)
    order = np.lexsort((P[:, 2], cell[:, 1], cell[:, 0]))
    _, first = np.unique(cell[order], axis=0, return_index=True)
    return P[order[first]]


def fit_plane(G, rng):
    best = None
    for _ in range(RANSAC_ITERS):
        a, b, c = G[rng.choice(len(G), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9 or abs(n[2]) / nn < MIN_NORMAL_Z:
            continue
        n /= nn
        inl = np.abs((G - a) @ n) < RANSAC_THRESH_M
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        return None
    Q = G[best]
    c = Q.mean(0)
    _, _, vt = np.linalg.svd(Q - c)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    rms = float(np.sqrt(np.mean(((Q - c) @ n) ** 2)))
    # 平面は n·x = n·c。センサ (原点) は床の上にあるので z = −n·c > 0。
    return n, float(-(n @ c)), Q, rms


def slope_in_base(Q, z, rpy_rad):
    B = Q @ rpy_to_R(*rpy_rad).T + np.array([0.0, 0.0, z])
    a, b, c = np.linalg.lstsq(np.c_[B[:, 0], B[:, 1], np.ones(len(B))],
                              B[:, 2], rcond=None)[0]
    return math.degrees(math.atan(a)), math.degrees(math.atan(b)), c


def main():
    params = os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params')
    ap = argparse.ArgumentParser(
        description='静止 bag の床面から base_link→velodyne の z・roll・pitch を求める')
    ap.add_argument('bag', help='rosbag2 のディレクトリ (/velodyne_points を含む静止 bag)')
    ap.add_argument('--frames', type=int, default=100,
                    help='先頭から使うフレーム数 (既定 100 = 約 10 秒)')
    ap.add_argument('--src-calib', default=os.path.join(params, 'VLP16db.yaml'),
                    help='記録時に driver が使った校正表 (既定 VLP16db.yaml)')
    ap.add_argument('--dst-calib', default=os.path.join(params, 'VLP16_hires_db.yaml'),
                    help='投影し直す先の校正表 (既定 VLP16_hires_db.yaml)')
    ap.add_argument('--yaw-deg', type=float, default=-0.317,
                    help='確認用の変換に使う yaw [deg] (床からは求まらない)')
    ap.add_argument('--check', nargs=4, type=float, metavar=('Z', 'ROLL', 'PITCH', 'YAW'),
                    help='この TF (m, deg) で床を base_link へ移したときの傾きも表示する')
    ap.add_argument('--seed', type=int, default=0, help='RANSAC の乱数の種 (再現用)')
    a = ap.parse_args()

    msgs = read_clouds(a.bag, a.frames)
    if not msgs:
        sys.exit(f'{a.bag}: /velodyne_points がない')
    tables = build_tables(a.src_calib, a.dst_calib)
    raw = to_xyz(msgs)
    # reproject() は msg.data を書き換えるので、記録時の点群を読んだ後に行う
    stats = {}
    hires = to_xyz([reproject(m, tables, 'ring', stats) for m in msgs])
    print(f'bag={a.bag} frames={len(msgs)} points={len(raw)} '
          f'reproject_zcheck_max={stats.get("max_zchk", float("nan")):.2e} m')

    ok = True
    for label, P in (('記録時 (' + os.path.basename(a.src_calib) + ')', raw),
                     ('投影後 (' + os.path.basename(a.dst_calib) + ')', hires)):
        G = floor_candidates(P)
        res = fit_plane(G, np.random.default_rng(a.seed)) if len(G) >= MIN_CANDIDATES else None
        if res is None:
            print(f'[{label}] 床の候補 {len(G)} 点: 平面が求まらない')
            ok = False
            continue
        n, z, Q, rms = res
        pitch, roll = -math.asin(n[0]), math.atan2(n[1], n[2])
        print(f'[{label}] candidates={len(G)} inliers={len(Q)} rms={rms * 1000:.1f} mm '
              f'z={z:.3f} m roll={math.degrees(roll):+.2f} deg pitch={math.degrees(pitch):+.2f} deg')
        sx, sy, z0 = slope_in_base(Q, z, (roll, pitch, math.radians(a.yaw_deg)))
        print(f'    確認 (この RPY で base_link へ): 床の傾き x={sx:+.3f} y={sy:+.3f} deg z0={z0:+.4f} m')
        if a.check:
            cz, *crpy = a.check
            sx, sy, z0 = slope_in_base(Q, cz, tuple(map(math.radians, crpy)))
            print(f'    --check {a.check}: 床の傾き x={sx:+.3f} y={sy:+.3f} deg z0={z0:+.4f} m')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""置いた静止物（と床）が base_link のどこに出るかを確かめる。

用途: 屋外の静止確認（#149）と、M8-4 の取付検証（計画書
docs/plans/2026-10-28-avoidance-baseline.md §4.2・§6）。各 bag は物を 1 つ車体の
中心線上に置いて止まったまま約 10 秒撮ったもの。何も置かない bag を背景にする。
bag は scripts/record_avoidance_bag.sh --profile static で ~/bags/<run>/ に撮る。

比べるデータ源:
  lidar  /velodyne_points を Hi-Res の仰角表の点群にして、--lidar-tf で base_link へ移す。
         記録時の校正表は ring ごとの仰角から自動で判定する（--calibration）。
         旧 VLP16db.yaml で撮った bag（2026-11-12 以前）は、velodyne_reproject_bag.py
         と同じ式で Hi-Res に投影し直す（ノードの出力と一致する厳密な逆変換）。
  depth  D435 の深度画像（画素ごとにフレームの中央値）を、bag の
         base_link->camera_link（または --camera-tf）で base_link へ移す。

物の点 = 物の周りの箱で切り出し、空の bag の点群から 7 cm 以内の点を除いたもの。
LiDAR の点はさらに、フレームの 20 % 以上で同じボクセルに出るものだけを残す。

notes.csv の列: bag,object,x_m,height_m[,y_m,half_width_m]
  bag      --bag-root からの bag ディレクトリ名
  x_m      物の手前の面の base_link x（後輪車軸の中点からテープで測る）
  height_m 物の本当の高さ

例:
  scripts/check_object_placement.py --bag-root ~/bags/2026-11-20-static \\
      --notes ~/bags/2026-11-20-static/notes.csv --empty empty_first_101500 \\
      --out ~/bags/2026-11-20-static/check

出力: <out>/results.json、側面図 <out>/side_<object>.png。
--accept で指定したデータ源で |dx| か |dz| が --tol を超えた物があれば終了コード 1。
"""

import argparse
import csv
import json
import math
import os
import sys
import warnings
from pathlib import Path

import numpy as np

# base_link -> velodyne の既定値。正本は docs/calibration-ledger.md §2.1
# （2026-11-13 のテープ固定後の実機静止確認 C）。launch の
# src/whill_sensors_bringup/launch/static_tf_launch.py の static_tf_velodyne と同じ値で、
# scripts/tests/test_check_object_placement.py が一致を確かめる。台帳を更新したら
# ここも同じコミットで変える。yaw は未測定（noetic の流用値）。
LEDGER_LIDAR_TF = (0.42, 0.36, 0.771, -0.05, 0.40, -0.317)
# 仰角の範囲で校正表を見分ける。VLP16db は ±15°（幅 30°）、Hi-Res は ±10°（幅 20°）。
# 中間の 25° を境にする。床や物までの距離は 1 m 以上あり、vert_offset（最大 11 mm）
# による見かけの仰角のずれは 1° 未満なので、境界から十分離れる。
CALIB_SPAN_SPLIT_DEG = 25.0
BG_RADIUS_M = 0.07
VOXEL_M = 0.05
VOXEL_FRAC = 0.2
DEPTH_FRAMES = 90
SOURCES = ('lidar', 'depth')
# tf と比べる許容。tf2 の float と度・ラジアン変換の丸めを吸収し、取付の変更は拾う。
TF_MATCH_M = 1e-3
TF_MATCH_DEG = 0.01


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_R(r, p, y):
    """tf2 の RPY（固定軸 X→Y→Z、R = Rz·Ry·Rx）。"""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def tf_from_args(vals):
    """(x, y, z, roll°, pitch°, yaw°) -> (R, t)。"""
    x, y, z, r, p, yaw = vals
    return rpy_to_R(*map(math.radians, (r, p, yaw))), np.array([x, y, z], float)


def tf_differs(a, b):
    """2 つの (R, t) が許容を超えて違えば、その説明を返す。同じなら None。"""
    (Ra, ta), (Rb, tb) = a, b
    dt = float(np.max(np.abs(ta - tb)))
    cosang = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    dang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
    if dt > TF_MATCH_M or dang > TF_MATCH_DEG:
        return f'並進の差 {dt * 1000:.1f} mm、回転の差 {dang:.3f}°'
    return None


def ring_elevation_span_deg(clouds):
    """velodyne 系の点群 (x, y, z, ring) から、最上と最下の ring の仰角の差 [°] を返す。"""
    P = np.vstack(clouds)
    P = P[np.hypot(P[:, 0], P[:, 1]) > 1.0]
    if not len(P):
        return None
    ring = P[:, 3].astype(int)
    elev = np.degrees(np.arctan2(P[:, 2], np.hypot(P[:, 0], P[:, 1])))
    lo, hi = ring.min(), ring.max()
    if hi - lo < 8:
        # 上下の ring が揃わないと幅が比べられない（空に向いた ring は点が出ないことがある）。
        return None
    # ring の番号は仰角の昇順（velodyne_pointcloud calibration.cpp）。端の ring の
    # 中央値の差を、ring 1 本あたりに直して 16 本分に伸ばす。上の ring が空で欠けても判定できる。
    per_ring = (np.median(elev[ring == hi]) - np.median(elev[ring == lo])) / (hi - lo)
    return float(per_ring * 15)


def detect_calibration(clouds):
    span = ring_elevation_span_deg(clouds)
    if span is None:
        return None
    return 'vlp16db' if span > CALIB_SPAN_SPLIT_DEG else 'hires'


def reproject_points(P, tables):
    """(x, y, z, ring) を別の仰角表へ投影し直す。

    式と tables は velodyne_reproject_bag.py の build_tables / reproject と同じ
    （bag の補正 #152 と同じ結果にするため）。PointCloud2 ではなく配列を扱う版。
    """
    sv, so, dv, do, ring_map = tables
    r = P[:, 3].astype(np.int64)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    xy = np.hypot(x, y)
    v, v2, o2 = sv[r], dv[r], do[r]
    d = xy * np.cos(v) + z * np.sin(v)  # vert_offset の項は打ち消し合う
    xy2 = d * np.cos(v2) - o2 * np.sin(v2)
    z2 = d * np.sin(v2) + o2 * np.cos(v2)
    with np.errstate(invalid='ignore', divide='ignore'):
        s = np.where(xy > 1e-6, xy2 / xy, 0.0)
    return np.c_[x * s, y * s, z2, ring_map[r]]


def hires_tables():
    """VLP16db.yaml -> VLP16_hires_db.yaml の表。ROS の環境が要る。"""
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from velodyne_reproject_bag import build_tables
    params = Path(get_package_share_directory('velodyne_pointcloud')) / 'params'
    return build_tables(str(params / 'VLP16db.yaml'), str(params / 'VLP16_hires_db.yaml'))


def read_bag(path):
    # ROS の import はここだけにする。純粋な計算の部分を ROS なしでテストできるようにするため。
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from tf2_msgs.msg import TFMessage

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
            # 投影し直しと TF の適用は velodyne 系の点が前提。
            if m.header.frame_id != 'velodyne':
                sys.exit(f'{path}: /velodyne_points の frame_id={m.header.frame_id!r}（velodyne 以外）')
            # 構造化配列で読む（humble の sensor_msgs_py）。1 フレーム約 2.5 万点を
            # タプルで回すより大幅に速い。
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


def lidar_to_base(clouds, calibration, lidar_tf, tables=None):
    """velodyne 系の点群を Hi-Res にそろえ、base_link 系へ移す。"""
    if calibration == 'vlp16db':
        clouds = [reproject_points(c, tables) for c in clouds]
    R, t = lidar_tf
    return [np.c_[c[:, :3] @ R.T + t, c[:, 3]] for c in clouds]


def build_sources(bag, args, state):
    tf, clouds, depth, K = read_bag(bag)
    out = {}
    if clouds:
        calib = args.calibration
        if calib == 'auto':
            calib = detect_calibration(clouds)
            if calib is None:
                sys.exit(f'{bag}: 校正表を判定できない。--calibration hires|vlp16db で指定する')
        state.setdefault('calibration', {})[os.path.basename(bag)] = calib
        tables = None
        if calib == 'vlp16db':
            if 'tables' not in state:
                state['tables'] = hires_tables()
            tables = state['tables']
        lidar_tf = tf_from_args(args.lidar_tf)
        bag_tf = tf.get(('base_link', 'velodyne'))
        if bag_tf is not None:
            diff = tf_differs(lidar_tf, bag_tf)
            if diff:
                # 旧 TF で撮った bag は違って当然。--lidar-tf の値を使い、差だけ知らせる。
                print(f'NOTE: {os.path.basename(bag)} の /tf_static の base_link->velodyne は '
                      f'--lidar-tf と違う（{diff}）。--lidar-tf の値で計算する', file=sys.stderr)
        out['lidar'] = lidar_to_base(clouds, calib, lidar_tf, tables)
    if depth and K is not None:
        st = np.stack(depth).astype(float)
        st[st == 0] = np.nan
        with warnings.catch_warnings():
            # 全フレームで深度が無い画素（空・近すぎる所）は NaN のまま捨てる。警告は不要。
            warnings.simplefilter('ignore', RuntimeWarning)
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
    """フレームの VOXEL_FRAC 以上で同じボクセルに出る点だけを残す（一瞬のノイズを除く）。"""
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
    """物の位置と高さの誤差を返す。戻り値は (rec, 物の点 or None)。"""
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
        return {'detected': False}, None
    rec = {
        'detected': True,
        'n_per_frame': round(len(P) / len(frames), 1),
        # 外れ値 1 点で前端や上端が決まらないよう、端は百分位で取る。
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


def floor_profile(frames, seed=0):
    """床の平面（RANSAC、水平に近い法線だけ）と、距離ごとの高さのばらつき。

    roll_deg はセンサから見た道路の横断勾配。bin ごとの z_sigma は、障害物とみなす
    最低の高さ（3σ + TF の誤差）の材料になる。乱数は呼び出しごとに seed から作る
    （既定引数で共有すると、bag を処理する順で結果が変わるため）。
    """
    rng = np.random.default_rng(seed)
    P = np.vstack(frames)[:, :3]
    P = P[(np.abs(P[:, 1]) < 1.5) & (P[:, 0] > 0.5) & (P[:, 0] < 8.0)
          & (np.abs(P[:, 2]) < 0.5)]
    if len(P) < 200:
        return {'bins': [], 'plane': None}
    # 0.25 m セルの最低点で当てはめる（地面候補の定石）。物や壁の点は床より上に
    # あるので、床が見えるセルでは最低点が床になる（LiDAR は約 4 m より先だけ）。
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
        # 壁や物の面を除く。道路の面は水平から約 15° 以内。
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
             'z0': round(float(coef[2]), 3), 'inlier_ratio': round(float(best.mean()), 2)}
    # ばらつき: 平面から 0.2 m 以内の点を base_link の z で見る。TF の高さの誤差
    # （中央値）とノイズ（σ）の両方が bin ごとに見える。
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


def accept_failures(results, accept, tol):
    """--accept のデータ源で、検出したのに誤差が tol を超えた物の一覧。"""
    failed = []
    for res in results:
        for k in accept:
            rec = res.get(k)
            if rec and rec['detected'] and max(abs(rec['dx']), abs(rec['dz'])) > tol:
                failed.append(f"{res['bag']}:{k}")
    return failed


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag-root', required=True, help='~/bags/<run>（docs/bags.md）')
    ap.add_argument('--notes', required=True)
    ap.add_argument('--empty', required=True, help='何も置かない bag の名前')
    ap.add_argument('--out', required=True)
    ap.add_argument('--lidar-tf', nargs=6, type=float, default=LEDGER_LIDAR_TF,
                    metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='base_link->velodyne（m と度）。既定は校正台帳 §2.1 の値。'
                         'bag の /tf_static と違えば知らせる')
    ap.add_argument('--calibration', choices=('auto', 'hires', 'vlp16db'), default='auto',
                    help='記録時の Velodyne の校正表。auto は ring の仰角から判定する')
    ap.add_argument('--camera-tf', nargs=6, type=float, default=None,
                    metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='base_link->camera_link の上書き（既定: bag の /tf_static）')
    ap.add_argument('--tol', type=float, default=0.05,
                    help='|dx| と |dz| の許容 [m]（計画書 §6 の ±5 cm）')
    ap.add_argument('--accept', choices=SOURCES, action='append', default=[],
                    help='このデータ源で検出した物が --tol を外れたら終了コード 1')
    return ap.parse_args(argv)


def main():
    args = parse_args()
    from scipy.spatial import cKDTree
    os.makedirs(args.out, exist_ok=True)
    with open(args.notes) as f:
        rows = list(csv.DictReader(f))
    state = {}
    empty = build_sources(os.path.join(args.bag_root, args.empty), args, state)
    trees = {k: cKDTree(np.vstack(v)[:, :3]) for k, v in empty.items()}
    report = {'lidar_tf': list(args.lidar_tf),
              'floor': {k: floor_profile(v) for k, v in empty.items()}, 'objects': []}
    plots = {}
    for row in rows:
        if row['bag'] == args.empty:
            continue
        src = build_sources(os.path.join(args.bag_root, row['bag']), args, state)
        res = {'bag': row['bag'], 'object': row['object'],
               'x_m': float(row['x_m']), 'height_m': float(row['height_m'])}
        for k, frames in src.items():
            if k not in trees:
                # 背景の bag にこのデータ源が無いと、物と背景を分けられない。
                res[k] = {'detected': False, 'error': 'empty bag にこのデータ源が無い'}
                continue
            rec, P = object_metrics(frames, trees[k], row)
            res[k] = rec
            if P is not None:
                plots.setdefault(row['object'], []).append((row, k, P))
        report['objects'].append(res)
        print(json.dumps(res, ensure_ascii=False))
    report['calibration'] = state.get('calibration', {})
    failed = accept_failures(report['objects'], args.accept, args.tol)
    report['accept_failed'] = failed
    with open(os.path.join(args.out, 'results.json'), 'w') as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print(json.dumps({'floor': report['floor'], 'calibration': report['calibration']},
                     ensure_ascii=False))
    _plot(plots, args.out)
    if failed:
        print(f'ACCEPT FAILED (tol {args.tol} m): {failed}', file=sys.stderr)
        sys.exit(1)


def _plot(plots, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    col = {'lidar': 'tab:blue', 'depth': 'tab:green'}
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
                   ncol=2, markerscale=4)
        fig.suptitle(f'{obj}: side view (dashed = true height, dotted = tape)')
        fig.tight_layout(rect=(0, 0.08, 1, 0.95))
        fig.savefig(os.path.join(out, f'side_{obj}.png'), dpi=110)
        plt.close(fig)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_ground_raster_from_bag.py — bag の各スキャンを GLIM の軌跡で直接重ね、密な地面の高さの面を作る（#202）。

縁石の抽出（map_roadway_from_curbs.py）は点の密度で頭打ちになっていた。GLIM のサブマップは
0.1 m で間引かれる（config_sub_mapping_gpu.json の submap_downsample_resolution）ので、
DUFOMap の元の点（202b）でも 5 cm のセルに平均 1.4 点しかない。ここでは GLIM を回し直さず、
補正済みの bag（bag-hires）の点を、同じ GLIM の実行の軌跡（traj_lidar.txt）でそのまま地図の
座標に置く。GLIM を回し直すと GPU の非決定性で地図が正本と少し変わる（campus-m8 README §2）が、
この方法なら正本の軌跡と static.pcd に揃ったままになる。

フレームと時刻（2026-11-16 に glim / glim_ros2 のソースと bag で確認）:
  - traj_lidar.txt は TUM 形式（t x y z qx qy qz qw）で、GLIM の world（= static.pcd・v2 の格子の
    map 座標）での LiDAR（velodyne フレーム）の姿勢 T_world_lidar。大域最適化の後の値。
  - bag の /velodyne_points は header.stamp がスキャンの最後の点の時刻で、点ごとの time 欄は
    header からの相対（−0.100〜+0.0013 s）。GLIM の TimeKeeper は time 欄が負のとき
    frame の時刻を header + min(time)（最初の点の時刻）にずらす（autoconf_prefer_frame_time=false）。
    したがって traj の時刻は各スキャンの最初の点の時刻で、点の絶対時刻は header + time。
  - 点の姿勢は、点の絶対時刻で軌跡を補間して求める（並進は線形、回転は四元数の slerp）。
    スキャン内を --time-bins 個の時間の区切りに分け、区切りの中央の時刻の姿勢を使う
    （点ごとの slerp を避ける近似。0.1 s / 50 = 2 ms で、1 m/s なら 2 mm、30°/s の旋回でも
    15 m 先で 1.6 cm）。軌跡の両端の外、または軌跡の間隔が --max-traj-gap を超える所の点は捨てる。
    GLIM は IMU で歪みを補正するが、ここでは隣り合う姿勢の間を等速とみなす。

手順:
  1. 点を地図に置き、--min-range〜--max-range（センサからの 3 次元の距離）で、センサより
     --max-above-sensor 以上高くない点だけを残す。さらに、正本の DUFOMap の static.pcd の
     0.1 m ボクセル（中心の集合、--static-dilate だけ太らせる）に入る点だけを残す。地図と同じ判定で
     動的な物（人・車）を除くため。
     残った点のセル番号・z・強度を --chunk-scans ごとに --work-dir/cache/ に書く（再開用）。
  2. キャッシュから、5 cm のセル（v2 の格子）ごとに点数・最小・最大を数える。
  3. もう一度キャッシュを読み、セルの最小から --hist-bins × --hist-bin の範囲の z の度数分布
     （uint16、点のあるセルだけ）と、最小から --intensity-band 以内の点の強度の和を数える。
     度数分布から下位 --percentile を（ビンの中で一様とみなして）求める。範囲より上の点は
     「範囲の上」として順位に数える。
  4. npz（float32 の面と点数）と確認画像を書く。

出力:
  --work-dir/ground_raster.npz : count, count_low, z_min, z_p, z_max, intensity_mean, intensity_count
                                 と格子・パラメータ（map_roadway_from_curbs.py --ground-npz で読む）
  --work-dir/count_log.png     : 点数（対数）の原寸の画像
  --preview-dir/               : 1/4 の点数・強度の図と、発進点まわりの原寸の強度（Git 管理）
"""

import argparse
import hashlib
import json
import pathlib
import resource
import sqlite3
import struct
import sys
import time

import numpy as np
import yaml
from PIL import Image

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'
BENCH = pathlib.Path.home() / 'bags/m5r-bench-data/2026-07-10-campus-outer-final'

# map_roadway_from_curbs.py の REGIONS['start_road'] と同じ箱。強度の原寸の切り出しに使う。
START_BOX = (-10.0, 70.0, -15.0, 15.0)

# キャッシュの中身を変える引数。変えたら作り直しが要るので、ハッシュを meta.json に残して照合する。
CACHE_KEYS = ('bag', 'topic', 'traj', 'static_pcd', 'static_voxel', 'static_dilate', 'layers_yaml',
              'min_range', 'max_range', 'max_above_sensor', 'time_bins', 'max_traj_gap',
              'chunk_scans', 'max_scans')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bag', type=pathlib.Path, default=BENCH / 'bag-hires',
                   help='sqlite3 の bag（ディレクトリか .db3）。読むだけ。')
    p.add_argument('--topic', default='/velodyne_points')
    p.add_argument('--traj', type=pathlib.Path, default=D / 'traj_lidar.txt',
                   help='同じ GLIM の実行の T_world_lidar（TUM）。')
    p.add_argument('--static-pcd', type=pathlib.Path,
                   default=BENCH / 'dufomap-m8-hires-noise-local/static.pcd',
                   help='正本の DUFOMap の出力（0.1 m のボクセルの中心）。読むだけ。')
    p.add_argument('--static-voxel', type=float, default=0.1,
                   help='static.pcd のボクセルの大きさ [m]。中心が 0.05 + 0.1k なので floor(x/0.1) が添字。')
    p.add_argument('--static-dilate', type=int, default=1,
                   help='静的ボクセルの集合を各軸この数のボクセルだけ太らせてから引く。0 で太らせない。'
                        '0 だと、センサより 0.5 m 以上低い点（ほぼ地面）の 34%% が落ちる。落ちた点の 6 割は'
                        '上下 ±1 m に静的ボクセルが 1 つも無い柱にあった（GLIM のサブマップは 0.1 m の'
                        '間引きの後に submap_target_num_points 5 万点へ無作為に減らすので、地面のボクセルが'
                        '抜けた穴になる）。1 なら地面の点の 99.6%% が残り、地面より上の点は 1.4%% が落ちる'
                        '（静的な物から 10 cm より離れた所の点、つまり人・動く車の胴体）。2026-11-16 に'
                        'スキャン 95 個（全行程から 200 個おき）で測った。足元の 10 cm は戻るが、'
                        '地面の高さは下位百分位で取るので効かない。')
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml',
                   help='格子（origin・解像度・大きさ）。手塗りと同じ格子に揃える。')
    p.add_argument('--work-dir', type=pathlib.Path,
                   default=pathlib.Path.home() / 'bags/m8-experiments/202c',
                   help='キャッシュと npz（大きい。Git 管理外）。')
    p.add_argument('--preview-dir', type=pathlib.Path, default=D / 'v2/experiments/202c',
                   help='1/4 の確認画像（Git 管理）。')
    p.add_argument('--min-range', type=float, default=0.5,
                   help='センサからの最小距離 [m]。GLIM の distance_near_thresh と同じ（車体・乗員の点を除く）。')
    p.add_argument('--max-range', type=float, default=15.0,
                   help='センサからの最大距離 [m]。遠い点はリングの間隔が開き、入射角が浅く高さの'
                        '誤差（姿勢の角度の誤差 × 距離）が大きいので、近い点だけで面を作る。')
    p.add_argument('--max-above-sensor', type=float, default=3.0,
                   help='センサよりこれ以上高い点 [m] を捨てる（木の枝・壁の上部。地面には要らない）。')
    p.add_argument('--time-bins', type=int, default=50,
                   help='スキャン内の時間の区切りの数（区切りごとに 1 つの姿勢を使う）。')
    p.add_argument('--max-traj-gap', type=float, default=0.25,
                   help='軌跡の隣り合う姿勢の間隔がこれ [s] を超える所は補間しない（GLIM が落とした'
                        'フレームの間）。10 Hz なら通常 0.1 s。')
    p.add_argument('--chunk-scans', type=int, default=500,
                   help='キャッシュの 1 ファイルのスキャン数。中断してもこの単位で再開する。')
    p.add_argument('--hist-bin', type=float, default=0.01,
                   help='z の度数分布のビン幅 [m]。縁石（5〜25 cm）より十分細かい。')
    p.add_argument('--hist-bins', type=int, default=64,
                   help='ビンの数。範囲はセルの最小から hist-bin × hist-bins（既定 0.64 m）。'
                        'map_roadway_from_curbs の ground-band 0.5 m を覆う。')
    p.add_argument('--percentile', type=float, default=5.0,
                   help='地面高さに使う下位百分位 [%%]。v2_layers.yaml の ground_percentile と同じ。')
    p.add_argument('--intensity-band', type=float, default=0.10,
                   help='強度の平均に使う点の、セルの最小からの高さ [m]。地面の点だけの強度にする。')
    p.add_argument('--max-scans', type=int, default=0,
                   help='先頭からこの数のスキャンだけ使う（試し用。0 で全部）。')
    p.add_argument('--force', action='store_true',
                   help='キャッシュの引数が違っても消して作り直す。')
    return p.parse_args(argv)


# ---------------------------------------------------------------- 姿勢の補間

def load_traj(path):
    """TUM（t x y z qx qy qz qw）を読み、時刻順の (t, p[N,3], q[N,4]) を返す。"""
    a = np.loadtxt(path, comments='#')
    a = a[np.argsort(a[:, 0], kind='stable')]
    q = a[:, 4:8]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return a[:, 0], a[:, 1:4], q


def quat_to_rot(q):
    """(N,4) の xyzw を (N,3,3) の回転行列にする。"""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def interp_poses(tt, tp, tq, t, max_gap):
    """時刻 t（1 次元）の姿勢を軌跡 (tt, tp, tq) から補間する。

    並進は線形、回転は slerp。t が軌跡の両端の外か、挟む 2 姿勢の間隔が max_gap を超えるときは
    valid=False（外挿しない。GLIM が姿勢を出していない区間の点は使わない）。
    返り値: (p[M,3], R[M,3,3], valid[M])
    """
    t = np.asarray(t, np.float64)
    i = np.searchsorted(tt, t, side='right') - 1
    valid = (t >= tt[0]) & (t <= tt[-1])
    i = np.clip(i, 0, len(tt) - 2)
    t0, t1 = tt[i], tt[i + 1]
    # 姿勢そのものの時刻（間隔の端）は、隣の間隔が広くても補間が要らないので使う。
    valid &= ((t1 - t0) <= max_gap) | (t == t0) | (t == t1)
    a = np.clip((t - t0) / np.maximum(t1 - t0, 1e-12), 0.0, 1.0)
    p = tp[i] + (tp[i + 1] - tp[i]) * a[:, None]
    q0, q1 = tq[i], tq[i + 1].copy()
    d = np.sum(q0 * q1, axis=1)
    q1[d < 0] *= -1          # 短い方の弧を通る
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    s = np.sin(th)
    small = s < 1e-9
    w0 = np.where(small, 1 - a, np.sin((1 - a) * th) / np.where(small, 1, s))
    w1 = np.where(small, a, np.sin(a * th) / np.where(small, 1, s))
    q = q0 * w0[:, None] + q1 * w1[:, None]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return p, quat_to_rot(q), valid


def transform_scan(xyz, rel_t, stamp, traj, n_bins, max_gap):
    """1 スキャンの点（センサ座標）を地図の座標へ。

    rel_t は header.stamp からの相対時刻。区切りの中央の時刻の姿勢を区切りの全点に使う。
    返り値: (xyz_map[N,3], sensor_z[N], ok[N])  ok=False は軌跡の外の点。
    """
    tt, tp, tq = traj
    lo, hi = float(rel_t.min()), float(rel_t.max())
    bw = max((hi - lo) / n_bins, 1e-9)
    b = np.clip(((rel_t - lo) / bw).astype(np.int64), 0, n_bins - 1)
    centers = stamp + lo + (np.arange(n_bins) + 0.5) * bw
    p, R, valid = interp_poses(tt, tp, tq, centers, max_gap)
    out = np.einsum('nij,nj->ni', R[b], xyz) + p[b]
    return out, p[b, 2], valid[b]


# ---------------------------------------------------------------- 静的ボクセル

class VoxelSet:
    """static.pcd のボクセルの中心を整数の添字の集合にし、点が入っているかを引く。

    dilate > 0 なら、集合を各軸 ±dilate ボクセルの立方体で太らせる（分離して 1 軸ずつ足す。
    27 通りを一度に並べるより中間の配列が小さい）。
    """

    def __init__(self, xyz, voxel, dilate=0):
        self.voxel = voxel
        ijk = np.floor(np.asarray(xyz, np.float64) / voxel).astype(np.int64)
        self.lo = ijk.min(0) - dilate
        self.dim = ijk.max(0) - self.lo + 1 + dilate
        keys = np.unique(self._key(ijk - self.lo))
        del ijk
        strides = (self.dim[1] * self.dim[2], self.dim[2], 1)
        for ax in (2, 1, 0):
            if dilate <= 0:
                break
            keys = np.unique(np.concatenate(
                [keys + d * strides[ax] for d in range(-dilate, dilate + 1)]))
        self.n_source = int(len(xyz))
        self.keys = keys

    def _key(self, ijk):
        return (ijk[:, 0] * self.dim[1] + ijk[:, 1]) * self.dim[2] + ijk[:, 2]

    def contains(self, pts):
        ijk = np.floor(pts / self.voxel).astype(np.int64) - self.lo
        inb = np.all((ijk >= 0) & (ijk < self.dim), axis=1)
        out = np.zeros(len(pts), bool)
        k = self._key(ijk[inb])
        j = np.searchsorted(self.keys, k)
        j = np.minimum(j, len(self.keys) - 1)
        out[inb] = self.keys[j] == k
        return out


def read_pcd_xyz(path):
    """binary の float32 x y z（先頭 3 欄）の PCD を読む。"""
    header = {}
    with path.open('rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise SystemExit(f'{path}: DATA 行が無い')
            key, _, val = line.decode('ascii', 'replace').strip().partition(' ')
            header[key] = val
            if key == 'DATA':
                off = f.tell()
                break
    nf = len(header['FIELDS'].split())
    if header['DATA'] != 'binary' or header['FIELDS'].split()[:3] != ['x', 'y', 'z'] \
            or header['TYPE'].split() != ['F'] * nf or header['SIZE'].split() != ['4'] * nf:
        raise SystemExit(f'{path}: binary の float32 x y z を想定（{header}）')
    n = int(header['POINTS'])
    return np.fromfile(path, np.float32, count=n * nf, offset=off).reshape(n, nf)[:, :3]


# ---------------------------------------------------------------- bag（sqlite3、CDR を直接読む）

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def parse_pointcloud2(buf):
    """sensor_msgs/PointCloud2 の CDR（little endian）を読む。

    rclpy の deserialize より速く、ROS の環境を読み込まずに動く。整列は 4 バイトの
    カプセル化ヘッダの後ろを 0 とした位置で取る（CDR の規則）。
    返り値: (stamp[s], 構造化配列)
    """
    def al(o, n):
        return (o - 4 + n - 1) // n * n + 4
    if buf[1] != 1:
        raise ValueError('big endian の CDR は想定しない')
    o = 4
    sec, nsec = struct.unpack_from('<iI', buf, o)
    o += 8
    n, = struct.unpack_from('<I', buf, o)
    o = al(o + 4 + n, 4)
    h, w, nf = struct.unpack_from('<III', buf, o)
    o += 12
    names, fmts, offs = [], [], []
    for _ in range(nf):
        o = al(o, 4)
        n, = struct.unpack_from('<I', buf, o)
        names.append(bytes(buf[o + 4:o + 4 + n - 1]).decode())
        o = al(o + 4 + n, 4)
        off, = struct.unpack_from('<I', buf, o)
        dt = buf[o + 4]
        o = al(o + 5, 4)
        cnt, = struct.unpack_from('<I', buf, o)
        o += 4
        if cnt != 1:
            raise ValueError(f'count {cnt} の欄は想定しない')
        offs.append(off)
        fmts.append(_DT[dt])
    o += 1   # is_bigendian
    o = al(o, 4)
    step, _row, n = struct.unpack_from('<III', buf, o)
    o += 12
    dtype = np.dtype({'names': names, 'formats': fmts, 'offsets': offs, 'itemsize': step})
    arr = np.frombuffer(buf, dtype=dtype, count=n // step, offset=o)
    return sec + nsec * 1e-9, arr


def open_bag(path):
    db = path if path.suffix == '.db3' else sorted(path.glob('*.db3'))
    if isinstance(db, list):
        if len(db) != 1:
            raise SystemExit(f'{path}: .db3 が 1 つでない（{db}）')
        db = db[0]
    # 読み取り専用で開く（正本の bag を変えない）。
    return sqlite3.connect(f'file:{db}?mode=ro', uri=True), db


# ---------------------------------------------------------------- 1. キャッシュ

def grid_from_yaml(path):
    g = yaml.safe_load(path.read_text())['grid']
    return int(g['width']), int(g['height']), float(g['resolution']), \
        float(g['origin'][0]), float(g['origin'][1])


def to_cell(xy, W, H, res, ox, oy):
    """v2 の格子のセル番号（r*W+c、画像の 0 行が map の y 最大）。格子の外は -1。"""
    c = np.floor((xy[:, 0] - ox) / res).astype(np.int64)
    r = H - 1 - np.floor((xy[:, 1] - oy) / res).astype(np.int64)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    return np.where(ok, r * W + c, -1)


def process_scan(arr, stamp, traj, vox, grid, args, st):
    """1 スキャンを処理し、残った点の (cell int32, z float32, intensity uint8) を返す。"""
    W, H, res, ox, oy = grid
    xyz = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
    fin = np.isfinite(xyz).all(1)
    st['points_finite'] += int(fin.sum())
    r = np.linalg.norm(xyz, axis=1)
    keep = fin & (r >= args.min_range) & (r <= args.max_range)
    xyz = xyz[keep]
    rel_t = arr['time'][keep].astype(np.float64)
    inten = arr['intensity'][keep]
    st['points_in_range'] += len(xyz)
    if not len(xyz):
        return None
    pm, sz, ok = transform_scan(xyz, rel_t, stamp, traj, args.time_bins, args.max_traj_gap)
    st['points_outside_traj'] += int((~ok).sum())
    ok &= (pm[:, 2] - sz) <= args.max_above_sensor
    pm, inten = pm[ok], inten[ok]
    st['points_below_cap'] += len(pm)
    s = vox.contains(pm)
    pm, inten = pm[s], inten[s]
    st['points_static'] += len(pm)
    cell = to_cell(pm[:, :2], W, H, res, ox, oy)
    g = cell >= 0
    st['points_kept'] += int(g.sum())
    return (cell[g].astype(np.int32), pm[g, 2].astype(np.float32),
            np.clip(np.rint(inten[g]), 0, 255).astype(np.uint8))


def cache_meta(args):
    d = {k: str(getattr(args, k)) for k in CACHE_KEYS}
    d['hash'] = hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]
    return d


def build_cache(args, grid):
    cache = args.work_dir / 'cache'
    cache.mkdir(parents=True, exist_ok=True)
    meta = cache_meta(args)
    mpath = cache / 'meta.json'
    if mpath.is_file():
        old = json.loads(mpath.read_text())
        if old.get('hash') != meta['hash']:
            if not args.force:
                raise SystemExit(f'{cache} は別の引数で作ったキャッシュ（{old}）。--force で作り直す')
            for f in cache.glob('chunk_*.npz'):
                f.unlink()
    mpath.write_text(json.dumps(meta, indent=2))

    con, db = open_bag(args.bag)
    tid = con.execute('select id from topics where name=?', (args.topic,)).fetchone()
    if tid is None:
        raise SystemExit(f'{db}: {args.topic} が無い')
    ids = [r[0] for r in con.execute(
        'select id from messages where topic_id=? order by timestamp', (tid[0],))]
    if args.max_scans:
        ids = ids[:args.max_scans]
    traj = load_traj(args.traj)
    print(f'[1] bag {db.name}: {args.topic} {len(ids):,} 個、軌跡 {len(traj[0]):,} 姿勢 '
          f'（{traj[0][0]:.3f}〜{traj[0][-1]:.3f}）')
    vox = None
    chunks = []
    for ci, s in enumerate(range(0, len(ids), args.chunk_scans)):
        path = cache / f'chunk_{ci:04d}.npz'
        chunks.append(path)
        if path.is_file():
            continue
        if vox is None:
            t0 = time.time()
            vox = VoxelSet(read_pcd_xyz(args.static_pcd), args.static_voxel, args.static_dilate)
            print(f'    static.pcd {vox.n_source:,} 点 → 太らせて {len(vox.keys):,} ボクセル'
                  f'（±{args.static_dilate}、{time.time() - t0:.1f} s、最大メモリ {maxrss_gb():.2f} GB）')
        t0 = time.time()
        st = {k: 0 for k in ('scans', 'scans_outside_traj', 'points_finite', 'points_in_range',
                             'points_outside_traj', 'points_below_cap', 'points_static',
                             'points_kept')}
        dts = []
        cells, zs, its = [], [], []
        for mid in ids[s:s + args.chunk_scans]:
            data, = con.execute('select data from messages where id=?', (mid,)).fetchone()
            stamp, arr = parse_pointcloud2(data)
            st['scans'] += 1
            rel = arr['time']
            t_first = stamp + float(rel.min())
            # GLIM の frame の時刻（最初の点）と最も近い軌跡の時刻の差。時刻の解釈の確認用。
            j = np.searchsorted(traj[0], t_first)
            near = min(abs(traj[0][min(j, len(traj[0]) - 1)] - t_first),
                       abs(traj[0][max(j - 1, 0)] - t_first))
            if t_first + float(rel.max() - rel.min()) < traj[0][0] or t_first > traj[0][-1]:
                st['scans_outside_traj'] += 1
                continue
            dts.append(near)
            out = process_scan(arr, stamp, traj, vox, grid, args, st)
            if out is not None:
                cells.append(out[0])
                zs.append(out[1])
                its.append(out[2])
        dts = np.asarray(dts)
        st['traj_match_dt_max_s'] = float(dts.max()) if len(dts) else None
        st['traj_match_within_1ms'] = int((dts < 1e-3).sum())
        tmp = path.with_suffix('.tmp.npz')
        np.savez(tmp, cell=np.concatenate(cells) if cells else np.zeros(0, np.int32),
                 z=np.concatenate(zs) if zs else np.zeros(0, np.float32),
                 intensity=np.concatenate(its) if its else np.zeros(0, np.uint8),
                 stats=json.dumps(st))
        tmp.rename(path)   # 書きかけのファイルを完了とみなさない
        print(f'    chunk {ci:3d}: スキャン {st["scans"]}、残した点 {st["points_kept"]:,}'
              f'（距離内 {st["points_in_range"]:,} の '
              f'{100 * st["points_kept"] / max(1, st["points_in_range"]):.1f}%）'
              f'、軌跡との時刻差 最大 {1e3 * (st["traj_match_dt_max_s"] or 0):.3f} ms'
              f'（{time.time() - t0:.1f} s、最大メモリ {maxrss_gb():.2f} GB）', flush=True)
    con.close()
    return chunks


def iter_cache(chunks):
    for path in chunks:
        with np.load(path) as f:
            yield f['cell'], f['z'], f['intensity']


def cache_stats(chunks):
    tot = {}
    for path in chunks:
        with np.load(path) as f:
            st = json.loads(str(f['stats']))
        for k, v in st.items():
            if k == 'traj_match_dt_max_s':
                tot[k] = max(tot.get(k) or 0.0, v or 0.0)
            else:
                tot[k] = tot.get(k, 0) + v
    return tot


# ---------------------------------------------------------------- 2〜3. 集計

def accumulate(chunk_iter_factory, n_cells, hist_bin, n_bins, percentile, intensity_band):
    """セルごとの点数・最小・最大・下位百分位・地面の強度の平均を返す。

    chunk_iter_factory() は (cell, z, intensity) の塊を返す反復子を作る（2 回読むため）。
    度数分布は点のあるセルだけに持つ（uint16、溢れは 65535 で止める）。
    """
    count = np.zeros(n_cells, np.uint32)
    zmin = np.full(n_cells, np.inf, np.float32)
    zmax = np.full(n_cells, -np.inf, np.float32)
    for cell, z, _ in chunk_iter_factory():
        count += np.bincount(cell, minlength=n_cells).astype(np.uint32)
        np.minimum.at(zmin, cell, z)
        np.maximum.at(zmax, cell, z)
    active = np.nonzero(count)[0]
    idx = np.full(n_cells, -1, np.int32)
    idx[active] = np.arange(len(active), dtype=np.int32)
    hist = np.zeros((len(active), n_bins), np.uint16)
    over = np.zeros(len(active), np.uint32)
    isum = np.zeros(len(active), np.float64)
    icnt = np.zeros(len(active), np.uint32)
    for cell, z, it in chunk_iter_factory():
        a = idx[cell]
        dz = z - zmin[cell]
        b = np.floor(dz / hist_bin).astype(np.int64)
        inr = b < n_bins
        over += np.bincount(a[~inr], minlength=len(active)).astype(np.uint32)
        key = a[inr].astype(np.int64) * n_bins + b[inr]
        u, c = np.unique(key, return_counts=True)
        flat = hist.reshape(-1)
        flat[u] = np.minimum(flat[u].astype(np.uint32) + c, 65535).astype(np.uint16)
        g = dz <= intensity_band
        isum += np.bincount(a[g], weights=it[g].astype(np.float64), minlength=len(active))
        icnt += np.bincount(a[g], minlength=len(active)).astype(np.uint32)

    zp = np.full(len(active), np.nan, np.float32)
    count_low = np.zeros(len(active), np.uint32)
    q = percentile / 100.0
    blk = 1 << 20
    for s in range(0, len(active), blk):
        h = hist[s:s + blk].astype(np.uint32)
        cum = np.cumsum(h, axis=1)
        inr = cum[:, -1]
        count_low[s:s + blk] = inr
        total = inr + over[s:s + blk]
        k = q * (total.astype(np.float64) - 1)          # 0 始まりの順位
        b = np.argmax(cum > k[:, None], axis=1)          # 順位 k の点が入るビン
        found = cum[np.arange(len(h)), b] > k            # 範囲の上に落ちたら False
        before = np.where(b > 0, cum[np.arange(len(h)), np.maximum(b - 1, 0)], 0)
        hb = h[np.arange(len(h)), b].astype(np.float64)
        frac = (k - before + 0.5) / np.maximum(hb, 1)
        val = zmin[active[s:s + blk]] + (b + np.clip(frac, 0, 1)) * hist_bin
        zp[s:s + blk] = np.where(found, val, zmin[active[s:s + blk]] + n_bins * hist_bin)
    # 最小の点そのものはビン 0 の下端にあるので、1 点だけのセルは最小に一致させる。
    one = count[active] == 1
    zp[one] = zmin[active[one]]
    del hist

    out = {
        'count': count,
        'z_min': np.where(count > 0, zmin, np.nan).astype(np.float32),
        'z_max': np.where(count > 0, zmax, np.nan).astype(np.float32),
        'z_p': np.full(n_cells, np.nan, np.float32),
        'count_low': np.zeros(n_cells, np.uint32),
        'intensity_mean': np.full(n_cells, np.nan, np.float32),
        'intensity_count': np.zeros(n_cells, np.uint32),
    }
    out['z_p'][active] = zp
    out['count_low'][active] = count_low
    ok = icnt > 0
    out['intensity_mean'][active[ok]] = (isum[ok] / icnt[ok]).astype(np.float32)
    out['intensity_count'][active] = icnt
    return out


# ---------------------------------------------------------------- 画像

def shrink_mean(a, f):
    """NaN を除いた平均で 1/f にする。"""
    H, W = a.shape
    h, w = H // f, W // f
    b = a[:h * f, :w * f].reshape(h, f, w, f)
    ok = np.isfinite(b)
    s = np.where(ok, b, 0).sum(axis=(1, 3))
    n = ok.sum(axis=(1, 3))
    return np.where(n > 0, s / np.maximum(n, 1), np.nan)


def to_gray(a, lo, hi):
    g = np.clip((a - lo) / max(hi - lo, 1e-9), 0, 1) * 255
    return np.where(np.isfinite(a), g, 0).astype(np.uint8)


def count_image(count, H, W):
    """点数を対数の濃淡にする（0 は黒、1 点で暗い灰、10^4 点で白）。"""
    c = count.reshape(H, W).astype(np.float32)
    v = np.where(c > 0, 40 + 215 * np.clip(np.log10(np.maximum(c, 1)) / 4.0, 0, 1), 0)
    return v.astype(np.uint8)


def region_slice(box, H, res, ox, oy):
    x0, x1, y0, y1 = box
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
    r0, r1 = int(H - 1 - (y1 - oy) / res), int(H - 1 - (y0 - oy) / res)
    return slice(max(0, r0), r1 + 1), slice(max(0, c0), c1 + 1)


def write_images(out, grid, args):
    W, H, res, ox, oy = grid
    sizes = {}
    full = count_image(out['count'], H, W)
    Image.fromarray(full).save(args.work_dir / 'count_log.png', optimize=True)
    args.preview_dir.mkdir(parents=True, exist_ok=True)
    f = 4
    c4 = out['count'].reshape(H, W)[:H // f * f, :W // f * f].reshape(H // f, f, W // f, f)
    c4 = c4.sum(axis=(1, 3)) / (f * f)
    q = np.where(c4 > 0, 40 + 215 * np.clip(np.log10(np.maximum(c4, 1)) / 4.0, 0, 1), 0)
    p = args.preview_dir / 'count_q.png'
    Image.fromarray(q.astype(np.uint8)).save(p, optimize=True)
    sizes[p.name] = p.stat().st_size

    inten = out['intensity_mean'].reshape(H, W)
    fin = inten[np.isfinite(inten)]
    lo, hi = (np.percentile(fin, [1, 99]) if len(fin) else (0, 1))
    p = args.preview_dir / 'intensity_q.png'
    Image.fromarray(to_gray(shrink_mean(inten, f), lo, hi)).save(p, optimize=True)
    sizes[p.name] = p.stat().st_size
    rs, cs = region_slice(START_BOX, H, res, ox, oy)
    p = args.preview_dir / 'start_intensity_1to1.png'
    Image.fromarray(to_gray(inten[rs, cs], lo, hi)).save(p, optimize=True)
    sizes[p.name] = p.stat().st_size
    # 同じ所の地面の高さ（下位百分位）。縁石の段が見えるかの目視用。
    zp = out['z_p'].reshape(H, W)[rs, cs]
    fz = zp[np.isfinite(zp)]
    if len(fz):
        zlo, zhi = np.percentile(fz, [2, 98])
        p = args.preview_dir / 'start_ground_1to1.png'
        Image.fromarray(to_gray(zp, zlo, zhi)).save(p, optimize=True)
        sizes[p.name] = p.stat().st_size
    return sizes, (float(lo), float(hi))


def maxrss_gb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024 / 1024


def short(path):
    path = pathlib.Path(path).resolve()
    for base, pre in ((REPO, ''), (pathlib.Path.home(), '~/')):
        try:
            return pre + str(path.relative_to(base))
        except ValueError:
            pass
    return str(path)


# ---------------------------------------------------------------- main

def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    grid = grid_from_yaml(args.layers_yaml)
    W, H, res, ox, oy = grid
    chunks = build_cache(args, grid)
    t_cache = time.time() - t0
    st = cache_stats(chunks)
    print(f'[1] キャッシュ {len(chunks)} 個（{t_cache:.0f} s）: {st}')

    t1 = time.time()
    out = accumulate(lambda: iter_cache(chunks), H * W, args.hist_bin, args.hist_bins,
                     args.percentile, args.intensity_band)
    t_acc = time.time() - t1
    cnt = out['count']
    cl = out['count_low']
    cells = {f'count_ge_{k}': int((cnt >= k).sum()) for k in (1, 3, 10, 100)}
    cells |= {f'count_low_ge_{k}': int((cl >= k).sum()) for k in (1, 3, 10)}
    print(f'[2] 集計 {t_acc:.0f} s: {cells}')

    npz = args.work_dir / 'ground_raster.npz'
    tmp = args.work_dir / 'ground_raster.tmp.npz'
    meta = {'width': W, 'height': H, 'resolution': res, 'origin': [ox, oy],
            'percentile': args.percentile, 'hist_bin': args.hist_bin, 'hist_bins': args.hist_bins,
            'intensity_band': args.intensity_band, 'cache': cache_meta(args)}
    np.savez_compressed(tmp, **{k: v.reshape(H, W) for k, v in out.items()},
                        meta=json.dumps(meta))
    tmp.rename(npz)
    sizes, irange = write_images(out, grid, args)

    summary = {
        'params': {k: (short(v) if isinstance(v, pathlib.Path) else v) for k, v in vars(args).items()},
        'stats': st, 'cells': cells,
        'z_p_minus_min_median_m': (float(np.nanmedian((out['z_p'] - out['z_min'])[cl >= 10]))
                                   if (cl >= 10).any() else None),
        'intensity_display_range': irange,
        'timing_s': {'cache': round(t_cache, 1), 'accumulate': round(t_acc, 1),
                     'total': round(time.time() - t0, 1)},
        'peak_rss_gb': round(maxrss_gb(), 2),
        'npz': short(npz), 'preview_png_bytes': sizes,
    }
    (args.preview_dir / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'所要 {summary["timing_s"]["total"]} s、最大メモリ {summary["peak_rss_gb"]} GB。'
          f'npz: {npz}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

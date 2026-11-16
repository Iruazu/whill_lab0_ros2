#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_ground_raster_from_bag の合成データでの単体テスト（#202）。

確かめること:
  1. 姿勢の補間：並進は線形、回転は slerp。軌跡の外と、間隔が広すぎる所は使わない。
  2. スキャンの変換：点ごとの時刻で姿勢を引くので、走りながらのスキャンの歪みが取れる。
  3. セルの集計：点数・最小・最大・下位百分位（度数分布から）・地面の強度の平均。
  4. 静的ボクセルの集合と太らせ方、PointCloud2 の CDR の読み取り。

Run:
    python3 -m pytest scripts/tests/test_ground_raster_from_bag.py -q
"""

import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import map_ground_raster_from_bag as g  # noqa: E402


def _yaw_quat(yaw):
    return np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])


def _traj(yaws, ps, ts):
    q = np.array([_yaw_quat(y) for y in yaws])
    return np.asarray(ts, float), np.asarray(ps, float), q


# ---------------------------------------------------------------- 1. 補間

def test_interp_translation_linear_and_rotation_slerp():
    tt, tp, tq = _traj([0.0, np.pi / 2], [[0, 0, 0], [2, 4, 6]], [10.0, 11.0])
    p, R, ok = g.interp_poses(tt, tp, tq, np.array([10.0, 10.25, 10.5, 11.0]), max_gap=2.0)
    assert ok.all()
    np.testing.assert_allclose(p[1], [0.5, 1.0, 1.5], atol=1e-12)
    np.testing.assert_allclose(p[3], [2, 4, 6], atol=1e-12)
    # slerp：0.25 の時刻で yaw 22.5°、0.5 で 45°（四元数の線形補間なら角度が等速にならない）。
    for i, yaw in ((1, np.pi / 8), (2, np.pi / 4)):
        np.testing.assert_allclose(R[i], [[np.cos(yaw), -np.sin(yaw), 0],
                                          [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]], atol=1e-12)


def test_interp_takes_short_arc():
    # q と −q は同じ回転。符号が逆の四元数の間でも短い方（10°）を通る。
    tt = np.array([0.0, 1.0])
    tp = np.zeros((2, 3))
    tq = np.array([_yaw_quat(np.deg2rad(5)), -_yaw_quat(np.deg2rad(-5))])
    _, R, _ = g.interp_poses(tt, tp, tq, np.array([0.5]), max_gap=2.0)
    np.testing.assert_allclose(R[0], np.eye(3), atol=1e-12)


def test_interp_rejects_outside_and_gaps():
    tt, tp, tq = _traj([0, 0, 0], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], [0.0, 0.1, 0.6])
    _, _, ok = g.interp_poses(tt, tp, tq, np.array([-0.01, 0.0, 0.05, 0.1, 0.3, 0.6, 0.61]),
                              max_gap=0.25)
    # 0.1〜0.6 は間隔 0.5 s > 0.25 s なので間の 0.3 は使わない。0.0・0.1・0.6 は姿勢そのもの。
    assert ok.tolist() == [False, True, True, True, False, True, False]


def test_interp_endpoint_exact():
    tt, tp, tq = _traj([0, 1.0], [[0, 0, 0], [1, 0, 0]], [0.0, 0.1])
    p, R, ok = g.interp_poses(tt, tp, tq, np.array([0.1]), max_gap=0.25)
    assert ok[0]
    np.testing.assert_allclose(p[0], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(R[0], g.quat_to_rot(tq[1:])[0], atol=1e-12)


# ---------------------------------------------------------------- 2. スキャンの変換

def test_transform_scan_removes_motion_skew():
    # 1 m/s で x に進みながら、同じ地面の点（地図で x=5）を 0.1 s の間に何度も測る。
    # センサ座標では 5 − t に見えるが、点ごとの時刻で姿勢を引けば地図では全部 x=5 になる。
    traj = _traj([0, 0, 0], [[0, 0, 1], [0.1, 0, 1], [0.2, 0, 1]], [100.0, 100.1, 100.2])
    stamp = 100.15                      # header は最後の点の時刻（bag と同じ）
    rel_t = np.linspace(-0.1, 0.0, 101)
    t_abs = stamp + rel_t
    xyz = np.stack([5.0 - t_abs + 100.0, np.zeros_like(rel_t), -np.ones_like(rel_t)], 1)
    out, sz, ok = g.transform_scan(xyz, rel_t, stamp, traj, n_bins=50, max_gap=0.25)
    assert ok.all()
    # 区切りの中央の時刻を使うので、誤差は区切りの半分（1 ms × 1 m/s）以内。
    assert np.abs(out[:, 0] - 5.0).max() <= 0.0011
    np.testing.assert_allclose(out[:, 2], 0.0, atol=1e-12)
    np.testing.assert_allclose(sz, 1.0, atol=1e-12)


def test_transform_scan_marks_points_before_trajectory():
    traj = _traj([0, 0], [[0, 0, 0], [0.1, 0, 0]], [100.0, 100.1])
    rel_t = np.linspace(-0.1, 0.0, 11)
    _, _, ok = g.transform_scan(np.ones((11, 3)), rel_t, 100.05, traj, 10, 0.25)
    assert not ok[0] and ok[-1]


# ---------------------------------------------------------------- 3. セルの集計

def _acc(cell, z, it, n_cells=10, n_bins=64, chunks=2):
    parts = np.array_split(np.arange(len(cell)), chunks)

    def factory():
        for idx in parts:
            yield (cell[idx].astype(np.int32), z[idx].astype(np.float32),
                   it[idx].astype(np.uint8))
    return g.accumulate(factory, n_cells, 0.01, n_bins, 5.0, 0.10)


def test_accumulate_count_min_max_percentile_intensity():
    rng = np.random.default_rng(0)
    # セル 2：地面 1000 点（z 一様 0.00〜0.02）と、植栽 200 点（z 0.4〜1.5、範囲 0.64 m の上も含む）。
    zg = rng.uniform(0.0, 0.02, 1000)
    zv = rng.uniform(0.4, 1.5, 200)
    # セル 5：1 点だけ。セル 7：2 点。
    cell = np.r_[np.full(1200, 2), 5, 7, 7]
    z = np.r_[zg, zv, -0.3, 0.10, 0.25]
    it = np.r_[np.full(1000, 20), np.full(200, 200), 50, 60, 90]
    out = _acc(cell, z, it)
    assert out['count'][2] == 1200 and out['count'][5] == 1 and out['count'][7] == 2
    assert out['count'][0] == 0 and np.isnan(out['z_p'][0])
    np.testing.assert_allclose(out['z_min'][2], zg.min(), atol=1e-6)
    np.testing.assert_allclose(out['z_max'][2], zv.max(), atol=1e-6)
    # 下位 5% は全 1200 点での順位（植栽を含む）。度数分布の 1 cm のビンの中で一様とみなすので、
    # 正確な値とはビンの幅以内で合う。
    exact = np.sort(z[:1200])[int(0.05 * 1199)]
    assert abs(out['z_p'][2] - exact) < 0.01
    # 範囲（最小から 0.64 m）の中の点数：地面 1000 点と、0.64 m 未満の植栽。
    assert out['count_low'][2] == 1000 + int((zv - zg.min() < 0.64).sum())
    # 強度は最小から 0.10 m 以内（地面）の点だけの平均。植栽の 200 は混ざらない。
    np.testing.assert_allclose(out['intensity_mean'][2], 20.0)
    assert out['intensity_count'][2] == 1000
    np.testing.assert_allclose(out['z_p'][5], -0.3, atol=1e-6)
    np.testing.assert_allclose(out['intensity_mean'][7], 60.0)   # 最小の点だけが 0.10 m 以内
    assert out['z_min'][7] <= out['z_p'][7] <= out['z_max'][7]


def test_accumulate_percentile_above_histogram_range():
    # 最小の 1 点の上に、範囲の外（0.64 m より上）の点しかないとき、百分位は範囲の上端にする。
    cell = np.r_[np.full(10, 1)]
    z = np.r_[0.0, np.full(9, 2.0)]
    out = _acc(cell, z, np.zeros(10), n_cells=3)
    # 順位 0.05×9 = 0.45 → 最小の点（ビン 0）。
    assert out['z_p'][1] < 0.01
    z = np.r_[0.0, np.full(99, 2.0)]
    out = _acc(np.full(100, 1), z, np.zeros(100), n_cells=3)
    # 順位 4.95 → 範囲の上の点。0.64 m を返す（上の点の高さは分からない）。
    np.testing.assert_allclose(out['z_p'][1], 0.64, atol=1e-6)


def test_accumulate_chunking_does_not_change_result():
    rng = np.random.default_rng(1)
    cell = rng.integers(0, 50, 5000)
    z = rng.normal(0, 0.05, 5000)
    it = rng.integers(0, 255, 5000)
    a = _acc(cell, z, it, n_cells=50, chunks=1)
    b = _acc(cell, z, it, n_cells=50, chunks=7)
    for k in a:
        np.testing.assert_array_equal(a[k], b[k])


# ---------------------------------------------------------------- 4. ボクセル・CDR・格子

def test_voxelset_contains_and_dilate():
    centers = np.array([[0.05, 0.05, 0.05], [1.05, 0.05, 0.05]])
    v0 = g.VoxelSet(centers, 0.1, dilate=0)
    q = np.array([[0.01, 0.09, 0.0], [0.11, 0.05, 0.05], [1.0, 0.0, 0.0], [0.5, 0.5, 0.5]])
    assert v0.contains(q).tolist() == [True, False, True, False]
    v1 = g.VoxelSet(centers, 0.1, dilate=1)
    assert len(v1.keys) == 2 * 27
    # 斜め隣（±1 ボクセル）は入り、2 ボクセル先は入らない。
    q = np.array([[0.15, 0.15, -0.05], [0.25, 0.05, 0.05], [-0.05, -0.05, -0.05]])
    assert v1.contains(q).tolist() == [True, False, True]


def _cdr_pointcloud2(stamp_sec, stamp_nsec, frame, pts):
    """sensor_msgs/PointCloud2 の CDR（little endian）を作る（読み取りの往復の確認用）。"""
    b = bytearray(b'\x00\x01\x00\x00')

    def pad(n):
        while (len(b) - 4) % n:
            b.append(0)

    def string(s):
        pad(4)
        e = s.encode() + b'\x00'
        b.extend(struct.pack('<I', len(e)) + e)

    b.extend(struct.pack('<iI', stamp_sec, stamp_nsec))
    string(frame)
    pad(4)
    fields = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('ring', 16, 4),
              ('time', 18, 7)]
    b.extend(struct.pack('<III', 1, len(pts), len(fields)))
    for name, off, dt in fields:
        string(name)
        pad(4)
        b.extend(struct.pack('<IB', off, dt))
        pad(4)
        b.extend(struct.pack('<I', 1))
    b.append(0)
    pad(4)
    step = 22
    data = bytearray()
    for x, y, z, i, r, t in pts:
        data.extend(struct.pack('<ffffHf', x, y, z, i, r, t))
    b.extend(struct.pack('<III', step, step * len(pts), len(data)))
    b.extend(data)
    b.append(1)
    return bytes(b)


def test_parse_pointcloud2_roundtrip():
    pts = [(1.0, 2.0, 3.0, 10.0, 3, -0.1), (float('nan'), 0.0, 0.0, 0.0, 15, 0.0)]
    stamp, arr = g.parse_pointcloud2(_cdr_pointcloud2(1783663663, 995902538, 'velodyne', pts))
    assert abs(stamp - 1783663663.995902538) < 1e-6
    assert arr.dtype.names == ('x', 'y', 'z', 'intensity', 'ring', 'time')
    np.testing.assert_allclose(arr['x'][0], 1.0)
    np.testing.assert_allclose(arr['time'][0], -0.1, rtol=1e-6)
    assert arr['ring'].tolist() == [3, 15]
    assert np.isnan(arr['x'][1])


def test_to_cell_matches_roadway_grid():
    # map_roadway_from_curbs.xy_to_rc と同じ向き（画像の 0 行が map の y 最大）。
    from map_roadway_from_curbs import xy_to_rc
    W, H, res, ox, oy = 6640, 6295, 0.05, -128.25, -90.35
    xy = np.array([[0.0, 0.0], [-128.24, -90.34], [203.74, 224.39], [500.0, 0.0]])
    cell = g.to_cell(xy, W, H, res, ox, oy)
    r, c = xy_to_rc(xy[:3], H, res, ox, oy)
    assert cell[:3].tolist() == (r * W + c).tolist()
    assert cell[3] == -1


def test_roadway_reads_dense_npz(tmp_path):
    # map_roadway_from_curbs --ground-npz：局所最小から --ground-band より高いセル（生垣の上など）は
    # 地面が見えていないとみなし、格子・百分位が違う npz は受け付けない。
    import json

    import pytest
    from map_roadway_from_curbs import ground_raster_from_npz, parse_args
    H, W = 40, 40
    zp = np.zeros((H, W), np.float32)
    zp[10:14, 10:14] = 1.2                     # 地面の点が無く、生垣の上面だけのセル
    zmin = zp.copy()
    cnt = np.full((H, W), 50, np.uint32)
    zp[0, 0] = np.nan
    cnt[0, 0] = 0
    meta = {'width': W, 'height': H, 'resolution': 0.05, 'origin': [0.0, 0.0], 'percentile': 5.0}
    path = tmp_path / 'g.npz'
    np.savez(path, z_p=zp, z_min=zmin, count_low=cnt, meta=json.dumps(meta))
    a = parse_args([])
    ground, count, n_in = ground_raster_from_npz(path, H, W, 0.05, 0.0, 0.0, a)
    assert np.isnan(ground[11, 11]) and count[11, 11] == 0
    assert ground[30, 30] == 0.0 and count[30, 30] == 50
    assert count[0, 0] == 0
    assert n_in == 50 * (H * W - 16 - 1)
    with pytest.raises(SystemExit):
        ground_raster_from_npz(path, H, W + 1, 0.05, 0.0, 0.0, a)
    a.ground_percentile = 10.0
    with pytest.raises(SystemExit):
        ground_raster_from_npz(path, H, W, 0.05, 0.0, 0.0, a)

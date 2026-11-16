#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_georef_gsi の変換・復号・道路の面の単体テスト（#205）。通信しない。

pyproj を使わずに投影を書いたので、既知の値を持ち込まずに確かめられる性質で検査する：
  1. 原点（36°N, 139°50'E）は (0, 0)、往復で元に戻る
  2. 中央子午線の上では northing = k0 × 子午線弧長（楕円の弧長を数値積分した独立の値）
  3. 等角（東と北の縮尺が等しい）で、縮尺は k0 (1 + E²/2R²) に一致する
タイルの式、map → 平面直角座標の剛体変換の向き、MVT の復号、粗い探索（FFT 相関）、
道路縁からの道路の面も、合成データで確かめる。

Run:
    python3 -m pytest scripts/tests/test_georef_gsi.py -q
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import map_georef_gsi as g  # noqa: E402

A = g.GRS80_A
F = g.GRS80_F
E2 = F * (2 - F)


def _meridian_arc(lat0_deg, lat1_deg, n=20000):
    """子午線弧長を、子午線の曲率半径 M(φ) の数値積分（Simpson）で求める。投影の式とは独立。"""
    phi = np.linspace(math.radians(lat0_deg), math.radians(lat1_deg), n + 1)
    M = A * (1 - E2) / (1 - E2 * np.sin(phi) ** 2) ** 1.5
    h = (phi[-1] - phi[0]) / n
    return h / 3 * (M[0] + M[-1] + 4 * M[1:-1:2].sum() + 2 * M[2:-1:2].sum())


def test_origin_and_roundtrip():
    E, N = g.tm_forward(g.ZONE9_LAT0, g.ZONE9_LON0)
    assert abs(float(E)) < 1e-6 and abs(float(N)) < 1e-6
    lat = np.array([36.55, 36.0, 37.2, 35.5])
    lon = np.array([139.93, 139.0, 140.6, 139.8333])
    E, N = g.tm_forward(lat, lon)
    la, lo = g.tm_inverse(E, N)
    assert np.max(np.abs(la - lat)) < 1e-10
    assert np.max(np.abs(lo - lon)) < 1e-10


@pytest.mark.parametrize('lat', [36.55, 35.2, 37.9])
def test_central_meridian_northing_is_scaled_arc(lat):
    E, N = g.tm_forward(lat, g.ZONE9_LON0)
    assert abs(float(E)) < 1e-6
    expect = g.ZONE9_K0 * _meridian_arc(g.ZONE9_LAT0, lat)
    assert abs(float(N) - expect) < 1e-3  # 1 mm


def test_conformal_and_scale_factor():
    lat, lon = 36.55, 139.93
    E0, N0 = g.tm_forward(lat, lon)
    phi = math.radians(lat)
    Mr = A * (1 - E2) / (1 - E2 * math.sin(phi) ** 2) ** 1.5   # 子午線の曲率半径
    Nr = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)            # 卯酉線の曲率半径
    d = 1e-6  # 度
    En, Nn = g.tm_forward(lat + d, lon)
    Ee, Ne = g.tm_forward(lat, lon + d)
    k_north = math.hypot(float(En - E0), float(Nn - N0)) / (Mr * math.radians(d))
    k_east = math.hypot(float(Ee - E0), float(Ne - N0)) / (Nr * math.cos(phi) * math.radians(d))
    assert abs(k_north - k_east) < 1e-7
    R = math.sqrt(Mr * Nr)
    k_expect = g.ZONE9_K0 * (1 + float(E0) ** 2 / (2 * R ** 2))
    assert abs(k_north - k_expect) < 1e-7


def test_meridian_convergence_sign():
    # 原点より東では座標北が真北より東に傾く（γ > 0）。中央子午線では 0
    assert g.meridian_convergence_deg(36.55, 139.93) > 0
    assert abs(g.meridian_convergence_deg(36.55, g.ZONE9_LON0)) < 1e-12


def test_tile_pixels():
    x, y = g.lonlat_to_global_px(0.0, 0.0, 3)
    assert abs(float(x) - 128 * 8) < 1e-9 and abs(float(y) - 128 * 8) < 1e-9
    la, lo = g.global_px_to_lonlat(*g.lonlat_to_global_px(36.55, 139.93, 18), 18)
    assert abs(float(la) - 36.55) < 1e-10 and abs(float(lo) - 139.93) < 1e-10


def test_transform_convention_and_inverse():
    # yaw 90°：map の +x が東から反時計回りに 90°、つまり北を向く
    T = g.make_transform(90.0, 100.0, 200.0)
    E, N = g.apply(T, 1.0, 0.0)
    assert abs(float(E) - 100.0) < 1e-9 and abs(float(N) - 201.0) < 1e-9
    T = g.make_transform(-95.15, 8521.7, 61107.3)
    x, y = g.apply(np.linalg.inv(T), *g.apply(T, 12.3, -4.5))
    assert abs(float(x) - 12.3) < 1e-9 and abs(float(y) + 4.5) < 1e-9


def _varint(v):
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def _field(num, wire, payload):
    key = _varint((num << 3) | wire)
    if wire == 0:
        return key + _varint(payload)
    return key + _varint(len(payload)) + payload


def _packed(vals):
    return b''.join(_varint(v) for v in vals)


def _zz(v):
    return (v << 1) ^ (v >> 31)


def test_mvt_decode_line_and_polygon():
    # 線 (10,20)→(13,24)、面 (0,0)→(5,0)→(5,5) を閉じる。属性 ftCode=2201
    line = [(1 << 3) | 1, _zz(10), _zz(20), (1 << 3) | 2, _zz(3), _zz(4)]
    poly = [(1 << 3) | 1, 0, 0, (2 << 3) | 2, _zz(5), 0, 0, _zz(5), (1 << 3) | 7]
    value = _field(5, 0, 2201)  # uint_value
    f1 = _field(2, 2, _packed([0, 0])) + _field(3, 0, 2) + _field(4, 2, _packed(line))
    f2 = _field(3, 0, 3) + _field(4, 2, _packed(poly))
    layer = (_field(15, 0, 2) + _field(1, 2, b'road') + _field(2, 2, f1) + _field(2, 2, f2)
             + _field(3, 2, b'ftCode') + _field(4, 2, value) + _field(5, 0, 4096))
    L = g.mvt_decode(_field(3, 2, layer))
    feats = L['road']['features']
    assert L['road']['extent'] == 4096
    assert feats[0]['props'] == {'ftCode': 2201} and feats[0]['type'] == 2
    assert feats[0]['parts'] == [[(10, 20), (13, 24)]]
    assert feats[1]['parts'] == [[(0, 0), (5, 0), (5, 5), (0, 0)]]


def test_coarse_search_recovers_pose():
    rng = np.random.default_rng(1)
    # 合成の「GSI の輪郭」：ランダムな長方形の外周。map の点は同じ外周を既知の姿勢で逆に写したもの
    gres = 0.5
    n = 400
    grid0 = (1000.0 - 100.0, 2000.0 - 100.0)
    edge = np.zeros((n, n), bool)
    T_true = g.make_transform(-95.0, 1000.0 + 6.0, 2000.0 - 4.0)
    Tinv = np.linalg.inv(T_true)
    struct_xy = []
    for _ in range(25):
        cx, cy = rng.uniform(-50, 50, 2)
        w, h = rng.uniform(5, 20, 2)
        t = np.linspace(0, 1, 200)
        xs = np.concatenate([cx + w * t, cx + w * np.ones_like(t), cx + w * (1 - t), cx * np.ones_like(t)])
        ys = np.concatenate([cy * np.ones_like(t), cy + h * t, cy + h * np.ones_like(t), cy + h * (1 - t)])
        E, N = g.apply(T_true, xs, ys)
        edge[np.round((N - grid0[1]) / gres).astype(int), np.round((E - grid0[0]) / gres).astype(int)] = True
        struct_xy.append(np.stack([xs, ys], 1))
    P = np.concatenate(struct_xy)
    assert np.allclose(g.apply(Tinv, *g.apply(T_true, P[:, 0], P[:, 1]))[0], P[:, 0])

    def S_fn(x, y):
        # map 側：構造物の点からの距離のガウス（総当たりで近似。テスト用に粗く）
        out = np.zeros(x.shape)
        ix = np.round(x / gres).astype(int)
        iy = np.round(y / gres).astype(int)
        occ = set(zip(np.round(P[:, 0] / gres).astype(int).tolist(), np.round(P[:, 1] / gres).astype(int).tolist()))
        flat = [(a, b) in occ for a, b in zip(ix.ravel().tolist(), iy.ravel().tolist())]
        out.ravel()[:] = flat
        return out

    def C_fn(x, y):
        return ((np.abs(x) < 80) & (np.abs(y) < 80)).astype(float)

    res = g.coarse_search(edge, S_fn, C_fn, grid0, gres, [-100.0, -95.0, -90.0, 0.0], 20.0, 10)
    best = max(range(len(res)), key=lambda i: res[i][0])
    assert best == 1
    assert abs(res[best][2] - 1006.0) <= gres and abs(res[best][3] - 1996.0) <= gres


def test_road_area_between_edges_and_no_leak():
    res = 0.1
    m = np.zeros((200, 300), bool)
    edges = m.copy()
    centers = m.copy()
    edges[60, :] = True        # 道路の上の縁
    edges[100, :] = True       # 道路の下の縁（幅 4 m）
    centers[80, :] = True
    # 下の縁に 1 m の切れ目。領域成長なら区画全体に漏れるが、この方法では
    # 「最も近い中心線の点までの線分が縁を横切らない」セルだけなので、切れ目の真下の帯
    # （幅は切れ目と同じ）に限られ、中心線から max_half_width_m より先には出ない
    edges[100, 140:150] = False
    road, info = g.gsi_road_area(edges, centers, res, gap_m=0.2, max_half_width_m=10.0)
    assert road[61:100, 50].all()
    assert road[:55, 50].sum() == 0
    below = road[105:, :]
    cols = np.nonzero(below.any(axis=0))[0]
    assert cols.min() >= 140 and cols.max() < 150      # 切れ目の真下だけ
    assert not road[181:, :].any()                      # 中心線（80 行）から 10 m より先は無い

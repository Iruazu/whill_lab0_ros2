#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""nav2_plan_check の地図・経路の計算の単体テスト（#154）。ROS のノードは起動しない。

確かめること：
  1. map の座標 → pgm の画素が map_route_check.py・waypoints.yaml の式と同じ（行は上から）
  2. OCC・非 FREE までの距離、inflation の内側の割合、長さ（直線の経路で手計算と一致）
  3. 経路のずれ：同じ経路は 0、平行に 0.3 m ずらすと平均・最大とも 0.3 m、片側だけの寄り道は
     Hausdorff で拾う
  4. --domain 0（実機のスタックと同じ domain）は引数の段階で止める
  5. costmap_raw の行の向きを pgm に合わせ、読ませた地図との食い違い（OCC が LETHAL でない、FREE が
     NO_INFORMATION）を数える。上下の返し忘れ・全面 UNKNOWN の初期の costmap は食い違いになる
  6. 地図の外の経路の点は距離に使わない。成功した組が無ければ絵を書かない

Run:
    python3 -m pytest scripts/tests/test_nav2_plan_check.py -q
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import nav2_plan_check as m  # noqa: E402

RES = 0.05


def make_map(h=40, w=200):
    # 254 = FREE、0 = OCC、205 = UNKNOWN（compose_occupancy の値）
    img = np.full((h, w), 205, np.uint8)
    img[10:30, :] = 254
    img[5, :] = 0  # FREE の帯の上（map の +y 側）に 5 セル離して OCC の線
    p = (255.0 - img.astype(np.float32)) / 255.0
    return {'res': RES, 'origin': [0.0, 0.0], 'img': img, 'occ': p > 0.65, 'free': p < 0.196}


def test_world_to_px_matches_waypoint_formula():
    mp = make_map()
    H = mp['img'].shape[0]
    c, r = 17, 23
    # waypoints.yaml の式：x = ox + (col + 0.5) * res、y = oy + (H - row - 0.5) * res
    xy = np.array([[(c + 0.5) * RES, (H - r - 0.5) * RES]])
    pc, pr = m.world_to_px(mp, xy)
    assert (int(pc[0]), int(pr[0])) == (c, r)


def test_path_metrics_straight_line():
    mp = make_map()
    H = mp['img'].shape[0]
    row = 12  # FREE の帯の上端（行 10）から 2 セル、OCC（行 5）から 7 セル
    y = (H - row - 0.5) * RES
    xy = np.array([[(c + 0.5) * RES, y] for c in range(20, 120)])
    rec = {'xy': xy.tolist()}
    dist_occ = ndimage.distance_transform_edt(~mp['occ']) * RES
    dist_nf = ndimage.distance_transform_edt(mp['free']) * RES
    m.path_metrics(mp, dist_occ, dist_nf, rec)
    assert rec['length_m'] == pytest.approx(99 * RES, abs=1e-6)
    assert rec['min_dist_occ_m'] == pytest.approx(7 * RES, abs=1e-6)
    assert rec['min_dist_nonfree_m'] == pytest.approx(3 * RES, abs=1e-6)
    assert rec['frac_within_0_6m'] == 1.0
    assert rec['n_within_0_45m'] == 100
    assert rec['frac_nonfree_within_0_6m'] == 1.0


def test_shift_metrics():
    base = np.array([[x * 0.05, 0.0] for x in range(200)])
    assert m.shift_metrics(base, base) == {'mean_m': 0.0, 'p95_m': 0.0, 'hausdorff_m': 0.0}
    moved = base + [0.0, 0.3]
    s = m.shift_metrics(base, moved)
    assert s['mean_m'] == pytest.approx(0.3, abs=1e-3)
    assert s['hausdorff_m'] == pytest.approx(0.3, abs=1e-3)
    # 新しい経路が短くなって基準の端を通らないとき、新→基準の距離は 0 でも Hausdorff は拾う
    short = base[:100]
    s = m.shift_metrics(base, short)
    assert s['mean_m'] == 0.0
    # 基準の端 x = 199 × 0.05 から、短い経路の端 x = 99 × 0.05 まで
    assert s['hausdorff_m'] == pytest.approx(100 * 0.05, abs=1e-3)


def test_domain_zero_rejected(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['nav2_plan_check.py', '--map', 'a.yaml', '--waypoints', 'w.yaml',
                                      '--out-dir', '/tmp/x', '--domain', '0'])
    with pytest.raises(SystemExit) as e:
        m.parse_args()
    assert e.value.code == 2


def test_costmap_rows_and_mismatch():
    # costmap_raw の data は行が下（map の −y 側）から。上下を返して pgm と比べ、一致なら食い違い 0
    mp = make_map()
    H, W = mp['img'].shape
    cost_rows_top = np.full((H, W), 255, np.uint8)
    cost_rows_top[mp['free']] = 0
    cost_rows_top[mp['occ']] = 254
    cost_rows_top[11, :] = 120  # inflation のコストは FREE の上でも食い違いにしない
    data = cost_rows_top[::-1].tobytes()
    cost = m.costmap_to_pgm_rows(data, W, H)
    assert np.array_equal(cost, cost_rows_top)
    assert m.costmap_mismatch(cost, mp) == {'occ_not_lethal': 0, 'unknown_on_free': 0}
    # 上下を返し忘れると食い違う（OCC の線が反対側に来る）
    flipped = np.frombuffer(data, np.uint8).reshape(H, W)
    assert m.costmap_mismatch(flipped, mp)['occ_not_lethal'] == W
    # 全面 UNKNOWN の初期の costmap も食い違う
    blank = np.full((H, W), 255, np.uint8)
    mism = m.costmap_mismatch(blank, mp)
    assert mism['occ_not_lethal'] == W and mism['unknown_on_free'] == int(mp['free'].sum())


def test_path_metrics_skips_points_outside_map():
    mp = make_map()
    H = mp['img'].shape[0]
    y = (H - 12 - 0.5) * RES
    # 負の x の点は、そのまま添字にすると反対の端（列 −1）を読む
    xy = np.array([[-0.5, y], [-0.025, y]] + [[(c + 0.5) * RES, y] for c in range(20, 30)])
    rec = {'xy': xy.tolist()}
    dist_occ = ndimage.distance_transform_edt(~mp['occ']) * RES
    dist_nf = ndimage.distance_transform_edt(mp['free']) * RES
    m.path_metrics(mp, dist_occ, dist_nf, rec)
    assert rec['n_outside_map'] == 2
    assert rec['min_dist_occ_m'] == pytest.approx(7 * RES, abs=1e-6)


def test_draw_crop_without_success(tmp_path):
    mp = make_map()
    res = [{'map': 'a.yaml', 'pairs': [{'start': 'a', 'goal': 'b', 'success': False}]}]
    assert m.draw_crop(mp, res, 1.0, tmp_path / 'x.png') is None
    assert not (tmp_path / 'x.png').exists()

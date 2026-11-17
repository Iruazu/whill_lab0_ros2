#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_widen_hand_roadway の合成データでの単体テスト（#154、154w）。実データも地図も使わない。

確かめること：
  1. 赤・橙（snapped かつ step / structure）の区間に面する所だけ広げる。青（強度）・桃（未補正）は広げない
  2. 上限（--max-widen-m）を超えて広げない。手塗りは削らない
  3. 手塗りから離れた飛び地は足さない
  4. 段・構造物・塩は帯より優先。1 セルおきの点の並びも、太らせれば帯がその奥へ入らない
  5. 除外の箱
  6. 区間の表の読み方（どの状態・根拠を赤・橙とみなすか、補正後の位置）

Run:
    python3 -m pytest scripts/tests/test_widen_hand_roadway.py -q
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import map_widen_hand_roadway as m  # noqa: E402

RES = 0.05
H, W = 60, 100
OX, OY = 0.0, 0.0


def y_of_row_top(r):
    """行 r の上の境の map の y。"""
    return (H - r) * RES


def scene():
    """横向きの道。手塗りは 20〜39 行、補正後の GSI の面は 10〜49 行（両側に 0.5 m 広い）。
    上の縁（10 行の上の境）は左半分が赤（段で寄せた）、右半分が桃（未補正）。下の縁は青（強度だけ）。"""
    hand = np.zeros((H, W), bool)
    hand[20:40] = True
    gsi = np.zeros((H, W), bool)
    gsi[10:50] = True
    xs = np.arange(0, W * RES, 0.01)
    top = np.column_stack([xs, np.full(xs.size, y_of_row_top(10))])
    bot = np.column_stack([xs, np.full(xs.size, y_of_row_top(50))])
    pts = np.vstack([top, bot])
    trusted = np.concatenate([xs < W * RES / 2, np.zeros(xs.size, bool)])
    return hand, gsi, pts, trusted


def run(hand, gsi, pts, trusted, barrier=None, exclude=None, cap=1.5, dil=0.0):
    z = np.zeros((H, W), bool)
    band, stages, _, _ = m.widen_band(hand, gsi, z if barrier is None else barrier, z if exclude is None else exclude,
                                      pts, trusted, RES, OX, OY, cap, 0.05, dil)
    return band, stages


def test_only_red_orange_side_widens():
    hand, gsi, pts, trusted = scene()
    band, _ = run(hand, gsi, pts, trusted)
    # 上の左半分（赤）は手塗りの縁から GSI の縁まで全部
    assert band[10:20, :45].all()
    # 上の右半分（桃）と下（青）は広げない。境目の付近（tie）は広げない側
    assert not band[10:20, 51:].any()
    assert not band[40:50].any()
    # 手塗りとは重ならない（手塗りは変えない）
    assert not (band & hand).any()


def test_cap_limits_distance():
    hand, gsi, pts, trusted = scene()
    band, _ = run(hand, gsi, pts, trusted, cap=0.25)
    # 0.25 m = 5 セル。手塗りの縁（20 行）から 5 セル（15〜19 行）まで
    assert band[15:20, :45].all()
    assert not band[:15].any()


def test_island_not_touching_hand_is_dropped():
    hand, gsi, pts, trusted = scene()
    gsi[17] = False   # 手塗りの外に、手塗りに接しない帯（10〜16 行）を作る
    band, st = run(hand, gsi, pts, trusted)
    assert band[18:20, :45].all()
    assert not band[10:17].any()
    assert st['dropped_islands'] > 0


def test_barrier_wins_and_dotted_barrier_is_closed():
    hand, gsi, pts, trusted = scene()
    barrier = np.zeros((H, W), bool)
    barrier[14, ::2] = True   # 1 セルおきの構造物の点（10 cm ボクセルの並び）
    band, _ = run(hand, gsi, pts, trusted, barrier=barrier)
    assert not (band & barrier).any()
    # 太らせないと、点の間を通って奥（10〜13 行）まで帯がつながる
    assert band[10:14, :45].any()
    band_d, _ = run(hand, gsi, pts, trusted, barrier=barrier, dil=0.05)
    assert not (band_d & barrier).any()
    assert not band_d[10:15].any()
    assert band_d[16:20, :45].all()


def test_exclude_box():
    hand, gsi, pts, trusted = scene()
    # 左の 0〜1 m（0〜19 列）を箱で除く
    ex = m.box_mask((H, W), RES, OX, OY, (0.0, 1.0, 0.0, H * RES))
    assert ex[:, :20].all() and not ex[:, 20:].any()
    band, _ = run(hand, gsi, pts, trusted, exclude=ex)
    assert not band[:, :20].any()
    assert band[10:20, 21:45].all()


def test_read_segments_trusted_kinds(tmp_path):
    p = tmp_path / 'seg.csv'
    head = ('line,seg,x,y,len_m,dist_traj_m,side_by,out_nx,out_ny,status,kind,raw_offset_m,offset_m,confidence,'
            'n_candidates,swept_dropped,grass_inside_dropped,outward_blocked,dup\n')
    rows = [
        '0,0,1.0,2.0,1.0,1.0,area,0.0,1.0,snapped,step,0.5,0.5,0.8,1,0,0,0,False',
        '0,1,2.0,2.0,1.0,1.0,area,0.0,1.0,snapped,structure,0.3,0.3,0.6,1,0,0,0,False',
        '0,2,3.0,2.0,1.0,1.0,area,0.0,1.0,snapped,intensity,0.2,0.2,0.3,1,0,0,0,False',
        '0,3,4.0,2.0,1.0,1.0,area,0.0,1.0,rejected_cap,step,1.8,0.0,0.0,1,0,0,0,False',
        '0,4,5.0,2.0,1.0,1.0,area,0.0,1.0,no_evidence,none,0.0,0.0,0.0,0,0,0,0,True',
    ]
    p.write_text(head + '\n'.join(rows) + '\n')
    s = m.read_segments(p)
    assert s['trusted'].tolist() == [True, True, False, False, False]
    assert s['dup'].tolist() == [False, False, False, False, True]
    # 補正後の位置 = 中点 + offset × 外向きの法線
    assert np.allclose(s['mid'][0], [1.0, 2.5]) and np.allclose(s['mid'][3], [4.0, 2.0])
    pts, idx = m.sample_segments(s, 0.25)
    assert idx.max() == 4 and np.allclose(pts[idx == 0][:, 1], 2.5)
    assert np.isclose(pts[idx == 0][:, 0].min(), 0.5) and np.isclose(pts[idx == 0][:, 0].max(), 1.5)

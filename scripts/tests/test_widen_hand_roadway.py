#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_widen_hand_roadway の合成データでの単体テスト（#154、154w）。実データも地図も使わない。

確かめること：
  1. 赤・橙（snapped かつ step / structure）の区間に面する所だけ広げる。青（強度）・桃（未補正）は広げない。
     境目（赤と桃の点から等距離に近いセル）は総当たりで計算した判定と 1 セルも違わない
  2. 上限（--max-widen-m）を超えて広げない。手塗りは削らない
  3. 手塗りから離れた飛び地は足さない
  4. 段・構造物は帯より優先。横・斜めに 1 セルおきに並ぶ点も、太らせれば帯がその奥へ入らない（MF-1）
  5. 除外の箱（y を限った箱の向きを含む）、keepout
  6. 縁の外側のセル（補正後の面の階段）を落とす（SF-5）
  7. 赤・橙が 1 つも無ければ帯は空
  8. 区間の表の読み方（どの状態・根拠を赤・橙とみなすか、補正後の位置）、小さい穴の検出

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
TIE = 0.05


def y_of_row_top(r):
    """行 r の上の境の map の y。"""
    return (H - r) * RES


def scene():
    """横向きの道。手塗りは 20〜39 行、補正後の GSI の面は 10〜49 行（両側に 0.5 m 広い）。
    上の縁（10 行の上の境、外向きの法線は +y）は左半分が赤（段で寄せた）、右半分が桃（未補正）。
    下の縁（法線は −y）は青（強度だけ）。"""
    hand = np.zeros((H, W), bool)
    hand[20:40] = True
    gsi = np.zeros((H, W), bool)
    gsi[10:50] = True
    xs = np.arange(0, W * RES, 0.01)
    top = np.column_stack([xs, np.full(xs.size, y_of_row_top(10))])
    bot = np.column_stack([xs, np.full(xs.size, y_of_row_top(50))])
    pts = np.vstack([top, bot])
    trusted = np.concatenate([xs < W * RES / 2, np.zeros(xs.size, bool)])
    nv = np.vstack([np.tile([0.0, 1.0], (xs.size, 1)), np.tile([0.0, -1.0], (xs.size, 1))])
    return hand, gsi, pts, trusted, nv


def run(hand, gsi, pts, trusted, nv=None, barrier=None, exclude=None, keepout=None, cap=1.5, dil=0.0):
    z = np.zeros((H, W), bool)
    r = m.widen_band(hand, gsi, z if barrier is None else barrier, z if exclude is None else exclude,
                     pts, trusted, RES, OX, OY, cap, TIE, dil, nv, keepout)
    return r['band'], r


def test_only_red_orange_side_widens():
    hand, gsi, pts, trusted, nv = scene()
    band, _ = run(hand, gsi, pts, trusted, nv)
    # 上の左半分（赤）は手塗りの縁から GSI の縁まで全部
    assert band[10:20, :45].all()
    # 上の右半分（桃）と下（青）は広げない
    assert not band[10:20, 51:].any()
    assert not band[40:50].any()
    # 手塗りとは重ならない（手塗りは変えない）
    assert not (band & hand).any()


def test_tie_columns_match_brute_force():
    """赤と桃の境目（45〜50 列）の判定を、点の集合からの総当たりの距離と比べる。等距離なら広げない。"""
    hand, gsi, pts, trusted, nv = scene()
    band, _ = run(hand, gsi, pts, trusted, nv)
    rr, cc = np.mgrid[10:20, 40:56]
    x, y = m.cell_xy(rr, cc, H, RES, OX, OY)
    q = np.stack([x, y], -1)[..., None, :]
    dT = np.linalg.norm(q - pts[trusted], axis=-1).min(-1)
    dU = np.linalg.norm(q - pts[~trusted], axis=-1).min(-1)
    expect = dT + TIE < dU
    assert np.array_equal(band[10:20, 40:56], expect)
    # 45〜50 列に、広げるセルと広げないセルの両方がある（境目を実際に通っている）
    assert band[10:20, 45:51].any() and not band[10:20, 45:51].all()


def test_equidistant_cell_is_not_widened():
    """赤と桃の点から同じ距離のセルは、tie が 0 でも広げない（strict <）。"""
    hand = np.zeros((H, W), bool)
    hand[20:40] = True
    gsi = np.zeros((H, W), bool)
    gsi[19, 50] = True
    x, y = m.cell_xy(np.array([19]), np.array([50]), H, RES, OX, OY)
    pts = np.array([[x[0] - 0.1, y[0] + 0.1], [x[0] + 0.1, y[0] + 0.1]])
    z = np.zeros((H, W), bool)
    r = m.widen_band(hand, gsi, z, z, pts, np.array([True, False]), RES, OX, OY, 1.5, 0.0)
    assert not r['band'].any()
    r = m.widen_band(hand, gsi, z, z, pts[:1], np.array([True]), RES, OX, OY, 1.5, 0.0)
    assert r['band'][19, 50]


def test_no_trusted_points_gives_empty_band():
    hand, gsi, pts, trusted, nv = scene()
    band, r = run(hand, gsi, pts, np.zeros_like(trusted), nv)
    assert not band.any() and r['stages']['nearest_trusted'] == 0


def test_cap_limits_distance():
    hand, gsi, pts, trusted, nv = scene()
    band, _ = run(hand, gsi, pts, trusted, nv, cap=0.25)
    # 0.25 m = 5 セル。手塗りの縁（20 行）から 5 セル（15〜19 行）まで
    assert band[15:20, :45].all()
    assert not band[:15].any()


def test_island_not_touching_hand_is_dropped():
    hand, gsi, pts, trusted, nv = scene()
    gsi[17] = False   # 手塗りの外に、手塗りに接しない帯（10〜16 行）を作る
    band, r = run(hand, gsi, pts, trusted, nv)
    assert band[18:20, :45].all()
    assert not band[10:17].any()
    assert r['stages']['dropped_islands'] > 0


def test_barrier_wins_and_dotted_row_is_closed():
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[14, ::2] = True   # 横に 1 セルおきの構造物の点（10 cm ボクセルの並び）
    band, _ = run(hand, gsi, pts, trusted, nv, barrier=barrier)
    assert not (band & barrier).any()
    # 太らせないと、点の間を通って奥（10〜13 行）まで帯がつながる
    assert band[10:14, :45].any()
    band_d, _ = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05)
    assert not (band_d & barrier).any()
    assert not band_d[10:15].any()
    assert band_d[16:20, :45].all()


def test_diagonal_dotted_row_is_closed():
    """斜めに 2 セルおきに並ぶ点 (19-2j, 2j)。十字（楕円 3×3）で太らせると、点の間の (r+1, c+1) が残り、
    8 連結の帯が壁の向こう（左上の三角）へすり抜ける。正方形なら閉じる（MF-1）。"""
    import cv2
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    for j in range(0, 6):
        barrier[19 - 2 * j, 2 * j] = True
    rr, cc = np.mgrid[0:H, 0:W]
    # 壁の左上で、壁から 2 セル以上離れた所（10〜18 行）
    tri = (rr >= 10) & (rr <= 18) & (cc + rr < 19 - 1)
    assert tri.any()
    cross = cv2.dilate(barrier.astype(np.uint8),
                       cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))).astype(bool)
    band_cross, _ = run(hand, gsi, pts, trusted, nv, barrier=cross)
    assert (band_cross & tri).any()          # 十字ではすり抜ける（直す前の振る舞い）
    band_sq, _ = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05)
    assert not (band_sq & tri).any()         # 正方形では入らない
    assert not (band_sq & m.square_dilate(barrier, 1)).any()
    assert band_sq[10:20, 20:45].all()       # 壁の右は広がる


def test_exclude_box_and_keepout():
    hand, gsi, pts, trusted, nv = scene()
    # 左の 0〜1 m（0〜19 列）を箱で除く
    ex = m.box_mask((H, W), RES, OX, OY, (0.0, 1.0, 0.0, H * RES))
    assert ex[:, :20].all() and not ex[:, 20:].any()
    keep = np.zeros((H, W), bool)
    keep[15:20, 30:35] = True
    band, r = run(hand, gsi, pts, trusted, nv, exclude=ex, keepout=keep)
    assert not band[:, :20].any()
    assert not (band & keep).any()
    assert band[10:20, 21:30].all()
    # 箱が無いときの帯（記録用）には箱の中も入る
    assert r['band_nobox'][10:20, 1:19].all()


def test_box_mask_y_orientation():
    """y を限った箱：画像の 0 行が map の y 最大。y 0.5〜1.0 m は下から 10〜19 行目（行 40〜49）。"""
    b = m.box_mask((H, W), RES, OX, OY, (0.0, W * RES, 0.5, 1.0))
    rows = np.flatnonzero(b.any(axis=1))
    assert rows.min() == H - 20 and rows.max() == H - 11
    x, y = m.cell_xy(rows, np.zeros_like(rows), H, RES, OX, OY)
    assert (y >= 0.5).all() and (y <= 1.0).all()


def test_cells_beyond_snapped_edge_are_dropped():
    """補正後の面が縁の外へ 2 行はみ出していても（階段）、縁の外側のセルは帯に入れない（SF-5）。"""
    hand, gsi, pts, trusted, nv = scene()
    gsi[8:10] = True   # 縁（10 行の上の境）の外の 2 行
    band, r = run(hand, gsi, pts, trusted, nv)
    assert not band[8:10].any()
    assert band[10:20, :45].all()
    assert r['stages']['outside_snapped_edge'] > 0 and r['outside_edge'][8:10, :45].all()
    # 法線を渡さなければ落とさない（比較）
    band0, _ = run(hand, gsi, pts, trusted, None)
    assert band0[8:10, :45].all()


def test_small_holes():
    mm = np.ones((20, 20), bool)
    mm[5, 5] = False          # 1 セルの穴
    mm[10:13, 10:13] = False  # 9 セルの穴
    mm[0, 3] = False          # 端に接する欠け（穴ではない）
    h = m.small_holes(mm, 4)
    assert h[5, 5] and not h[11, 11] and not h[0, 3]
    assert m.small_holes(mm, 9)[11, 11]


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

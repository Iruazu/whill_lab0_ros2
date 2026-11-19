#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_widen_hand_roadway の合成データでの単体テスト（#154、154w）。実データも地図も使わない。

確かめること：
  1. 赤・橙（snapped かつ step / structure）の区間に面する所だけ広げる。青（強度）・桃（未補正）は広げない。
     境目（赤と桃の点から等距離に近いセル）は総当たりで計算した判定と 1 セルも違わない
  2. 上限（--max-widen-m）を超えて広げない。手塗りは削らない
  3. 手塗りから離れた飛び地は足さない
  4. 段・構造物は帯より優先。横・斜めに 2 セルごと（間に 1 セル）に並ぶ点も、太らせれば帯がその奥へ入らない（MF-1）
  5. 除外の箱（y を限った箱の向きを含む）、keepout
  6. 縁の外側のセル（補正後の面の階段）を落とす（SF-5）
  7. 赤・橙が 1 つも無ければ帯は空
  8. 区間の表の読み方（どの状態・根拠を赤・橙とみなすか、補正後の位置）、小さい穴の検出
  9. R6：帯の中の 3 セル以下の点は帯へ戻し、4 セル以上の塊・帯の縁にまたがる点は残す。
     --band-salt-gap-cells 1 では 2 セルごと（間に 1 セル）の点の格子を残し、その奥へ帯を入れない。
     gap は「隙間 gap セル以下」で連結する（隙間 1 と 2 を区別する）。引数は 0〜3 に限る
 10. R3：上限まで広がった区間を求め（集計と同じ割り当て）、drop ではその区間の帯を全部落とす。
     落とした区間の中の点は R6 で戻さず、障害物として残す

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


def run(hand, gsi, pts, trusted, nv=None, barrier=None, exclude=None, keepout=None, cap=1.5, dil=0.0,
        salt=0, drop=None, gap=0):
    z = np.zeros((H, W), bool)
    r = m.widen_band(hand, gsi, z if barrier is None else barrier, z if exclude is None else exclude,
                     pts, trusted, RES, OX, OY, cap, TIE, dil, nv, keepout, salt, drop, gap)
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
    barrier[14, ::2] = True   # 横に 2 セルごと（間に 1 セル）の構造物の点（10 cm ボクセルの並び）
    band, _ = run(hand, gsi, pts, trusted, nv, barrier=barrier)
    assert not (band & barrier).any()
    # 太らせないと、点の間を通って奥（10〜13 行）まで帯がつながる
    assert band[10:14, :45].any()
    band_d, _ = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05)
    assert not (band_d & barrier).any()
    assert not band_d[10:15].any()
    assert band_d[16:20, :45].all()


def test_diagonal_dotted_row_is_closed():
    """斜めに 2 セルごと（間に 1 セル）に並ぶ点 (19-2j, 2j)。十字（楕円 3×3）で太らせると、点の間の (r+1, c+1) が残り、
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


def test_band_salt_small_clusters_erased_large_kept():
    """R6：帯の中の 1・3 セルの点は帯へ戻し（太らせた分も帯に戻る）、4 セルの塊は残して周りを 1 セル空ける。"""
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[14, 5] = True                       # 1 セル
    barrier[14, 12:15] = True                   # 3 セル（横一列）
    barrier[14:16, 25:27] = True                # 4 セル（2×2）
    band0, r0 = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05)
    band, r = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3)
    # 前の振る舞い（salt=0）：どの点も帯から除き、周り 1 セルも空ける
    assert not (band0 & barrier).any() and not band0[13:16, 4:7].any() and r0['stages']['salt_cells'] == 0
    # 1・3 セルは帯へ戻り、周りの 1 セルも帯になる
    assert band[14, 5] and band[14, 12:15].all()
    assert band[13:16, 4:7].all() and band[13:16, 11:16].all()
    expect = barrier.copy()
    expect[:, 20:] = False                      # 戻すのは 1・3 セルの 4 セルだけ
    assert np.array_equal(r['salt'], expect)
    # 4 セルの塊は残り（帯に入らない）、周り 1 セルも帯に入らない
    assert not band[13:17, 24:28].any()
    assert r['barrier_kept'][14:16, 25:27].all() and not r['barrier_kept'][14, :20].any()
    assert r['stages']['salt_cells'] == 4
    # 戻した点の外は前と同じ（太らせた分が戻る以外に帯は変わらない）
    assert not (band0 & ~band).any()


def test_band_salt_straddling_edge_or_near_large_is_kept():
    """R6：成分の一部が候補の外（補正後の面の外）にある点、残す塊を太らせた所に掛かる点は戻さない。"""
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[9:11, 8] = True                     # 2 セル。9 行は補正後の面の外（縁にまたがる）
    barrier[14:16, 30:32] = True                # 4 セルの塊
    barrier[14, 33] = True                      # 塊から 1 セル空けた 1 セル（別の成分）
    barrier[17, 40] = True                      # 孤立した 1 セル（比較）
    band, r = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.10, salt=3)
    assert not r['salt'][9:11, 8].any() and r['barrier_kept'][9:11, 8].all()
    # 0.10 m = 2 セル太らせる。(14, 33) は塊を太らせた所（28〜33 列）に掛かるので残す
    assert not r['salt'][14, 33] and r['barrier_kept'][14, 33]
    assert r['salt'][17, 40] and band[17, 40]


def test_band_salt_hand_cells_count_as_room():
    """R6：手塗りと帯にまたがる 3 セル以下の点も戻す（手塗りの側は合成の規則で元から消える）。"""
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[19:21, 10] = True                   # 19 行は帯、20 行は手塗り
    band, r = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3)
    assert r['salt'][19, 10] and not r['salt'][20, 10]   # 帯の側だけが帯に入る
    assert not r['barrier_kept'].any()


def test_band_salt_gap_keeps_dot_lattice():
    """--band-salt-gap-cells 1：2 セルごと（間に 1 セル）の点の格子は 1 つの大きい成分とみなして残す。孤立点は戻す。"""
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[11:17:2, 4:14:2] = True             # 3×5 の格子（各点は生の 8 連結では 1 セル）
    barrier[15, 30] = True                      # 孤立点
    small_raw, _ = m.small_components(barrier, 3)
    small_gap, _ = m.small_components(barrier, 3, 1)
    assert small_raw[11:17:2, 4:14:2].all()     # 生の 8 連結では格子の点も 1 セルずつ ≤ 3
    assert not small_gap[11:17:2, 4:14:2].any() and small_gap[15, 30]
    _, r_raw = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3)
    _, r_gap = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3, gap=1)
    assert r_raw['salt'][11:17:2, 4:14:2].all()
    assert not r_gap['salt'][11:17:2, 4:14:2].any() and r_gap['salt'][15, 30]


def test_capped_segments_and_drop():
    """R3：上限まで広がった区間（赤の左半分）を集計と同じ割り当てで求め、drop で落とすと帯から消える。"""
    hand, gsi, pts, trusted, nv = scene()
    # 区間を 2 つに分ける：左（0〜1 m、行 10 の上の境）と右（1〜2.25 m）。どちらも赤。
    seg = {'mid': np.array([[0.5, y_of_row_top(10)], [1.625, y_of_row_top(10)]]),
           'nv': np.array([[0.0, 1.0], [0.0, 1.0]]), 'len': np.array([1.0, 1.25])}
    pts2, pidx = m.sample_segments(seg, 0.01)
    tr = np.ones(len(pts2), bool)
    dup = np.zeros(len(pts2), bool)
    gsi2 = gsi.copy()
    gsi2[10:20, 20:] = False
    gsi2[17:20, 20:] = True                     # 右の区間の面は手塗りから 0.15 m だけ
    gsi2[40:] = False                           # 下の縁（ここでは全部赤とみなす）の面は除く
    band1, r1 = run(hand, gsi2, pts2, tr, None, cap=0.5)
    lab = m.assign_segments(band1, pts2, tr, pidx, dup, H, RES, OX, OY)
    assert set(np.unique(lab[band1])) == {1, 2}
    cap = m.capped_segments(lab, r1['d_hand'], 4, 0.5 - RES / 2)
    assert cap.tolist() == [0]                  # 左だけが上限 0.5 m まで広がる
    drop = np.isin(lab, cap + 1)
    band2, r2 = run(hand, gsi2, pts2, tr, None, cap=0.5, drop=drop)
    assert not (band2 & drop).any() and band2[17:20, 20:45].all()
    assert not (band2 & ~band1).any()           # 2 回目は 1 回目の部分集合
    lab2 = m.assign_segments(band2, pts2, tr, pidx, dup, H, RES, OX, OY)
    assert m.capped_segments(lab2, r2['d_hand'], 4, 0.5 - RES / 2).size == 0
    assert r2['stages']['minus_capped'] < r2['stages']['minus_keepout']


def test_gap_connects_only_up_to_gap_cells():
    """gap = 1 は隙間 1 セル（差 2）までを連結し、隙間 2 セル（差 3）は連結しない。横・縦・斜めとも（SF-1）。
    中心に置いた半径 1 の正方形で太らせると、隙間 2 でも正方形どうしが接して連結になっていた。"""
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        for gap_cells, connected in ((1, True), (2, False)):
            mm = np.zeros((20, 20), bool)
            mm[10, 10] = True
            mm[10 + dr * (gap_cells + 1), 10 + dc * (gap_cells + 1)] = True
            small, _ = m.small_components(mm, 1, 1)
            assert (not small.any()) == connected, (dr, dc, gap_cells)
    # gap = 2 なら隙間 2 も連結する
    mm = np.zeros((20, 20), bool)
    mm[10, 10] = mm[10, 13] = True
    assert not m.small_components(mm, 1, 2)[0].any()
    # gap_dilate は元のセルを必ず含む
    assert (m.gap_dilate(mm, 1) >= mm).all()


def test_band_salt_lattice_gap0_vs_gap1_behind_cells():
    """test_barrier_wins_and_dotted_row_is_closed と同じ配置（14 行に 2 セルごとの点）で R6 を有効にする。
    gap 0：格子の点が 1 点ずつ戻され、太らせた壁が無くなって、格子の奥（10〜13 行）まで帯が入る。
    gap 1：格子は大きい障害物として残り、奥へは入らない（既定の振る舞い、MF-1）。"""
    hand, gsi, pts, trusted, nv = scene()
    barrier = np.zeros((H, W), bool)
    barrier[14, ::2] = True
    band0, r0 = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3, gap=0)
    band1, r1 = run(hand, gsi, pts, trusted, nv, barrier=barrier, dil=0.05, salt=3, gap=1)
    assert band0[10:14, :40].all() and r0['salt'][14, :40:2].all()
    assert not band1[10:15].any() and not r1['salt'].any()
    assert r1['barrier_kept'][14, ::2].all()
    assert band1[16:20, :45].all()


def test_band_salt_max_cells_arg_range():
    """--band-salt-max-cells は 0〜3（gen_auto_free_mask.py の 3 が上限）。4 は引数の段階で止まる（SF-2）。"""
    import pytest
    assert m.parse_args(['--band-salt-max-cells', '3']).band_salt_max_cells == 3
    assert m.parse_args([]).band_salt_gap_cells == 1
    with pytest.raises(SystemExit):
        m.parse_args(['--band-salt-max-cells', '4'])
    with pytest.raises(SystemExit):
        m.parse_args(['--band-salt-gap-cells', '-1'])


def test_capped_drop_with_band_salt():
    """R3 と R6：1 回目に戻した点が、落とす区間の中にあれば 2 回目は戻さず障害物として残す。
    残した点を太らせた所も帯に入らない（落とした区間の外の帯は変わらない）。"""
    hand, gsi, pts, trusted, nv = scene()
    seg = {'mid': np.array([[0.5, y_of_row_top(10)], [1.625, y_of_row_top(10)]]),
           'nv': np.array([[0.0, 1.0], [0.0, 1.0]]), 'len': np.array([1.0, 1.25])}
    pts2, pidx = m.sample_segments(seg, 0.01)
    tr = np.ones(len(pts2), bool)
    dup = np.zeros(len(pts2), bool)
    gsi2 = gsi.copy()
    gsi2[10:20, 20:] = False
    gsi2[17:20, 20:] = True
    gsi2[40:] = False
    barrier = np.zeros((H, W), bool)
    barrier[14, 8] = True                       # 左の区間（上限まで広がる）の中の 1 点
    barrier[18, 30] = True                      # 右の区間（落とさない）の中の 1 点
    band1, r1 = run(hand, gsi2, pts2, tr, None, barrier=barrier, cap=0.5, dil=0.05, salt=3)
    assert r1['salt'][14, 8] and r1['salt'][18, 30]
    lab = m.assign_segments(band1, pts2, tr, pidx, dup, H, RES, OX, OY)
    cap = m.capped_segments(lab, r1['d_hand'], 4, 0.5 - RES / 2)
    assert cap.tolist() == [0]
    drop = np.isin(lab, cap + 1)
    band2, r2 = run(hand, gsi2, pts2, tr, None, barrier=barrier, cap=0.5, dil=0.05, salt=3, drop=drop)
    assert not r2['salt'][14, 8] and r2['barrier_kept'][14, 8]
    assert r2['salt'][18, 30] and band2[18, 30]
    assert not (band2 & drop).any() and not (band2 & ~band1).any()

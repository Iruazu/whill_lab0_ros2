#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_gsi_edge_snap の合成データでの単体テスト（#202、202d）。通信も実データも使わない。

確かめること：
  1. 寄せ：段の線が GSI の縁の外 0.6 m にあれば、そこへ寄る（証拠のラスタから断面を取る経路ごと）
  2. 優先：段 > 構造物 > 強度。強度だけなら信頼度が低い。強度が段と一致すれば信頼度が上がる
  3. 上限：段は --max-offset-m、強度だけは --max-offset-intensity-m を超えたら寄せない
  4. 未補正：証拠が無ければ動かさない。窓の端から始まる帯（縁が帯の中）は使わない
  5. 連続性：近傍の中央値から飛ぶ区間・孤立した区間は却下し、残りをならす
  6. 外へ寄せる帯に構造物が点々とある候補、内側が芝の段の候補は落とす
  7. 頂点の移動、幅の測り方（半直線と線分）、舗装の続く長さ
  8. レビューで足したもの：内側の縁石が外の候補を止める（SF-1）、1 区間の線は寄せない（SF-3）、
     ならした値も根拠の上限を超えない（SF-4）、道の側の判定の 4 通り（SF-5）、タイルの重なりの検出

Run:
    python3 -m pytest scripts/tests/test_gsi_edge_snap.py -q
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import map_gsi_edge_snap as m  # noqa: E402

DS = 0.05
S = np.arange(-1.5, 1.5 + DS / 2, DS)


def _band(lo, hi):
    return ((S > lo) & (S < hi)).astype(float)


def test_barrier_edge_is_inner_boundary():
    sup = _band(0.6, 0.8)
    c = m.barrier_edges(sup, S, 0.8, 0.2)
    assert len(c) == 1 and abs(c[0] - 0.6) <= DS


def test_barrier_starting_at_window_edge_is_ignored():
    # 窓の内側の端（-1.5）から始まる帯は、縁がどこにあるか分からないので使わない
    sup = _band(-1.6, -1.2)
    assert m.barrier_edges(sup, S, 0.8, 0.2) == []


def test_intensity_transition():
    paved = (S < 0.3).astype(float)
    grass = 1 - paved
    c = m.intensity_edges(paved, grass, S, 0.4, 0.7)
    assert len(c) == 1 and abs(c[0] - 0.3) <= DS


def test_priority_and_confidence():
    off, kind, conf, st, _ = m.choose_evidence([0.6], [0.2], [0.1], 1.5, 1.0, 0.3)
    assert (kind, st) == ('step', 'snapped') and off == 0.6 and conf == 0.8
    off, kind, conf, st, _ = m.choose_evidence([0.6], [], [0.5], 1.5, 1.0, 0.3)
    assert kind == 'step' and conf == 0.95          # 強度の段が一致すると上がる
    off, kind, conf, st, _ = m.choose_evidence([], [], [-0.4], 1.5, 1.0, 0.3)
    assert kind == 'intensity' and conf == 0.3 and off == -0.4
    _, kind, _, _, _ = m.choose_evidence([], [0.7], [0.1], 1.5, 1.0, 0.3)
    assert kind == 'structure'


def test_nearest_candidate_wins_and_ambiguity_lowers_confidence():
    off, _, conf, _, n = m.choose_evidence([1.2, -0.3], [], [], 1.5, 1.0, 0.3)
    assert off == -0.3 and n == 2 and conf == 0.7


def test_caps():
    assert m.choose_evidence([1.6], [], [], 1.5, 1.0, 0.3)[3] == 'rejected_cap'
    assert m.choose_evidence([], [], [1.2], 1.5, 1.0, 0.3)[3] == 'rejected_cap'
    assert m.choose_evidence([], [], [0.9], 1.5, 1.0, 0.3)[3] == 'snapped'


def test_no_evidence_is_not_moved():
    off, kind, conf, st, _ = m.choose_evidence([], [], [], 1.5, 1.0, 0.3)
    assert (off, kind, conf, st) == (0.0, 'none', 0.0, 'no_evidence')


def test_smoothing_rejects_jump_and_isolated():
    off = np.array([0.3, 0.3, 0.35, 0.3, 1.2, 0.3, 0.25, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0])
    ok = np.array([1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0], bool)
    out, st = m.smooth_offsets(off, ok, 5, 0.5, 2)
    assert st[4] == 'rejected_jump'
    assert st[10] == 'rejected_isolated'
    assert all(st[i] == 'snapped' for i in range(8) if i != 4)
    assert np.allclose(out[[0, 1, 2, 3, 5, 6, 7]], 0.3, atol=0.03)
    assert out[4] == 0 and out[10] == 0


def test_swept_filter_drops_outward_over_structure():
    struct = _band(0.2, 0.25) + _band(0.45, 0.5)        # 駐輪の帯の点々（約 10 %）
    step = np.zeros_like(S)
    keep, drop = m.filter_swept([0.8, -0.4], S, struct, step, 0.03, 0.1, 0.1)
    assert keep == [-0.4] and drop == 1                   # 内へ寄せる候補は見ない
    keep, drop = m.filter_swept([0.8], S, np.zeros_like(S), step, 0.03, 0.1, 0.1)
    assert keep == [0.8] and drop == 0


def test_inside_grass_filter():
    grass = _band(0.0, 0.6)                               # 舗装の端 0 の外に芝、0.6 に生垣の段
    keep, drop = m.filter_inside_grass([0.6], S, grass, 0.4, 0.5)
    assert keep == [] and drop == 1
    keep, _ = m.filter_inside_grass([0.0], S, grass, 0.4, 0.5)
    assert keep == [0.0]


def test_snap_from_raster_profile():
    """証拠のラスタ → 断面の標本 → 候補 → 選択 の経路で、外 0.6 m の段の線へ寄る。"""
    res, H, W = 0.05, 200, 200
    ox, oy = 0.0, 0.0
    ev = np.zeros((H, W), np.uint8)
    ev[:, :] = m.B_PAVED | m.B_GROUND
    # y ≈ 5.5〜5.65 m の 3 セルの横線に段（行は上が y 最大）。GSI の縁は y = 5.0、道は y < 5
    row = int(m.mr.xy_to_rc(np.array([[0.0, 5.575]]), H, res, ox, oy)[0][0])
    ev[row - 1:row + 2, :] |= m.B_STEP
    y_inner = oy + (H - 1 - (row + 1)) * res              # 段の帯の一番下のセルの下端
    mid = np.array([[5.0, 5.0]])
    tng = np.array([[1.0, 0.0]])
    u = np.array([[0.0, 1.0]])                            # 外向き（道の外が正）
    along = np.array([0.1, 0.3, 0.5, 0.7, 0.9])
    P = (mid[:, None, None, :] + (along - 0.5)[None, :, None, None] * tng[:, None, None, :]
         + S[None, None, :, None] * u[:, None, None, :])
    code = m.sample(ev, P, H, res, ox, oy)
    sup = ((code & m.B_STEP) > 0).mean(axis=1)[0]
    c = m.barrier_edges(sup, S, 0.8, 0.2)
    off, kind, _, st, _ = m.choose_evidence(c, [], [], 1.5, 1.0, 0.3)
    assert st == 'snapped' and kind == 'step'
    assert abs(off - (y_inner - 5.0)) <= res              # 段の帯の内側の境（標本の間隔の誤差まで）


def test_densify_keeps_vertices_and_zero_shift_is_identity():
    P = np.array([[0.0, 0.0], [2.5, 0.0], [2.5, 1.2]])
    V = m.densify(P, 1.0)
    assert any(np.allclose(v, [2.5, 0.0]) for v in V)
    assert np.max(np.hypot(*np.diff(V, axis=0).T)) <= 1.0 + 1e-9
    _, _, n, _ = m.segment_frames(V)
    assert np.allclose(m.shift_vertices(V, n, np.zeros(len(n))), V)


def test_shift_vertices_moves_line():
    V = m.densify(np.array([[0.0, 0.0], [5.0, 0.0]]), 1.0)
    _, _, n, _ = m.segment_frames(V)
    V2 = m.shift_vertices(V, n, np.full(len(n), 0.5))
    assert np.allclose(V2[:, 1], 0.5)
    # 半分だけ寄せると、境の頂点は半分（0.25）で滑らかにつながる
    off = np.r_[np.full(2, 0.5), np.zeros(len(n) - 2)]
    V3 = m.shift_vertices(V, n, off)
    assert np.isclose(V3[2, 1], 0.25)


def test_ray_width_between_parallel_edges():
    A = np.array([[-10.0, 3.0], [-10.0, -2.5]])
    B = np.array([[10.0, 3.0], [10.0, -2.5]])
    P = np.array([[0.0, 0.0], [5.0, 0.0]])
    up = np.array([[0.0, 1.0], [0.0, 1.0]])
    l_ = m.ray_hits(P, up, A, B, 10.0)
    r_ = m.ray_hits(P, -up, A, B, 10.0)
    assert np.allclose(l_ + r_, 5.5)
    assert np.isnan(m.ray_hits(P, up, A, B, 2.0)).all()   # 上限より遠ければ当たらない


def test_paved_run():
    codes = np.r_[np.full(40, m.B_PAVED), np.full(20, m.B_GRASS)].astype(np.uint8)
    assert math.isclose(m.paved_run(codes, 0.05, 6), 2.0)
    # 1 セルの継ぎ目（マンホール）は越える
    codes2 = codes.copy()
    codes2[20] = 0
    assert math.isclose(m.paved_run(codes2, 0.05, 6), 2.0)
    # 分からない所で止まったら測れない
    codes3 = np.r_[np.full(40, m.B_PAVED), np.zeros(20)].astype(np.uint8)
    assert math.isnan(m.paved_run(codes3, 0.05, 6))


# --- 202d のレビューで足したもの ---------------------------------------------------------

def test_inner_curb_blocks_outward_candidate():
    """内 −0.8 m に縁石、外 +0.5 m に別の段：外を選んで縁石を道に含めない（SF-1）。"""
    sc, tc, ic, n = m.block_outward([-0.8, 0.5], [], [0.3])
    assert sc == [-0.8] and ic == [] and n == 2
    off, kind, _, st, _ = m.choose_evidence(sc, tc, ic, 1.5, 1.0, 0.3)
    assert (off, kind, st) == (-0.8, 'step', 'snapped')
    # 内側の構造物も外の段を止める
    sc, tc, _, n = m.block_outward([0.5], [-0.4], [])
    assert sc == [] and tc == [-0.4] and n == 1
    # 内側に段・構造物が無ければ外の候補はそのまま（強度の段は外を止めない）
    sc, tc, ic, n = m.block_outward([0.5], [], [-0.3])
    assert sc == [0.5] and ic == [-0.3] and n == 0


def test_short_line_is_not_snapped():
    """1 区間だけの線は孤立の判定を素通りしない（SF-3）。"""
    out, st = m.smooth_offsets(np.array([0.4]), np.array([True]), 5, 0.5, 2)
    assert st[0] == 'rejected_short_line' and out[0] == 0
    # 2 区間の線で両方寄せたなら通る
    out, st = m.smooth_offsets(np.array([0.4, 0.5]), np.array([True, True]), 5, 0.5, 2)
    assert list(st) == ['snapped', 'snapped'] and np.allclose(out, 0.45)


def test_smoothed_value_is_capped_per_kind():
    """段の大きな値に引かれても、強度だけの区間は強度の上限を超えない（SF-4）。"""
    off = np.array([1.3, 1.3, 0.95, 1.3, 1.3])
    cap = np.array([1.5, 1.5, 1.0, 1.5, 1.5])
    out, st = m.smooth_offsets(off, np.ones(5, bool), 5, 0.5, 2, cap)
    assert all(x == 'snapped' for x in st)
    assert out[2] == 1.0 and np.isclose(out[0], 1.3)
    out, _ = m.smooth_offsets(-off, np.ones(5, bool), 5, 0.5, 2, cap)
    assert out[2] == -1.0


def test_decide_side_four_cases():
    """道が左（法線の正）・右・両側・判定不能の 4 通り（SF-5）。"""
    nrm = np.array([[0.0, 1.0]] * 4)
    plus = np.array([1.0, 0.0, 1.0, 0.0])
    minus = np.array([0.0, 1.0, 1.0, 0.0])
    # 3 番目（両側とも道）は中心線が正の側 2 m、4 番目（両側とも道でない）は中心線が 20 m 先
    vc = np.array([[0.0, 3.0], [0.0, -3.0], [0.0, 2.0], [0.0, 20.0]])
    dc = np.hypot(vc[:, 0], vc[:, 1])
    sign, how, known = m.decide_side(plus, minus, vc, dc, nrm, 10.0)
    assert list(sign[:3]) == [1.0, -1.0, 1.0]
    assert list(how) == ['road_area', 'road_area', 'centerline', '']
    assert list(known) == [True, True, True, False]
    # 中心線が法線とほぼ直交（道の延長方向）なら中心線でも決めない
    sign, how, known = m.decide_side([1.0], [1.0], [[3.0, 0.1]], [3.0], [[0.0, 1.0]], 10.0)
    assert not known[0] and how[0] == ''


def test_find_duplicates_across_lines_only():
    xy = np.array([[0.0, 0.0], [0.3, 0.0], [0.2, 0.05], [0.0, 3.0]])
    nv = np.array([[0.0, 1.0]] * 4)
    line = np.array([0, 0, 1, 2])
    d = m.find_duplicates(xy, nv, line, 0.5)
    # 同じ線の隣（1）は重なりではない。別の線（2）は 0 か 1 の重なり。3 m 離れた縁（3）は別
    assert d[0] == -1 and d[1] == -1 and d[2] in (0, 1) and d[3] == -1
    # 向きが逆（道の反対側の縁）なら重なりではない
    d = m.find_duplicates(xy[:3], np.array([[0, 1.0], [0, 1.0], [0, -1.0]]), line[:3], 0.5)
    assert d[2] == -1

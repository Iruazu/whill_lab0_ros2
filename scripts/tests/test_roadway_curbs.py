#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_roadway_from_curbs.detect_curbs の合成データでの単体テスト（#202）。

static.pcd は z が 10 cm 刻み（ボクセルの中心）なので、検出器が分けるべきものは
  1. 10 cm の段（縁石 1 段）→ 段の線に沿って縁石として出る
  2. 3 cm の段（路面の凹凸）→ 出ない
  3. 量子化の境目にある平らな路面の揺れ（隣のセルと 10 cm 違う）→ 出ない（平均の面の条件）
  4. 構造物層のあるセル → 縁石から除く
の 4 つ。実データの閾値（既定値）のまま確かめる。
坂の等高線（数 m ごとの 1 段）は局所的には縁石と同じ形で、この検出器では落とせない
（docs/maps/campus-m8/README.md §3「縁石からの自動抽出の試作」）。そのためここでは試さない。

Run:
    python3 -m pytest scripts/tests/test_roadway_curbs.py -q
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from map_roadway_from_curbs import detect_curbs, parse_args  # noqa: E402

RES = 0.05
H, W = 80, 120
STEP_COL = 60


def _detect(ground, structure=None):
    a = parse_args([])   # 既定値をそのまま使う（スクリプトの既定と試験の閾値をずらさない）
    known = np.ones((H, W), bool)
    if structure is None:
        structure = np.zeros((H, W), bool)
    curb, high = detect_curbs(ground.astype(np.float32), known, structure, RES, a.window,
                              a.curb_dz_min, a.curb_dz_max, a.curb_slope_min,
                              a.trend_window, a.trend_dz_min)
    return curb, high


def _step(dz):
    g = np.zeros((H, W), np.float32)
    g[:, STEP_COL:] = dz
    return g


def _interior(mask):
    # 端の 15 セル（0.75 m）は窓・平均が画像の外に掛かるので見ない。
    return mask[15:-15, 15:-15]


def test_10cm_step_is_a_curb_line():
    curb, high = _detect(_step(0.10))
    inner = _interior(curb)
    cols = np.nonzero(inner.any(axis=0))[0] + 15
    # 段の線（列 STEP_COL-1 と STEP_COL の境）から 2 セル以内にだけ出る。
    assert cols.size > 0
    assert np.all(np.abs(cols - (STEP_COL - 0.5)) <= 2.5), cols
    # どの行でも途切れずに出る（領域成長の壁として線が続く）。
    assert inner.any(axis=1).all()
    assert not _interior(high).any()


def test_3cm_step_is_not_a_curb():
    curb, high = _detect(_step(0.03))
    assert not _interior(curb).any()
    assert not _interior(high).any()


def test_quantization_flicker_is_not_a_curb():
    # 路面が量子化の境目（z=0）にあり、0.1 m のボクセル（2×2 セル）ごとに -0.05 / +0.05 の
    # どちらかになる面。隣り合うセルで 10 cm の段が至る所に出るが、縁石ではない。
    rng = np.random.default_rng(202)
    blocks = rng.integers(0, 2, size=(H // 2, W // 2)).astype(np.float32) * 0.1 - 0.05
    g = np.kron(blocks, np.ones((2, 2), np.float32))
    curb, _ = _detect(g)
    assert not _interior(curb).any()


def test_structure_cells_are_excluded():
    structure = np.zeros((H, W), bool)
    structure[:, STEP_COL - 3:STEP_COL + 3] = True
    curb, _ = _detect(_step(0.10), structure)
    assert not (curb & structure).any()

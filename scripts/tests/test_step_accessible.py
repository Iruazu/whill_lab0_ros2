#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Synthetic unit test for compute_step_accessible_side (Rule 3).

Campus verification (M1) cannot exercise ditches (negative obstacles)
because no side drain exists on the surveyed route. This synthetic test
covers the Rule 3 accessible-side logic on BOTH sides of a step:

  1. Curb (upstep from chair side): accessible = LOWER side (valley).
  2. Ditch (downstep from chair side): accessible = UPPER side.

Constructs a small synthetic terrain, calls
compute_step_accessible_side, asserts the occupied mask lands on the
CHAIR-ACCESSIBLE side (nearer to the artificial "trajectory" cell)
in both cases.

Run:
    python3 scripts/tests/test_step_accessible.py
"""

import sys
from pathlib import Path

# Bring the parent scripts/ directory onto PYTHONPATH so we can import
# from pcd_to_occupancy_v2 without installing anything.
SCRIPT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPT_DIR))

import numpy as np

from pcd_to_occupancy_v2 import compute_step_accessible_side


def build_terrain(kind, W=20, H=10, step_col=10, drop_m=0.15):
    """Build a synthetic (H, W) ground_z map with a step at column `step_col`.

    kind='curb':  left half is z=0.0 (road / chair side), right half is
                  z=+drop (sidewalk / higher).
    kind='ditch': left half is z=0.0 (road / chair side), right half is
                  z=-drop (drain / lower).
    """
    ground_z = np.zeros((H, W), dtype=np.float32)
    if kind == 'curb':
        ground_z[:, step_col:] = +drop_m
    elif kind == 'ditch':
        ground_z[:, step_col:] = -drop_m
    else:
        raise ValueError(kind)
    valid = np.ones((H, W), dtype=bool)   # all cells known
    return ground_z, valid


def build_accessibility(W=20, H=10, traj_col=3):
    """Higher score = closer to the fictional chair path (column `traj_col`).

    Matches the negated-distance-transform semantics of the real pipeline.
    A column at traj_col scores 0 (closest); columns further away score
    negative values proportional to distance.
    """
    cols = np.arange(W).reshape(1, W).repeat(H, axis=0)
    dist_from_traj = np.abs(cols - traj_col).astype(np.float32)
    return -dist_from_traj


def test_curb_lower_side():
    """Chair on the LOW side of a curb → occupied on the LOW (valley) side."""
    ground, valid = build_terrain('curb')
    access = build_accessibility(traj_col=3)   # chair on col 3, low side
    occ = compute_step_accessible_side(ground, valid, access, step_threshold=0.03)
    # Cells at the LOW side just BEFORE the step should be occupied.
    left_side = occ[:, 9]   # step_col=10, so col 9 is the last LOW cell
    right_side = occ[:, 10]
    assert left_side.any(), (
        f'curb: expected occupied on LOW side (col 9), got {left_side}')
    assert not right_side.any(), (
        f'curb: expected NO occupied on HIGH side (col 10), got {right_side}')
    print('PASS  curb: occupied on lower side (chair-accessible = valley)')


def test_ditch_upper_side():
    """Chair on the HIGH side of a ditch → occupied on the HIGH side (Rule 3
    inverse of the naive "valley" rule)."""
    ground, valid = build_terrain('ditch')
    access = build_accessibility(traj_col=3)   # chair on col 3, HIGH side
    occ = compute_step_accessible_side(ground, valid, access, step_threshold=0.03)
    # Cells at the HIGH side just BEFORE the step should be occupied.
    left_side = occ[:, 9]   # last HIGH cell
    right_side = occ[:, 10]  # first LOW cell (ditch bottom)
    assert left_side.any(), (
        f'ditch: expected occupied on HIGH side (col 9, chair-accessible), '
        f'got {left_side}')
    assert not right_side.any(), (
        f'ditch: expected NO occupied on LOW side (col 10, ditch bottom), '
        f'got {right_side}')
    print('PASS  ditch: occupied on upper side (chair-accessible ≠ valley)')


def test_no_trajectory_falls_back_to_valley():
    """Without accessibility info (all zeros), tie-break must be valley side."""
    ground, valid = build_terrain('curb')
    access = np.zeros((10, 20), dtype=np.float32)   # no accessibility signal
    occ = compute_step_accessible_side(ground, valid, access, step_threshold=0.03)
    left_side = occ[:, 9]   # LOW side of curb
    right_side = occ[:, 10]
    assert left_side.any(), (
        f'no-traj fallback: expected occupied on VALLEY (col 9), got {left_side}')
    assert not right_side.any(), (
        f'no-traj fallback: expected NO occupied on ridge (col 10), got {right_side}')
    print('PASS  no-traj: falls back to valley side (curb-safe default)')


def test_no_step_below_threshold():
    """A 1 cm step must NOT be flagged when step_threshold = 0.03 m."""
    ground, valid = build_terrain('curb', drop_m=0.01)   # < 3 cm
    access = build_accessibility(traj_col=3)
    occ = compute_step_accessible_side(ground, valid, access, step_threshold=0.03)
    assert not occ.any(), (
        f'sub-threshold step: expected NO occupied cells anywhere, got {int(occ.sum())}')
    print('PASS  sub-threshold: 1 cm step correctly ignored at threshold=3 cm')


def main():
    tests = [
        test_curb_lower_side,
        test_ditch_upper_side,
        test_no_trajectory_falls_back_to_valley,
        test_no_step_below_threshold,
    ]
    for t in tests:
        t()
    print(f'\n{len(tests)}/{len(tests)} tests PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

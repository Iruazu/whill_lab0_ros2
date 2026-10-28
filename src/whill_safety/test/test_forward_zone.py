# SPDX-License-Identifier: BSD-3-Clause
"""Layer D zone geometry (failsafe_node.forward_zone_blocked).

The sector cases pin the pre-2026-10-28 behaviour; the rectangle cases
pin the avoidance-mode geometry that lets the chair pass beside an
obstacle without a stop.
"""
import math

from whill_safety.failsafe_node import (
    FORWARD_POINT_COUNT_MIN, FORWARD_SECTOR_HALF_ANGLE_RAD,
    FORWARD_SECTOR_MAX_M, FORWARD_SECTOR_MIN_M, LAYER_D_RECT_HALF_WIDTH_M,
    forward_zone_blocked)

INC = math.radians(0.5)
N = int(round(2 * math.pi / INC))
ANGLE_MIN = -math.pi


def _scan(points):
    """Build a 360° scan with returns at (angle_deg, range_m)."""
    ranges = [math.inf] * N
    for deg, r in points:
        # Modulo so +180° wraps onto the -180° bin, as on a real 360° scan.
        ranges[int(round((math.radians(deg) - ANGLE_MIN) / INC)) % N] = r
    return ranges


def _blocked(points, shape):
    return forward_zone_blocked(
        _scan(points), ANGLE_MIN, INC, shape,
        FORWARD_SECTOR_MIN_M, FORWARD_SECTOR_MAX_M,
        FORWARD_SECTOR_HALF_ANGLE_RAD, LAYER_D_RECT_HALF_WIDTH_M,
        FORWARD_POINT_COUNT_MIN)


def test_empty_scan_is_clear():
    assert not _blocked([], 'sector')
    assert not _blocked([], 'rectangle')


def test_dead_ahead_blocks_both():
    assert _blocked([(0.0, 1.5)], 'sector')
    assert _blocked([(0.0, 1.5)], 'rectangle')


def test_outside_distance_band_is_clear():
    for r in (0.9, 2.1):
        assert not _blocked([(0.0, r)], 'sector')
        assert not _blocked([(0.0, r)], 'rectangle')


def test_beside_path_blocks_sector_only():
    # 25° at 1.9 m: x = 1.72, y = 0.80. Inside the ±30° sector but 0.8 m
    # off the centreline, i.e. an obstacle the chair would pass beside.
    assert _blocked([(25.0, 1.9)], 'sector')
    assert not _blocked([(25.0, 1.9)], 'rectangle')


def test_rectangle_edges():
    # y just inside / outside the 0.5 m half-width at x ~= 1.5 m.
    inside = math.degrees(math.atan2(0.45, 1.5))
    outside = math.degrees(math.atan2(0.60, 1.5))
    assert _blocked([(inside, math.hypot(1.5, 0.45))], 'rectangle')
    assert not _blocked([(outside, math.hypot(1.5, 0.60))], 'rectangle')


def test_rear_returns_never_block():
    assert not _blocked([(180.0, 1.5)], 'sector')
    assert not _blocked([(180.0, 1.5)], 'rectangle')


def test_count_threshold():
    pts = [(0.0, 1.5), (1.0, 1.5)]
    assert forward_zone_blocked(
        _scan(pts), ANGLE_MIN, INC, 'sector', 1.0, 2.0,
        FORWARD_SECTOR_HALF_ANGLE_RAD, 0.5, 2)
    assert not forward_zone_blocked(
        _scan(pts[:1]), ANGLE_MIN, INC, 'sector', 1.0, 2.0,
        FORWARD_SECTOR_HALF_ANGLE_RAD, 0.5, 2)

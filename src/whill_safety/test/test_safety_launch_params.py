# SPDX-License-Identifier: BSD-3-Clause
"""safety_launch.py mode -> failsafe_node parameters (C3 wiring).

avoidance:=false must add nothing beyond use_sim_time, so failsafe_node
runs on its built-in defaults exactly as the 2026-08 demo did.
"""
import importlib.util
import math
import os

import pytest

from whill_safety import failsafe_node as fs

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch',
                       'safety_launch.py')
_spec = importlib.util.spec_from_file_location('safety_launch', _LAUNCH)
safety_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(safety_launch)
TOPICS = safety_launch._DEFAULT_WATCH_TOPICS


def test_avoidance_false_adds_nothing():
    assert safety_launch.failsafe_mode_params('false', TOPICS) == {}
    assert safety_launch.failsafe_mode_params('False', '') == {}


def test_node_defaults_are_the_demo_values():
    # The values failsafe_node falls back to when the launch passes nothing.
    assert fs.FORWARD_SECTOR_HALF_ANGLE_RAD == math.radians(30.0)
    assert (fs.FORWARD_SECTOR_MIN_M, fs.FORWARD_SECTOR_MAX_M) == (1.0, 2.0)
    assert fs.FORWARD_POINT_COUNT_MIN == 1
    assert fs.FORWARD_CLEAR_HYSTERESIS_S == 0.5
    assert fs.PERCEPTION_TIMEOUT_S == 2.0


def test_avoidance_true_is_rectangle_and_watches_cm_inputs():
    p = safety_launch.failsafe_mode_params('true', TOPICS)
    assert p['layer_d_shape'] == 'rectangle'
    assert p['layer_d_rect_half_width_m'] == 0.5
    assert p['extra_perception_topics'] == [
        '/velodyne_points_hires_no_ground', '/camera/camera/depth/color/points']
    # Band unchanged: forward_min/max come from the node defaults.
    assert 'forward_min_m' not in p and 'forward_max_m' not in p


def test_avoidance_true_empty_watch_list_omits_key():
    assert 'extra_perception_topics' not in \
        safety_launch.failsafe_mode_params('true', ' , ')


def test_typo_rejected():
    with pytest.raises(RuntimeError):
        safety_launch.failsafe_mode_params('ture', TOPICS)

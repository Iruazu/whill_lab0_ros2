# SPDX-License-Identifier: BSD-3-Clause
"""Collision Monitor config (avoidance:=true) stays tied to the footprint.

collision_monitor.yaml sizes its polygons from nav2_params.yaml
robot_radius. If someone changes the footprint (e.g. to a measured polygon)
these tests fail so the safety zones are re-derived instead of drifting.
"""
import os

import yaml

CFG = os.path.join(os.path.dirname(__file__), '..', 'config')


def _load(name):
    with open(os.path.join(CFG, name)) as f:
        return yaml.safe_load(f)


def _cm():
    return _load('collision_monitor.yaml')['collision_monitor']['ros__parameters']


def _radius():
    nav = _load('nav2_params.yaml')
    radii = {nav[c][c]['ros__parameters']['robot_radius']
             for c in ('local_costmap', 'global_costmap')}
    assert len(radii) == 1, f'local/global robot_radius differ: {radii}'
    return radii.pop()


def _box(points):
    xs, ys = points[0::2], points[1::2]
    return min(xs), max(xs), min(ys), max(ys)


def test_stop_polygon_from_footprint():
    r = _radius()
    x0, x1, y0, y1 = _box(_cm()['PolygonStop']['points'])
    assert (x0, y0, y1) == (0.0, -r, r)
    assert abs(x1 - (r + 0.35)) < 1e-9


def test_slow_polygon_contains_stop_and_reaches_layer_d():
    r = _radius()
    cm = _cm()
    sx0, sx1, sy0, sy1 = _box(cm['PolygonStop']['points'])
    x0, x1, y0, y1 = _box(cm['PolygonSlow']['points'])
    assert x0 <= sx0 and x1 > sx1 and y0 < sy0 and y1 > sy1
    assert abs(x1 - (r + 1.20)) < 1e-9 and abs(y1 - (r + 0.20)) < 1e-9
    # Layer D starts at 1.0 m: the chair should already be slowed there.
    assert x1 >= 1.0
    assert 0.0 < cm['PolygonSlow']['slowdown_ratio'] < 1.0


def test_approach_uses_footprint_radius():
    fa = _cm()['FootprintApproach']
    assert fa['type'] == 'circle' and fa['radius'] == _radius()
    assert fa['action_type'] == 'approach'


def test_wiring_and_sources():
    cm = _cm()
    assert cm['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == '/whill/controller/cmd_vel'
    assert cm['base_frame_id'] == 'base_link'
    assert set(cm['observation_sources']) == {'lidar', 'depth'}
    for s in cm['observation_sources']:
        assert cm[s]['type'] == 'pointcloud' and cm[s]['enabled']
    # Humble ignores a stale source instead of stopping; failsafe stops after
    # perception_timeout_s = 2.0 s. The source must outlive that.
    assert cm['source_timeout'] > 2.0

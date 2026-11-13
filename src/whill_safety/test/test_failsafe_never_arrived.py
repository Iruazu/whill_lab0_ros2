# SPDX-License-Identifier: BSD-3-Clause
"""Issue #159: inputs that never arrive (failsafe_node startup_grace_sec).

The default (0.0) must reproduce the pre-#159 behaviour exactly: an input
that never arrives does not stop the chair. With a grace > 0 every B/C/D
input that has received nothing stops it until its first message.

Time is injected by replacing FailsafeNode._now_s, so the tests need an
rclpy context but no ROS graph traffic and no waiting.
"""
import math

import pytest
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2

from whill_safety.failsafe_node import (
    PERCEPTION_TIMEOUT_S, STARTUP_GRACE_S_DEFAULT, FailsafeNode,
    never_arrived_layers)

T0 = 1000.0
NEVER = ('B:pcl_pose_never_arrived', 'C:perception_never_arrived',
         'D:scan_never_arrived')


class _FakePub:
    def __init__(self):
        self.count = 0

    def publish(self, _msg):
        self.count += 1


class _Clock:
    def __init__(self, t):
        self.t = t


@pytest.fixture
def make_node(monkeypatch):
    """make_node(grace=None, t0=T0) -> (node, clock, pub)."""
    made = []

    def _make(grace=None, t0=T0):
        clock = _Clock(t0)
        monkeypatch.setattr(FailsafeNode, '_now_s', lambda self: clock.t)
        args = ['--ros-args']
        if grace is not None:
            args += ['-p', f'startup_grace_sec:={grace}']
        rclpy.init(args=args)
        try:
            node = FailsafeNode()
        except Exception:
            rclpy.shutdown()
            raise
        pub = _FakePub()
        node._pub = pub
        made.append(node)
        return node, clock, pub

    yield _make
    for node in made:
        node.destroy_node()
        rclpy.try_shutdown()


def _step(node, clock, t):
    clock.t = t
    node._tick()
    return node._active_layers()


# --- pure helper ---------------------------------------------------------

def test_helper_disabled_when_grace_is_zero():
    assert never_arrived_layers(1e9, 0.0, 0.0, {'X': None}) == []


def test_helper_waits_for_grace_then_lists_missing_only():
    last = {'B': None, 'C': 5.0, 'D': None}
    assert never_arrived_layers(T0 + 4.99, T0, 5.0, last) == []
    assert never_arrived_layers(T0 + 5.0, T0, 5.0, last) == ['B', 'D']


def test_helper_does_nothing_before_the_clock_starts():
    assert never_arrived_layers(1e9, None, 5.0, {'B': None}) == []


# --- node: default keeps the old behaviour -------------------------------

def test_default_is_off():
    assert STARTUP_GRACE_S_DEFAULT == 0.0


def test_default_never_stops_for_missing_inputs(make_node):
    node, clock, pub = make_node()
    for t in (T0 + 1.0, T0 + 11.0, T0 + 3600.0):
        assert _step(node, clock, t) == ()
    assert pub.count == 0


# --- node: grace > 0 -----------------------------------------------------

def test_stops_after_grace_when_nothing_arrived(make_node):
    node, clock, pub = make_node(grace=5.0)
    assert _step(node, clock, T0 + 4.9) == ()
    assert pub.count == 0
    assert _step(node, clock, T0 + 5.0) == NEVER
    assert pub.count == 1
    # Stays engaged (20 Hz in the real timer) while nothing arrives.
    assert _step(node, clock, T0 + 60.0) == NEVER
    assert pub.count == 2


def test_each_input_clears_on_its_first_message(make_node):
    node, clock, pub = make_node(grace=5.0)
    _step(node, clock, T0 + 6.0)

    clock.t = T0 + 6.1
    node._on_pcl_pose(PoseWithCovarianceStamped())
    assert _step(node, clock, T0 + 6.1) == NEVER[1:]

    node._on_perception(PointCloud2())
    assert _step(node, clock, T0 + 6.1) == NEVER[2:]

    node._on_scan(LaserScan())  # empty scan: nothing in the Layer D zone
    pub.count = 0
    assert _step(node, clock, T0 + 6.1) == ()
    assert pub.count == 0


def test_input_that_arrived_in_time_never_trips_it(make_node):
    node, clock, _ = make_node(grace=5.0)
    clock.t = T0 + 1.0
    node._on_pcl_pose(PoseWithCovarianceStamped())
    node._on_perception(PointCloud2())
    node._on_scan(LaserScan())
    # Keep the pose and perception fresh so only the #159 labels could fire.
    for t in (T0 + 5.0, T0 + 30.0):
        clock.t = t
        node._on_pcl_pose(PoseWithCovarianceStamped())
        node._on_perception(PointCloud2())
        assert _step(node, clock, t) == ()


def test_late_silence_still_uses_the_existing_watchdog(make_node):
    # After the first message the input is "armed"; going silent later is
    # the pre-existing C:perception_silent, not a never-arrived label.
    node, clock, _ = make_node(grace=5.0)
    clock.t = T0 + 1.0
    node._on_pcl_pose(PoseWithCovarianceStamped())
    node._on_perception(PointCloud2())
    node._on_scan(LaserScan())
    t = T0 + 1.0 + PERCEPTION_TIMEOUT_S + 0.1
    clock.t = t
    node._on_pcl_pose(PoseWithCovarianceStamped())
    assert _step(node, clock, t) == ('C:perception_silent',)


def test_grace_waits_for_sim_clock(make_node):
    # use_sim_time before the first /clock: now() is 0. The grace starts at
    # the first non-zero tick, not at 0, so a bag stamped 1e9 s does not
    # trip it on its first tick.
    node, clock, pub = make_node(grace=5.0, t0=0.0)
    assert _step(node, clock, 0.0) == ()
    assert _step(node, clock, 1e9) == ()
    assert _step(node, clock, 1e9 + 4.9) == ()
    assert _step(node, clock, 1e9 + 5.0) == NEVER


# '.nan' is YAML for a float NaN ('nan' would arrive as a string).
@pytest.mark.parametrize('bad', ['-1.0', '.nan'])
def test_rejects_invalid_grace(make_node, bad):
    with pytest.raises(ValueError):
        make_node(grace=bad)


def test_integer_value_is_refused_not_ignored(make_node):
    # safety_launch.py converts with ParameterValue(value_type=float); a
    # direct "-p startup_grace_sec:=5" (integer) must fail loudly rather
    # than silently fall back to the default.
    with pytest.raises(InvalidParameterTypeException):
        make_node(grace='5')
    node, _, _ = make_node(grace='5.0')
    assert math.isclose(node._startup_grace_s, 5.0)

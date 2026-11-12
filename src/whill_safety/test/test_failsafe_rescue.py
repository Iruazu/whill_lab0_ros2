# SPDX-License-Identifier: BSD-3-Clause
"""Issue #161 / ADR-0013 (proposed): teleop rescue while Layer D holds a stop.

The default (layer_d_rescue_enabled: false) must keep today's behaviour:
any active layer publishes zero on /cmd_vel_safety, so teleop never gets
through. With the option on, only a fresh, non-forward teleop command
while Layer D is the ONLY active layer suppresses the zero.

Time is injected by replacing FailsafeNode._now_s; no ROS traffic.
"""
import math
from pathlib import Path

import pytest
import rclpy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from whill_safety.failsafe_node import (
    FORWARD_CLEAR_HYSTERESIS_S, RESCUE_CMD_TIMEOUT_S, RESCUE_LAYER,
    RESCUE_MAX_REVERSE_MPS, RESCUE_MAX_WZ, TWIST_MUX_TELEOP_TIMEOUT_S,
    FailsafeNode, rescue_command_ok, rescue_passes)

T0 = 1000.0
TWIST_MUX_YAML = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'


def _twist(vx=0.0, wz=0.0, **extra):
    t = Twist()
    t.linear.x = float(vx)
    t.angular.z = float(wz)
    for name, v in extra.items():
        part, axis = name.split('_')
        setattr(getattr(t, part), axis, float(v))
    return t


def _blocked_scan():
    # One return straight ahead at 1.5 m: inside the Layer D sector.
    s = LaserScan()
    s.angle_min = 0.0
    s.angle_increment = 0.01
    s.ranges = [1.5]
    return s


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
    made = []

    def _make(**params):
        clock = _Clock(T0)
        monkeypatch.setattr(FailsafeNode, '_now_s', lambda self: clock.t)
        args = ['--ros-args']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
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


def _tick(node, clock, t):
    clock.t = t
    node._tick()


def _block_d(node, clock, t):
    clock.t = t
    node._on_scan(_blocked_scan())


# --- pure helpers --------------------------------------------------------

@pytest.mark.parametrize('cmd,ok', [
    (_twist(-0.2, 0.0), True),        # straight back, UI speed
    (_twist(0.0, 0.4), True),         # rotate in place, UI speed
    (_twist(-0.1, -0.6), True),       # back while turning, at the limits
    (_twist(0.0, 0.0), True),         # stop
    (_twist(0.01, 0.0), False),       # any forward
    (_twist(-0.21, 0.0), False),      # faster reverse than allowed
    (_twist(0.0, 0.61), False),       # faster turn than allowed
    (_twist(-0.1, 0.0, linear_y=0.1), False),
    (_twist(-0.1, 0.0, angular_x=0.1), False),
    (_twist(math.nan, 0.0), False),
    (_twist(-0.1, math.inf), False),
])
def test_rescue_command_ok(cmd, ok):
    assert rescue_command_ok(cmd, RESCUE_MAX_REVERSE_MPS, RESCUE_MAX_WZ) is ok


def test_rescue_passes_only_for_layer_d_alone():
    fresh = dict(cmd_ok=True, cmd_age_s=0.1, timeout_s=0.3, locked=False)
    assert rescue_passes((RESCUE_LAYER,), **fresh)
    assert not rescue_passes((), **fresh)
    assert not rescue_passes(('A:reinit', RESCUE_LAYER), **fresh)
    assert not rescue_passes(('B:pcl_pose_silent', RESCUE_LAYER), **fresh)
    assert not rescue_passes(('C:perception_silent',), **fresh)


def test_rescue_passes_needs_fresh_ok_unlocked_command():
    d = (RESCUE_LAYER,)
    assert not rescue_passes(d, cmd_ok=False, cmd_age_s=0.1, timeout_s=0.3,
                             locked=False)
    assert not rescue_passes(d, cmd_ok=True, cmd_age_s=None, timeout_s=0.3,
                             locked=False)
    assert not rescue_passes(d, cmd_ok=True, cmd_age_s=0.31, timeout_s=0.3,
                             locked=False)
    assert not rescue_passes(d, cmd_ok=True, cmd_age_s=0.1, timeout_s=0.3,
                             locked=True)


def test_default_timeout_beats_twist_mux_teleop_timeout():
    # The rescue window must close before twist_mux drops the teleop slot,
    # or Nav2 (priority 10) could get a turn. Read the shipped config so a
    # later edit of twist_mux.yaml cannot silently break the margin.
    topics = yaml.safe_load(TWIST_MUX_YAML.read_text())[
        'twist_mux']['ros__parameters']['topics']
    assert topics['teleop']['timeout'] == TWIST_MUX_TELEOP_TIMEOUT_S
    assert RESCUE_CMD_TIMEOUT_S < TWIST_MUX_TELEOP_TIMEOUT_S
    assert topics['safety']['priority'] > topics['teleop']['priority'] \
        > topics['navigation']['priority']


# --- node: default keeps today's behaviour --------------------------------

def test_default_blocks_teleop_and_does_not_subscribe(make_node):
    node, clock, pub = make_node()
    topics = {s.topic_name for s in node.subscriptions}
    assert '/cmd_vel_teleop' not in topics
    assert '/cmd_vel' not in topics
    _block_d(node, clock, T0 + 1.0)
    _tick(node, clock, T0 + 1.05)
    assert pub.count == 1


# --- node: option on -----------------------------------------------------

def test_reverse_and_rotate_pass_forward_does_not(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _tick(node, clock, T0 + 1.0)
    assert pub.count == 1                   # stopped, no teleop yet

    node._on_teleop(_twist(-0.2, 0.0))
    _tick(node, clock, T0 + 1.05)
    assert pub.count == 1                   # reverse passes

    node._on_teleop(_twist(0.0, 0.4))
    _tick(node, clock, T0 + 1.1)
    assert pub.count == 1                   # rotation passes

    # Forward: zero goes out from the callback itself, without the tick.
    node._on_teleop(_twist(0.2, 0.0))
    assert pub.count == 2
    _tick(node, clock, T0 + 1.15)
    assert pub.count == 3


def test_stale_teleop_resumes_the_stop(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    clock.t = T0 + 1.0
    node._on_teleop(_twist(-0.2, 0.0))
    # Keep Layer D blocked; only the teleop command ages.
    _block_d(node, clock, T0 + 1.0 + RESCUE_CMD_TIMEOUT_S)
    _tick(node, clock, T0 + 1.0 + RESCUE_CMD_TIMEOUT_S)
    assert pub.count == 0
    _block_d(node, clock, T0 + 1.0 + RESCUE_CMD_TIMEOUT_S + 0.05)
    _tick(node, clock, T0 + 1.0 + RESCUE_CMD_TIMEOUT_S + 0.05)
    assert pub.count == 1


def test_other_layers_still_block_teleop(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    clock.t = T0 + 1.0
    node._last_reinit_time = T0 + 1.0       # Layer A at the same time
    node._on_teleop(_twist(-0.2, 0.0))
    _tick(node, clock, T0 + 1.05)
    assert pub.count == 1


def test_forward_on_mux_output_locks_rescue_until_d_clears(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    clock.t = T0 + 1.0
    node._on_teleop(_twist(-0.2, 0.0))
    _tick(node, clock, T0 + 1.0)
    assert pub.count == 0                   # rescue passing

    # Something other than the rescue command drives forward through the
    # mux (e.g. Nav2 while twist_mux missed the teleop stream): stop now.
    node._on_cmd_vel(_twist(0.3, 0.0))
    assert pub.count == 1
    node._on_teleop(_twist(-0.2, 0.0))
    _tick(node, clock, T0 + 1.05)
    assert pub.count == 2                   # locked: reverse no longer passes

    # Layer D clears (no blocked scan for the hysteresis): lock resets.
    _tick(node, clock, T0 + 1.0 + FORWARD_CLEAR_HYSTERESIS_S + 0.01)
    assert node._rescue_locked is False
    _block_d(node, clock, T0 + 3.0)
    node._on_teleop(_twist(-0.2, 0.0))
    before = pub.count
    _tick(node, clock, T0 + 3.0)
    assert pub.count == before              # passes again


def test_mux_forward_outside_rescue_does_not_lock(make_node):
    node, clock, _ = make_node(layer_d_rescue_enabled=True)
    clock.t = T0 + 1.0
    node._on_cmd_vel(_twist(0.3, 0.0))      # normal Nav2 driving, D clear
    _tick(node, clock, T0 + 1.0)
    assert node._rescue_locked is False


@pytest.mark.parametrize('name,bad', [
    ('rescue_cmd_timeout_s', '0.5'),        # not shorter than twist_mux's
    ('rescue_cmd_timeout_s', '0.0'),
    ('rescue_max_reverse_mps', '-0.1'),
    ('rescue_max_wz', '.nan'),
])
def test_rejects_invalid_rescue_params(make_node, name, bad):
    with pytest.raises(ValueError):
        make_node(layer_d_rescue_enabled=True, **{name: bad})

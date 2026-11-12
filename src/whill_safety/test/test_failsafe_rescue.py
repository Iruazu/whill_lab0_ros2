# SPDX-License-Identifier: BSD-3-Clause
"""Issue #161 / ADR-0013 (proposed): teleop rescue while Layer D holds a stop.

Relay design (option E). The default (layer_d_rescue_enabled: false) keeps
today's behaviour: zero on /cmd_vel_safety while any layer is active, and
no extra subscription. With the option on, while Layer D is the ONLY active
layer, failsafe publishes the validated reverse/rotate teleop command itself
on /cmd_vel_safety, and a zero as soon as any condition fails.

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
    FORWARD_CLEAR_HYSTERESIS_S, PUBLISH_HZ, RESCUE_CMD_TIMEOUT_S,
    RESCUE_LAYER, RESCUE_MAX_REVERSE_MPS, RESCUE_MAX_WZ,
    TWIST_MUX_TELEOP_TIMEOUT_S, FailsafeNode, check_rescue_timing,
    rescue_command_ok, rescue_relay_allowed)

T0 = 1000.0
TICK = 1.0 / PUBLISH_HZ
TWIST_MUX_YAML = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'


def _twist(vx=0.0, wz=0.0, **extra):
    t = Twist()
    t.linear.x = float(vx)
    t.angular.z = float(wz)
    for name, v in extra.items():
        part, axis = name.split('_')
        setattr(getattr(t, part), axis, float(v))
    return t


def _xz(t):
    return (t.linear.x, t.angular.z)


ZERO = (0.0, 0.0)


def _blocked_scan():
    # One return straight ahead at 1.5 m: inside the Layer D sector.
    s = LaserScan()
    s.angle_min = 0.0
    s.angle_increment = 0.01
    s.ranges = [1.5]
    return s


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(_xz(msg))

    def take(self):
        out, self.sent = self.sent, []
        return out


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


def _at(clock, t):
    clock.t = t


def _tick(node, clock, t):
    clock.t = t
    node._tick()


def _block_d(node, clock, t):
    clock.t = t
    node._on_scan(_blocked_scan())


def _teleop(node, clock, t, vx, wz=0.0):
    clock.t = t
    node._on_teleop(_twist(vx, wz))


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


def test_relay_only_for_layer_d_alone():
    fresh = dict(cmd_ok=True, cmd_age_s=0.1, timeout_s=0.3)
    assert rescue_relay_allowed((RESCUE_LAYER,), **fresh)
    assert not rescue_relay_allowed((), **fresh)
    for other in ('A:reinit', 'B:fitness', 'B:pcl_pose_silent',
                  'C:perception_silent',
                  # #159 layers must also keep the zero once merged.
                  'B:pcl_pose_never_arrived', 'C:perception_never_arrived'):
        assert not rescue_relay_allowed((RESCUE_LAYER, other), **fresh)
        assert not rescue_relay_allowed((other,), **fresh)


def test_relay_needs_fresh_rescue_shaped_command():
    d = (RESCUE_LAYER,)
    assert not rescue_relay_allowed(d, cmd_ok=False, cmd_age_s=0.1,
                                    timeout_s=0.3)
    assert not rescue_relay_allowed(d, cmd_ok=True, cmd_age_s=None,
                                    timeout_s=0.3)
    assert not rescue_relay_allowed(d, cmd_ok=True, cmd_age_s=0.31,
                                    timeout_s=0.3)


def test_timing_includes_the_tick():
    check_rescue_timing(RESCUE_CMD_TIMEOUT_S, PUBLISH_HZ)
    # 0.35 + 0.05 tick + 0.1 margin = 0.5: not strictly below the mux's.
    with pytest.raises(ValueError):
        check_rescue_timing(0.35, 20.0)
    # A slower tick eats the margin even with the default timeout.
    with pytest.raises(ValueError):
        check_rescue_timing(RESCUE_CMD_TIMEOUT_S, 10.0)
    with pytest.raises(ValueError):
        check_rescue_timing(0.0, 20.0)


def test_shipped_twist_mux_matches_the_assumptions():
    # Read the shipped config so a later edit of twist_mux.yaml cannot
    # silently break the timing or the priority order the relay relies on.
    topics = yaml.safe_load(TWIST_MUX_YAML.read_text())[
        'twist_mux']['ros__parameters']['topics']
    assert topics['teleop']['timeout'] == TWIST_MUX_TELEOP_TIMEOUT_S
    assert topics['safety']['priority'] > topics['teleop']['priority'] \
        > topics['navigation']['priority']
    # failsafe publishes every tick while active; the safety slot must not
    # expire between two ticks.
    assert topics['safety']['timeout'] > TICK


# --- node: default keeps today's behaviour --------------------------------

def test_default_zero_and_no_subscription(make_node):
    node, clock, pub = make_node()
    assert '/cmd_vel_teleop' not in {s.topic_name for s in node.subscriptions}
    _block_d(node, clock, T0 + 1.0)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == [ZERO]
    _tick(node, clock, T0 + 1.0 + FORWARD_CLEAR_HYSTERESIS_S + 0.01)
    assert pub.take() == []                 # released: silent


# --- node: option on -----------------------------------------------------

def test_relays_reverse_and_rotation_on_the_safety_topic(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == [ZERO]             # stopped, no teleop yet

    _teleop(node, clock, T0 + 1.02, -0.2)
    _tick(node, clock, T0 + 1.05)
    assert pub.take() == [(-0.2, 0.0)]      # relay content = teleop

    _teleop(node, clock, T0 + 1.07, 0.0, 0.4)
    _tick(node, clock, T0 + 1.1)
    assert pub.take() == [(0.0, 0.4)]


def test_forward_teleop_gets_zero_at_once(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _teleop(node, clock, T0 + 1.0, -0.2)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == [(-0.2, 0.0)]
    # Zero from the callback itself, then every tick.
    _teleop(node, clock, T0 + 1.02, 0.2)
    assert pub.take() == [ZERO]
    _tick(node, clock, T0 + 1.05)
    assert pub.take() == [ZERO]


def test_stale_teleop_returns_to_zero(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _teleop(node, clock, T0 + 1.0, -0.2)
    t = T0 + 1.0 + RESCUE_CMD_TIMEOUT_S
    _block_d(node, clock, t)
    _tick(node, clock, t)
    assert pub.take() == [(-0.2, 0.0)]      # age == timeout: still fresh
    t += TICK
    _block_d(node, clock, t)
    _tick(node, clock, t)
    assert pub.take() == [ZERO]


def test_other_layer_switches_relay_to_zero(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _teleop(node, clock, T0 + 1.0, -0.2)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == [(-0.2, 0.0)]
    node._last_reinit_time = T0 + 1.02      # Layer A trips
    _teleop(node, clock, T0 + 1.03, -0.2)
    _tick(node, clock, T0 + 1.05)
    assert pub.take() == [ZERO]


def test_trailing_zero_after_layer_d_clears(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _teleop(node, clock, T0 + 1.0, -0.2)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == [(-0.2, 0.0)]

    # Layer D clears while the operator keeps backing: keep relaying.
    t = T0 + 1.0 + FORWARD_CLEAR_HYSTERESIS_S + 0.01
    _teleop(node, clock, t, -0.2)
    _tick(node, clock, t)
    assert node._active_layers() == ()
    assert pub.take() == [(-0.2, 0.0)]

    # Operator releases (no more teleop): one zero, then silence.
    t += RESCUE_CMD_TIMEOUT_S + TICK
    _tick(node, clock, t)
    assert pub.take() == [ZERO]
    _tick(node, clock, t + TICK)
    assert pub.take() == []


def test_trailing_zero_when_forward_after_layer_d_clears(make_node):
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _block_d(node, clock, T0 + 1.0)
    _teleop(node, clock, T0 + 1.0, -0.2)
    _tick(node, clock, T0 + 1.0)
    t = T0 + 1.0 + FORWARD_CLEAR_HYSTERESIS_S + 0.01
    _teleop(node, clock, t, -0.2)
    _tick(node, clock, t)
    pub.take()
    # D is clear, so forward is ordinary teleop again: one zero, then quiet.
    _teleop(node, clock, t + 0.02, 0.2)
    assert pub.take() == [ZERO]
    _tick(node, clock, t + TICK)
    assert pub.take() == []


def test_no_relay_without_layer_d(make_node):
    # Rescue only exists while stopped; normal teleop never goes through
    # the safety topic.
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    _teleop(node, clock, T0 + 1.0, -0.2)
    _tick(node, clock, T0 + 1.0)
    assert pub.take() == []


@pytest.mark.parametrize('name,bad', [
    ('rescue_cmd_timeout_s', '0.35'),       # + tick + margin reaches 0.5
    ('rescue_cmd_timeout_s', '0.0'),
    ('rescue_max_reverse_mps', '-0.1'),
    ('rescue_max_wz', '.nan'),
])
def test_rejects_invalid_rescue_params(make_node, name, bad):
    with pytest.raises(ValueError):
        make_node(layer_d_rescue_enabled=True, **{name: bad})

# SPDX-License-Identifier: BSD-3-Clause
"""Issue #193: /scan silence after its first message (D:scan_silent).

Before #193, once /scan had arrived, a later outage (LiDAR unplugged,
velodyne_driver or pointcloud_to_laserscan dead) left Layer D at its last
decision and the chair kept moving. scan_silent_sec (default 0.5) makes the
silence an active layer of its own, released after scans have been back for
SCAN_SILENT_RELEASE_S.

What the tests pin, besides stop / release:
- before the first /scan the monitor is inert, so the startup sequence (Nav2
  not up yet) cannot trip it, and #159's never-arrived stop is unchanged;
- the rescue relay (#161) is refused while D:scan_silent is active;
- the handoff from D:forward_blocked to D:scan_silent leaves no tick without
  a stop.

Time is injected by replacing FailsafeNode._now_s / _steady_s; no ROS traffic.
"""
import pytest
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from whill_safety.failsafe_node import (
    FORWARD_CLEAR_HYSTERESIS_S, PUBLISH_HZ, SCAN_SILENT_RELEASE_S,
    SCAN_SILENT_S_DEFAULT, FailsafeNode, silence_step)

T0 = 1000.0
TICK = 1.0 / PUBLISH_HZ
SCAN_DT = 0.1      # /scan at 10 Hz
SILENT = 'D:scan_silent'


def _scan(blocked=False):
    s = LaserScan()
    s.angle_min = 0.0
    s.angle_increment = 0.01
    # 1.5 m straight ahead is inside the Layer D sector; 5 m is outside.
    s.ranges = [1.5 if blocked else 5.0]
    return s


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))

    def take(self):
        out, self.sent = self.sent, []
        return out


class _Clock:
    def __init__(self, t):
        self.t = t


@pytest.fixture
def make_node(monkeypatch):
    made = []

    def _make(t0=T0, **params):
        clock = _Clock(t0)
        monkeypatch.setattr(FailsafeNode, '_now_s', lambda self: clock.t)
        monkeypatch.setattr(FailsafeNode, '_steady_s', lambda self: clock.t)
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


def _run(node, clock, t_from, t_to, scans=True, blocked=False, teleop=None):
    """Advance in 1/20 s ticks; /scan every 0.1 s when scans is True.

    Returns the active layers seen at each tick.
    """
    seen = []
    n = int(round((t_to - t_from) / TICK))
    for i in range(1, n + 1):
        clock.t = t_from + i * TICK
        if scans and i % 2 == 0:
            node._on_scan(_scan(blocked))
        if teleop is not None:
            node._on_teleop(teleop)
        node._tick()
        seen.append(node._active_layers())
    return seen


# --- pure helper ---------------------------------------------------------

def test_helper_inert_before_first_message_and_when_disabled():
    assert silence_step(1e9, None, 0.5, 0.5, False, None) == (False, None)
    assert silence_step(1e9, 0.0, 0.0, 0.5, False, None) == (False, None)


def test_helper_fires_at_the_threshold():
    assert silence_step(10.49, 10.0, 0.5, 0.5, False, None) == (False, None)
    assert silence_step(10.5, 10.0, 0.5, 0.5, False, None) == (True, None)


def test_helper_release_needs_the_hysteresis():
    latched, ok = silence_step(20.0, 19.95, 0.5, 0.5, True, None)
    assert (latched, ok) == (True, 20.0)
    assert silence_step(20.4, 20.35, 0.5, 0.5, True, ok) == (True, 20.0)
    assert silence_step(20.5, 20.45, 0.5, 0.5, True, ok) == (False, None)


def test_helper_clock_step_back_does_not_hold_forever():
    # ok_since in the future (bag --clock loop): restart the hysteresis now.
    assert silence_step(5.0, 4.95, 0.5, 0.5, True, 100.0) == (True, 5.0)


def test_default_is_on_and_matches_layer_d_hysteresis():
    assert SCAN_SILENT_S_DEFAULT == 0.5
    assert SCAN_SILENT_S_DEFAULT == FORWARD_CLEAR_HYSTERESIS_S
    assert SCAN_SILENT_RELEASE_S == FORWARD_CLEAR_HYSTERESIS_S


# --- node ----------------------------------------------------------------

def test_steady_scans_never_trip_it(make_node):
    node, clock, pub = make_node()
    seen = _run(node, clock, T0, T0 + 30.0)
    assert all(SILENT not in a for a in seen)
    assert pub.take() == []


def test_silence_stops_and_resume_releases(make_node):
    node, clock, pub = make_node()
    _run(node, clock, T0, T0 + 2.0)
    pub.take()
    last_scan = clock.t
    seen = _run(node, clock, clock.t, clock.t + 2.0, scans=False)
    # Not yet before 0.5 s, active from 0.5 s on, and zero on every tick.
    fire = [i for i, a in enumerate(seen) if SILENT in a][0]
    assert (fire + 1) * TICK == pytest.approx(SCAN_SILENT_S_DEFAULT)
    assert all(SILENT in a for a in seen[fire:])
    assert pub.take() == [(0.0, 0.0)] * (len(seen) - fire)
    assert last_scan + SCAN_SILENT_S_DEFAULT <= T0 + 4.0

    # Scans back: still stopped for the release hysteresis, then silent.
    seen = _run(node, clock, clock.t, clock.t + 2.0)
    held = [a for a in seen if SILENT in a]
    # The hysteresis counts from the first tick that sees a fresh scan; the
    # first resumed scan lands up to one scan period after the resume.
    assert SCAN_SILENT_RELEASE_S <= len(held) * TICK + 1e-9 \
        <= SCAN_SILENT_RELEASE_S + SCAN_DT + 1e-9
    assert all(SILENT not in a for a in seen[len(held):])
    assert seen[-1] == ()


def test_intermittent_scans_stay_stopped(make_node):
    # One scan every 0.6 s: each gap is a silence, so the release
    # hysteresis never completes and the stop does not flap.
    node, clock, pub = make_node()
    _run(node, clock, T0, T0 + 1.0)
    _run(node, clock, clock.t, clock.t + 0.6, scans=False)
    seen = []
    for _ in range(10):
        node._on_scan(_scan())
        seen += _run(node, clock, clock.t, clock.t + 0.6, scans=False)
    assert all(SILENT in a for a in seen)


def test_forward_blocked_hands_over_without_a_gap(make_node):
    # The last scan saw an obstacle, then /scan died: D:forward_blocked
    # expires exactly when D:scan_silent starts; no tick may be unstopped.
    node, clock, pub = make_node()
    _run(node, clock, T0, T0 + 1.0, blocked=True)
    seen = _run(node, clock, clock.t, clock.t + 2.0, scans=False)
    assert all(a for a in seen)
    assert seen[0] == ('D:forward_blocked',)
    assert seen[-1] == (SILENT,)


def test_startup_before_first_scan_is_unaffected(make_node):
    # Nav2 (the /scan source) starts after m6r_bringup: with the default
    # (startup grace off) nothing stops before the first /scan.
    node, clock, pub = make_node()
    seen = _run(node, clock, T0, T0 + 60.0, scans=False)
    assert all(a == () for a in seen)
    assert pub.take() == []


def test_never_arrived_then_silence_are_separate(make_node):
    # #159: before the first /scan it is D:scan_never_arrived (only with a
    # grace). After it, D:scan_silent. Never both at once.
    node, clock, pub = make_node(startup_grace_sec=10.0)
    seen = _run(node, clock, T0, T0 + 12.0, scans=False)
    assert 'D:scan_never_arrived' in seen[-1]
    assert all(SILENT not in a for a in seen)
    seen = _run(node, clock, clock.t, clock.t + 1.0)
    assert all('D:scan_never_arrived' not in a and SILENT not in a
               for a in seen[2:])
    seen = _run(node, clock, clock.t, clock.t + 1.0, scans=False)
    assert SILENT in seen[-1]
    assert all('D:scan_never_arrived' not in a for a in seen)


def test_scan_before_sim_clock_does_not_trip_it(make_node):
    # use_sim_time: a /scan received before the first /clock is stamped 0.
    # Counting it would trip the stop the moment the clock jumps to bag time.
    node, clock, pub = make_node(t0=0.0)
    node._on_scan(_scan())
    clock.t = 1.7e9
    node._tick()
    assert SILENT not in node._active_layers()
    seen = _run(node, clock, clock.t, clock.t + 5.0)
    assert all(SILENT not in a for a in seen)


def test_disabled_keeps_the_pre_193_behaviour(make_node):
    node, clock, pub = make_node(scan_silent_sec=0.0)
    _run(node, clock, T0, T0 + 1.0)
    seen = _run(node, clock, clock.t, clock.t + 30.0, scans=False)
    assert all(a == () for a in seen)
    assert pub.take() == []


def test_rescue_refused_while_scan_silent(make_node):
    # #161: the relay is for Layer D alone. With /scan dead, Layer D cannot
    # see what a reverse or a turn would sweep into, so the zero holds.
    node, clock, pub = make_node(layer_d_rescue_enabled=True)
    back = Twist()
    back.linear.x = -0.2
    _run(node, clock, T0, T0 + 1.0, blocked=True)
    seen = _run(node, clock, clock.t, clock.t + 0.4, blocked=True,
                teleop=back)
    assert seen[-1] == ('D:forward_blocked',)
    assert pub.take()[-1] == (-0.2, 0.0)          # relaying
    seen = _run(node, clock, clock.t, clock.t + 2.0, scans=False,
                teleop=back)
    assert seen[-1] == (SILENT,)
    sent = pub.take()
    assert sent[-1] == (0.0, 0.0)
    # From the first tick D:scan_silent is active, only zeros go out.
    fire = [i for i, a in enumerate(seen) if SILENT in a][0]
    assert set(sent[fire:]) == {(0.0, 0.0)}


@pytest.mark.parametrize('bad', ['-0.1', 'nan'])
def test_rejects_invalid_scan_silent(make_node, bad):
    with pytest.raises(ValueError):
        make_node(scan_silent_sec=bad)


def test_integer_value_is_refused_not_ignored(make_node):
    with pytest.raises(InvalidParameterTypeException):
        make_node(scan_silent_sec=1)

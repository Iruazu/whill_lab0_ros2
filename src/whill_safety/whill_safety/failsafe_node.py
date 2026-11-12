#!/usr/bin/env python3
"""M6R-3 lite failsafe (2026-08-01 demo scope).

Publishes zero /cmd_vel_safety while any of the layers below trips.
twist_mux (priority 100 on the safety topic) then wins over Nav2 output
on the shared /cmd_vel bus and the chair coasts to a stop.

Layers (per ADR-0007 §Demo-scope reduction):

  A  /reinitialization_requested received within LAYER_A_HOLD_S
     (operator says "stopping to reset localizer")
  B  either
       - /alignment_status fitness_score > FITNESS_MAX for FITNESS_WINDOW_S
       - /alignment_status has_converged == false for FITNESS_WINDOW_S
       - /pcl_pose silent for PCL_POSE_TIMEOUT_S
  C  /velodyne_points_no_ground silent for PERCEPTION_TIMEOUT_S
     (patchworkpp_node crash / hang. M6R4-3 F3 mitigation: with
     use_collision_detection: true, a silent perception pipe would
     leave stale obstacle_layer cells while the chair moves. Layer C
     forces a stop within ~2 s of the pipe going silent.)
  D  /scan has >= FORWARD_POINT_COUNT_MIN points inside a forward
     base_link sector (±FORWARD_SECTOR_HALF_ANGLE_RAD, distance
     FORWARD_SECTOR_MIN_M..FORWARD_SECTOR_MAX_M). Continuous clear for
     FORWARD_CLEAR_HYSTERESIS_S releases. M6R4-3 V2 blocker mitigation
     (2026-07-16 field): RPP's own collision_check reaches only ~0.3 m
     along the carrot, and once the planner replans around the person
     the "obstacle on path" condition drops entirely. Layer D provides
     the stop-for-obstacle behaviour independently of Nav2 planning.
     See ADR-0007 §Layer D for the geometry rationale.

Optional (parameter layer_d_rescue_enabled, default false, Issue #161,
ADR-0013 proposed): while Layer D is the ONLY active layer, a fresh
/cmd_vel_teleop command with no forward component (reverse / rotate within
limits) is RELAYED by this node on /cmd_vel_safety instead of the zero.
The safety slot (priority 100) never goes silent, so neither Nav2 nor a
forward teleop command can reach /cmd_vel while the stop holds.

Deliberately omitted (post-demo backlog): jump detection (3-frame /pcl_pose
delta), SAFE_HOLD release hysteresis, G4 hardware 3-test acceptance. The
demo is operated with a walker beside the chair whose WHILL joystick
bypasses this path, so we accept the small risk of brief flapping on
condition-clear in exchange for keeping the code minimal for 2026-08-01.
"""

import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud2


FITNESS_MAX = 1.0
FITNESS_WINDOW_S = 2.0
PCL_POSE_TIMEOUT_S = 1.0
# /velodyne_points_no_ground publishes at ~10 Hz (matches VLP-16 rate).
# 2 s silent = 20 missed frames; that is well past any realistic
# scheduling jitter and definitively means patchworkpp_node is down.
PERCEPTION_TIMEOUT_S = 2.0
# Layer D — forward sector perception gate. Rationale in ADR-0007
# §Layer D. Half-angle 30° covers the chair's 0.6 m width out to 1.15 m
# on either side; distance 1.0-2.0 m matches the effective downstream
# perception coverage — Patchwork++ filters points closer than 1.0 m as
# WHILL body self-return (whill_perception/config/patchworkpp.yaml
# `min_range: 1.0`), so /scan is silent below 1.0 m by design. Layer D
# min was 0.5 in the draft but that was aspirational; the honest number
# is 1.0 until Patchwork++ is re-evaluated (post-demo, see ADR-0007
# §Layer D "Patchwork++ min_range alignment"). 2.0 m gives ~6.7 s
# reaction budget at desired_linear_vel = 0.3 m/s (velocity_smoother
# stop distance from max_decel = 0.5 m/s² is only 0.09 m). Point
# threshold 5 is 1/6 of a person's ~30-beam silhouette at 2 m, so
# single ghost returns do not fire. Hysteresis 0.5 s = 5 consecutive
# clear scans at 10 Hz — same re-latch pattern as LAYER_A_HOLD_S.
FORWARD_SECTOR_HALF_ANGLE_RAD = math.radians(30.0)
FORWARD_SECTOR_MIN_M = 1.0
FORWARD_SECTOR_MAX_M = 2.0
# 1, not 5: 2026-07-19 field の実測 (band_probe, 149 scan) で、帯内に人が
# 立っていても点数は 0-2 点が支配的 (>=3 は 6/149)。ADR-0009 の「脚 ~4 点」
# は現行の /scan (129 bin と疎) では成立せず、3 以上のいかなる閾値でも
# 単独歩行者の検出がフリッカーする。1 点で即遮断し、解除は 0.5 s
# ヒステリシス (5 連続クリア scan) に委ねる。ghost 1 点での不要停止は
# 増え得るが、demo は並走運用で「止まりすぎ」は安全側。/scan が疎な根本
# 原因 (p2ls 角度分解能 / ADR-0009 再検証) は Issue #102 で追う。
FORWARD_POINT_COUNT_MIN = 1
FORWARD_CLEAR_HYSTERESIS_S = 0.5
# Dead-input watchdog: how long we tolerate a first-message-arm layer
# staying unarmed before shouting ERROR. 2026-07-16 incident: Layer D's
# /scan subscription defaulted to RELIABLE while p2ls publishes
# BEST_EFFORT — the QoS mismatch produced a silent subscribe (0
# messages) and the layer never armed. The chair drove into a person
# during V2 verification because failsafe stayed dormant. The watchdog
# turns that failure mode from silent to loud.
STARTUP_DEAD_INPUT_TIMEOUT_S = 10.0
LAYER_A_HOLD_S = 1.0
PUBLISH_HZ = 20.0
# Issue #161 / ADR-0013 (proposed) — teleop rescue while Layer D stops.
# Default off: changing the stop behaviour needs the user's approval.
RESCUE_LAYER = 'D:forward_blocked'
# twist_mux's teleop timeout (config/twist_mux.yaml; test_failsafe_rescue.py
# pins the two together). A relayed command may outlive the operator's last
# teleop message by up to rescue_cmd_timeout_s + one tick; that must stay
# shorter than what twist_mux itself would honour for teleop.
TWIST_MUX_TELEOP_TIMEOUT_S = 0.5
# Head-room on top of timeout + tick for DDS / executor jitter.
RESCUE_TIMING_MARGIN_S = 0.1
# 0.3 s: the Web UI streams at 10 Hz, so two lost commands are tolerated.
RESCUE_CMD_TIMEOUT_S = 0.3
# Ceilings for a command to count as rescue. Commands outside are refused
# (zero continues), not clamped. 0.2 m/s = the Web UI reverse speed
# (velocity_smoother limits reverse further to 0.1); 0.6 rad/s =
# dispatch_node's TELEOP_WZ_MAX.
RESCUE_MAX_REVERSE_MPS = 0.2
RESCUE_MAX_WZ = 0.6


def _to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _to_bool(s):
    if s is None:
        return None
    lo = str(s).strip().lower()
    if lo in ('true', '1', 'ok'):
        return True
    if lo in ('false', '0'):
        return False
    return None


def rescue_command_ok(cmd, max_reverse_mps, max_wz):
    """True when a teleop Twist cannot move the chair forward.

    Reverse (-max_reverse_mps <= x <= 0) and rotation (|z| <= max_wz) only;
    every other component must be exactly 0 and every value finite, so a
    malformed or unexpected command fails closed.
    """
    lin, ang = cmd.linear, cmd.angular
    vals = (lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
    if not all(math.isfinite(v) for v in vals):
        return False
    return (-max_reverse_mps <= lin.x <= 0.0
            and abs(ang.z) <= max_wz
            and lin.y == 0.0 and lin.z == 0.0
            and ang.x == 0.0 and ang.y == 0.0)


def rescue_cmd_valid(cmd_ok, cmd_age_s, timeout_s):
    """The latest teleop command is fresh and rescue-shaped."""
    return cmd_ok and cmd_age_s is not None and cmd_age_s <= timeout_s


def rescue_relay_allowed(active, cmd_ok, cmd_age_s, timeout_s):
    """True when failsafe should relay the teleop command instead of zero.

    Only Layer D on its own: A (reinit), B (localization), C (perception)
    and any later-added layer (e.g. #159's *_never_arrived) keep the zero.
    """
    return (tuple(active) == (RESCUE_LAYER,)
            and rescue_cmd_valid(cmd_ok, cmd_age_s, timeout_s))


def check_rescue_timing(timeout_s, publish_hz):
    """Raise unless the relay cannot outlive twist_mux's teleop timeout."""
    if not timeout_s > 0.0 or not publish_hz > 0.0 or not (
            timeout_s + 1.0 / publish_hz + RESCUE_TIMING_MARGIN_S
            < TWIST_MUX_TELEOP_TIMEOUT_S):
        raise ValueError(
            f'rescue_cmd_timeout_s={timeout_s!r} + 1/publish_hz '
            f'({publish_hz!r} Hz) + {RESCUE_TIMING_MARGIN_S}s margin must '
            f'be < the twist_mux teleop timeout {TWIST_MUX_TELEOP_TIMEOUT_S}s')


class FailsafeNode(Node):

    def __init__(self):
        super().__init__('failsafe_node')

        self._last_reinit_time = None
        self._fitness_bad_since = None
        self._converged_bad_since = None
        self._last_pose_time = None
        self._last_perception_time = None
        self._last_scan_time = None
        self._forward_last_blocked_time = None
        self._active_prev = ()
        self._start_time = self._now_s()
        self._dead_input_warned = False

        self.create_subscription(
            Bool, '/reinitialization_requested', self._on_reinit, 10)
        self.create_subscription(
            DiagnosticArray, '/alignment_status', self._on_alignment, 10)
        # PoseWithCovarianceStamped: lidar_localization_ros2 publishes
        # /pcl_pose as PoseWithCovarianceStamped (covariance is used by
        # the M6R-2 alignment gate). An earlier PoseStamped subscription
        # here silently type-mismatched — B:pcl_pose_silent never had a
        # last_pose_time to time out against, so failsafe layer B could
        # not observe a genuine localizer stall. Verified in Phase B on
        # 2026-07-14 that the fixed type reaches the callback.
        self.create_subscription(
            PoseWithCovarianceStamped, '/pcl_pose', self._on_pcl_pose, 10)
        # Layer C: /velodyne_points_no_ground is Patchwork++'s output
        # (whill_perception, ADR-0011). Sensor-data QoS (BEST_EFFORT +
        # KEEP_LAST 5) matches the publisher; a Reliable subscription
        # would silently fail to receive, which would falsely trip Layer C
        # from launch. Only the message arrival timestamp is used; the
        # payload is discarded — cheap for a 10 Hz PointCloud2.
        self.create_subscription(
            PointCloud2, '/velodyne_points_no_ground',
            self._on_perception, qos_profile_sensor_data)
        # Layer D: /scan is p2ls_node's output (whill_navigation
        # nav_launch.py). p2ls publishes BEST_EFFORT (sensor QoS) —
        # verified 2026-07-16 field after an incident in which this
        # subscription defaulted to RELIABLE and received zero messages,
        # leaving Layer D silently unarmed and the chair driving into a
        # person during V2 verification. qos_profile_sensor_data
        # (BEST_EFFORT + KEEP_LAST 5) is compatible with either
        # publisher policy, so this is the safe choice regardless of
        # what upstream ends up doing.
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self._pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        # Kept as an attribute so the rescue timing check uses the real rate
        # (C3 turns it into a parameter; substitute there).
        self._publish_hz = PUBLISH_HZ
        self._init_rescue()
        self.create_timer(1.0 / self._publish_hz, self._tick)

        # Optional features append one note each; kept as a list so later
        # options (#159 startup_grace_sec) merge as one more append.
        ready_notes = [self._rescue_note()]
        self.get_logger().info(
            f'failsafe_node ready: fitness > {FITNESS_MAX} for '
            f'{FITNESS_WINDOW_S}s | pcl_pose silent > {PCL_POSE_TIMEOUT_S}s | '
            f'perception silent > {PERCEPTION_TIMEOUT_S}s | '
            f'forward_blocked > {FORWARD_POINT_COUNT_MIN} pts in '
            f'±{math.degrees(FORWARD_SECTOR_HALF_ANGLE_RAD):.0f}° @ '
            f'{FORWARD_SECTOR_MIN_M}-{FORWARD_SECTOR_MAX_M} m, '
            f'hysteresis {FORWARD_CLEAR_HYSTERESIS_S}s | '
            f'A hold {LAYER_A_HOLD_S}s | publish {PUBLISH_HZ:.0f} Hz'
            + ''.join(f' | {n}' for n in ready_notes))

    def _init_rescue(self):
        # Issue #161. Declared even when disabled so `ros2 param get` shows
        # the state; the subscription exists only when enabled, so the
        # default node graph is unchanged.
        self._rescue_enabled = bool(self.declare_parameter(
            'layer_d_rescue_enabled', False).value)
        self._rescue_timeout_s = float(self.declare_parameter(
            'rescue_cmd_timeout_s', RESCUE_CMD_TIMEOUT_S).value)
        self._rescue_max_reverse = float(self.declare_parameter(
            'rescue_max_reverse_mps', RESCUE_MAX_REVERSE_MPS).value)
        self._rescue_max_wz = float(self.declare_parameter(
            'rescue_max_wz', RESCUE_MAX_WZ).value)
        # Refuse values that would silently widen the bypass.
        check_rescue_timing(self._rescue_timeout_s, self._publish_hz)
        for name, v in (('rescue_max_reverse_mps', self._rescue_max_reverse),
                        ('rescue_max_wz', self._rescue_max_wz)):
            if not math.isfinite(v) or v < 0.0:
                raise ValueError(f'{name}={v!r} must be a finite value >= 0')
        self._rescue_cmd = Twist()
        self._rescue_cmd_ok = False
        self._rescue_cmd_time = None
        self._relaying = False
        if self._rescue_enabled:
            # Same depth as twist_mux's own subscription.
            self.create_subscription(
                Twist, '/cmd_vel_teleop', self._on_teleop, 10)

    def _rescue_note(self):
        if not self._rescue_enabled:
            return 'Layer D rescue off'
        return (f'Layer D rescue relay ON (reverse <= '
                f'{self._rescue_max_reverse} m/s, |wz| <= '
                f'{self._rescue_max_wz} rad/s, teleop fresh <= '
                f'{self._rescue_timeout_s}s)')

    def _on_teleop(self, msg):
        self._rescue_cmd = msg
        self._rescue_cmd_time = self._now_s()
        self._rescue_cmd_ok = rescue_command_ok(
            msg, self._rescue_max_reverse, self._rescue_max_wz)
        # A non-rescue command (e.g. forward) while relaying: zero at once
        # instead of relaying the previous reverse until the next tick.
        # Forward teleop itself cannot reach /cmd_vel either way, because
        # the safety slot stays live.
        if self._relaying and not self._rescue_cmd_ok:
            self._end_relay('teleop command not reverse/rotate')

    def _rescue_cmd_age(self, now):
        return None if self._rescue_cmd_time is None \
            else now - self._rescue_cmd_time

    def _end_relay(self, why):
        self._relaying = False
        self.get_logger().warn(f'failsafe RESCUE ended — {why}')
        self._pub.publish(Twist())

    def _rescue_output(self, active, now):
        """Twist to publish on /cmd_vel_safety this tick, or None (silent).

        Rescue disabled: zero while any layer is active, else silent, i.e.
        exactly the pre-#161 behaviour.
        """
        if not self._rescue_enabled:
            return Twist() if active else None
        age = self._rescue_cmd_age(now)
        if active:
            if rescue_relay_allowed(active, self._rescue_cmd_ok, age,
                                    self._rescue_timeout_s):
                if not self._relaying:
                    self.get_logger().warn(
                        'failsafe RESCUE relay — Layer D only, teleop '
                        'reverse/rotate relayed on /cmd_vel_safety')
                self._relaying = True
                return self._rescue_cmd
            if self._relaying:
                self._relaying = False
                self.get_logger().warn(
                    f'failsafe RESCUE ended — {list(active)}')
            return Twist()
        if self._relaying:
            # Layer D cleared mid-relay. Keep relaying until the teleop goes
            # stale (or stops being rescue-shaped), then send one zero: going
            # silent at once would let the last relayed motion hold in
            # twist_mux's safety slot for its 0.5 s timeout and drop the
            # operator's release-zero (it arrives on the outranked teleop).
            if rescue_cmd_valid(self._rescue_cmd_ok, age,
                                self._rescue_timeout_s):
                return self._rescue_cmd
            self._end_relay('teleop stale after Layer D cleared')
        return None

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_reinit(self, msg):
        # data=True is the trigger; data=False is treated as "no signal"
        # per Issue #67. A momentary True latches for LAYER_A_HOLD_S.
        if msg.data:
            self._last_reinit_time = self._now_s()

    def _on_alignment(self, msg):
        # Walk every DiagnosticStatus / KeyValue so we tolerate any upstream
        # rearrangement of the fields we do not depend on.
        fitness = None
        converged = None
        for status in msg.status:
            for kv in status.values:
                if kv.key == 'fitness_score':
                    v = _to_float(kv.value)
                    if v is not None:
                        fitness = v
                elif kv.key == 'has_converged':
                    v = _to_bool(kv.value)
                    if v is not None:
                        converged = v

        now = self._now_s()
        if fitness is not None:
            if fitness > FITNESS_MAX:
                if self._fitness_bad_since is None:
                    self._fitness_bad_since = now
            else:
                self._fitness_bad_since = None
        if converged is not None:
            if not converged:
                if self._converged_bad_since is None:
                    self._converged_bad_since = now
            else:
                self._converged_bad_since = None

    def _on_pcl_pose(self, msg):
        self._last_pose_time = self._now_s()

    def _on_perception(self, msg):
        self._last_perception_time = self._now_s()

    def _on_scan(self, msg):
        # Every scan updates _last_scan_time — this is what the dead-input
        # watchdog checks. _forward_last_blocked_time only ticks when the
        # sector is blocked; a silent scan pipe would leave both at None
        # and the watchdog would not know whether Layer D armed via
        # "first sector block" or "first scan arrived". Track both.
        self._last_scan_time = self._now_s()
        # Count /scan returns inside the forward safety sector. Short-
        # circuit as soon as the threshold is met — no reason to keep
        # counting once Layer D is going to fire; the callback runs at
        # the scan rate and the gate stays engaged for HYSTERESIS_S
        # regardless of the exact count.
        count = 0
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) \
                    and FORWARD_SECTOR_MIN_M <= r <= FORWARD_SECTOR_MAX_M \
                    and abs(angle) <= FORWARD_SECTOR_HALF_ANGLE_RAD:
                count += 1
                if count >= FORWARD_POINT_COUNT_MIN:
                    self._forward_last_blocked_time = self._now_s()
                    return
            angle += msg.angle_increment

    def _active_layers(self):
        now = self._now_s()
        out = []
        if self._last_reinit_time is not None \
                and now - self._last_reinit_time < LAYER_A_HOLD_S:
            out.append('A:reinit')
        if self._fitness_bad_since is not None \
                and now - self._fitness_bad_since >= FITNESS_WINDOW_S:
            out.append('B:fitness')
        if self._converged_bad_since is not None \
                and now - self._converged_bad_since >= FITNESS_WINDOW_S:
            out.append('B:converged')
        # pcl_pose watchdog: only after first reception, otherwise startup
        # would trip immediately before the localizer produced anything.
        if self._last_pose_time is not None \
                and now - self._last_pose_time > PCL_POSE_TIMEOUT_S:
            out.append('B:pcl_pose_silent')
        # Layer C perception watchdog: same "arm on first reception"
        # pattern so bringup does not trip during the ~1 s window
        # patchworkpp needs to load its config and receive its first
        # scan.
        if self._last_perception_time is not None \
                and now - self._last_perception_time > PERCEPTION_TIMEOUT_S:
            out.append('C:perception_silent')
        # Layer D forward-sector gate: fires while the sector is blocked
        # and stays engaged for HYSTERESIS_S after the last blocked
        # scan. Same first-message-arm pattern as Layer A/C.
        if self._forward_last_blocked_time is not None \
                and now - self._forward_last_blocked_time < FORWARD_CLEAR_HYSTERESIS_S:
            out.append('D:forward_blocked')
        return tuple(out)

    def _tick(self):
        now = self._now_s()
        # Dead-input watchdog: shout once when the startup budget expires
        # if any first-message-arm layer has not received its input.
        # 2026-07-16 incident: Layer D was subscribing to /scan on the
        # default (RELIABLE) QoS while p2ls publishes BEST_EFFORT — zero
        # messages, zero armed layers, and a contact test that failed
        # closed against a person. The gate must not be silent when it
        # cannot arm.
        if (not self._dead_input_warned
                and now - self._start_time > STARTUP_DEAD_INPUT_TIMEOUT_S):
            missing = []
            if self._last_pose_time is None:
                missing.append('B:/pcl_pose')
            if self._last_perception_time is None:
                missing.append('C:/velodyne_points_no_ground')
            if self._last_scan_time is None:
                missing.append('D:/scan')
            if missing:
                self.get_logger().error(
                    f'DEAD INPUT after '
                    f'{STARTUP_DEAD_INPUT_TIMEOUT_S:.0f}s: {missing} — '
                    f'these subscriptions received ZERO messages. Likely '
                    f'DDS/QoS mismatch, wrong topic name, or upstream '
                    f'not running. The listed failsafe layers CANNOT '
                    f'ARM. Do not drive.')
            self._dead_input_warned = True

        active = self._active_layers()
        if active != self._active_prev:
            if active:
                self.get_logger().warn(
                    f'failsafe ENGAGED — {list(active)}')
            else:
                self.get_logger().info('failsafe RELEASED')
            self._active_prev = active
        out = self._rescue_output(active, now)
        if out is not None:
            self._pub.publish(out)


def _raise_keyboard_interrupt(*_):
    raise KeyboardInterrupt()


def main():
    # Clean shutdown path for `ros2 launch` and manual Ctrl-C.
    #
    # rclpy's default signal handling (SignalHandlerOptions.ALL) shuts down
    # the context inside the C-level signal handler, which races the wait-
    # set `rclpy.spin` is blocked on. In humble that surfaces as a bare
    # `rclpy._rclpy_pybind11.RCLError: the given context is not valid`
    # leaking out of spin — neither `KeyboardInterrupt` nor
    # `ExternalShutdownException` — and the process exits 1 with a
    # traceback.
    #
    # Trading rclpy's handlers for plain Python handlers keeps the exit
    # path Pythonic: SIGINT (Python default) raises KeyboardInterrupt,
    # and we install the same for SIGTERM so `ros2 launch` teardown also
    # exits cleanly. rclpy.try_shutdown() in `finally` is the idempotent
    # variant so double-shutdown does not error either.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    node = FailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

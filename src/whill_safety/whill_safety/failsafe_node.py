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

Parameters (2026-10-28, avoidance-baseline F1-F4): every threshold below
is a ROS parameter whose default equals the module constant, so a launch
that passes no parameters behaves exactly as before. The additions are
opt-in and inert at their defaults:

  layer_d_shape            'sector' (default, the geometry above) or
                           'rectangle': x in [forward_min_m, forward_max_m]
                           and |y| <= layer_d_rect_half_width_m in
                           base_link. Used only with avoidance:=true so
                           obstacles beside the path no longer stop the
                           chair while Collision Monitor handles slowdown.
  extra_perception_topics  extra PointCloud2 inputs watched like Layer C
                           (e.g. the Hi-Res detection cloud and depth
                           points). Empty by default.
  /failsafe/state          std_msgs/String with the active layers ("OK"
                           when clear), on change and at 1 Hz, so bags can
                           record the failsafe state instead of grepping
                           /rosout.

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
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
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
LAYER_D_SHAPES = ('sector', 'rectangle')
# Rectangle half-width for avoidance mode: chair half-width (~0.3 m) plus
# 0.2 m margin. Narrow enough that a planned pass beside an obstacle is not
# vetoed, wide enough to cover the chair's own swept path.
LAYER_D_RECT_HALF_WIDTH_M = 0.5
STATE_HEARTBEAT_S = 1.0


def forward_zone_blocked(ranges, angle_min, angle_increment, shape,
                         min_m, max_m, half_angle_rad, half_width_m,
                         count_min):
    """True when >= count_min /scan returns fall inside the Layer D zone.

    The sector branch keeps the original loop (accumulated angle, polar
    range test) so the default behaviour is bit-for-bit unchanged.
    """
    count = 0
    angle = angle_min
    for r in ranges:
        if math.isfinite(r):
            if shape == 'sector':
                inside = min_m <= r <= max_m and abs(angle) <= half_angle_rad
            else:
                x = r * math.cos(angle)
                inside = min_m <= x <= max_m \
                    and abs(r * math.sin(angle)) <= half_width_m
            if inside:
                count += 1
                if count >= count_min:
                    return True
        angle += angle_increment
    return False


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


class FailsafeNode(Node):

    def __init__(self):
        super().__init__('failsafe_node')

        p = self.declare_parameter
        self._fitness_max = p('fitness_max', FITNESS_MAX).value
        self._fitness_window_s = p('fitness_window_s', FITNESS_WINDOW_S).value
        self._pcl_pose_timeout_s = p('pcl_pose_timeout_s', PCL_POSE_TIMEOUT_S).value
        self._perception_timeout_s = p('perception_timeout_s', PERCEPTION_TIMEOUT_S).value
        # Degrees for humans; the 30.0 literal mirrors the constant above so
        # math.radians() reproduces FORWARD_SECTOR_HALF_ANGLE_RAD exactly.
        self._fwd_half_angle_rad = math.radians(p(
            'forward_sector_half_angle_deg', 30.0).value)
        self._fwd_min_m = p('forward_min_m', FORWARD_SECTOR_MIN_M).value
        self._fwd_max_m = p('forward_max_m', FORWARD_SECTOR_MAX_M).value
        self._fwd_count_min = p('forward_point_count_min', FORWARD_POINT_COUNT_MIN).value
        self._fwd_hysteresis_s = p(
            'forward_clear_hysteresis_s', FORWARD_CLEAR_HYSTERESIS_S).value
        self._dead_input_timeout_s = p(
            'startup_dead_input_timeout_s', STARTUP_DEAD_INPUT_TIMEOUT_S).value
        self._layer_a_hold_s = p('layer_a_hold_s', LAYER_A_HOLD_S).value
        self._publish_hz = p('publish_hz', PUBLISH_HZ).value
        self._layer_d_shape = p('layer_d_shape', 'sector').value
        self._rect_half_width_m = p(
            'layer_d_rect_half_width_m', LAYER_D_RECT_HALF_WIDTH_M).value
        # Declared by type, not by an empty-list default: rclpy types []
        # as BYTE_ARRAY, which would then reject a string-array override.
        extra = p('extra_perception_topics', Parameter.Type.STRING_ARRAY).value
        self._extra_topics = [t for t in (extra or []) if t]
        # A typo here would silently fall through to the rectangle branch
        # of forward_zone_blocked; refuse to start instead.
        if self._layer_d_shape not in LAYER_D_SHAPES:
            raise ValueError(
                f'layer_d_shape={self._layer_d_shape!r} must be one of '
                f'{LAYER_D_SHAPES}')

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
        self._extra_last = {t: None for t in self._extra_topics}
        self._last_state_pub = None

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

        # Extra perception heartbeats (F3): same sensor-data QoS and
        # arm-on-first-message rule as Layer C. The default argument binds
        # the topic per lambda; a bare closure would capture only the last.
        for topic in self._extra_topics:
            self.create_subscription(
                PointCloud2, topic,
                lambda _msg, t=topic: self._extra_last.__setitem__(t, self._now_s()),
                qos_profile_sensor_data)

        self._pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self._state_pub = self.create_publisher(String, '/failsafe/state', 10)
        self.create_timer(1.0 / self._publish_hz, self._tick)

        if self._layer_d_shape == 'sector':
            zone = (f'±{math.degrees(self._fwd_half_angle_rad):.0f}° @ '
                    f'{self._fwd_min_m}-{self._fwd_max_m} m')
        else:
            zone = (f'rectangle x {self._fwd_min_m}-{self._fwd_max_m} m, '
                    f'|y| <= {self._rect_half_width_m} m')
        self.get_logger().info(
            f'failsafe_node ready: fitness > {self._fitness_max} for '
            f'{self._fitness_window_s}s | pcl_pose silent > {self._pcl_pose_timeout_s}s | '
            f'perception silent > {self._perception_timeout_s}s '
            f'(+{len(self._extra_topics)} extra) | '
            f'forward_blocked >= {self._fwd_count_min} pts in {zone}, '
            f'hysteresis {self._fwd_hysteresis_s}s | '
            f'A hold {self._layer_a_hold_s}s | publish {self._publish_hz:.0f} Hz')

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
            if fitness > self._fitness_max:
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
        # Count /scan returns inside the forward safety zone. The helper
        # short-circuits once the threshold is met; the gate then stays
        # engaged for the hysteresis regardless of the exact count.
        if forward_zone_blocked(
                msg.ranges, msg.angle_min, msg.angle_increment,
                self._layer_d_shape, self._fwd_min_m, self._fwd_max_m,
                self._fwd_half_angle_rad, self._rect_half_width_m,
                self._fwd_count_min):
            self._forward_last_blocked_time = self._now_s()

    def _active_layers(self):
        now = self._now_s()
        out = []
        if self._last_reinit_time is not None \
                and now - self._last_reinit_time < self._layer_a_hold_s:
            out.append('A:reinit')
        if self._fitness_bad_since is not None \
                and now - self._fitness_bad_since >= self._fitness_window_s:
            out.append('B:fitness')
        if self._converged_bad_since is not None \
                and now - self._converged_bad_since >= self._fitness_window_s:
            out.append('B:converged')
        # pcl_pose watchdog: only after first reception, otherwise startup
        # would trip immediately before the localizer produced anything.
        if self._last_pose_time is not None \
                and now - self._last_pose_time > self._pcl_pose_timeout_s:
            out.append('B:pcl_pose_silent')
        # Layer C perception watchdog: same "arm on first reception"
        # pattern so bringup does not trip during the ~1 s window
        # patchworkpp needs to load its config and receive its first
        # scan.
        if self._last_perception_time is not None \
                and now - self._last_perception_time > self._perception_timeout_s:
            out.append('C:perception_silent')
        for topic, last in self._extra_last.items():
            if last is not None and now - last > self._perception_timeout_s:
                out.append(f'C:{topic}_silent')
        # Layer D forward-sector gate: fires while the sector is blocked
        # and stays engaged for HYSTERESIS_S after the last blocked
        # scan. Same first-message-arm pattern as Layer A/C.
        if self._forward_last_blocked_time is not None \
                and now - self._forward_last_blocked_time < self._fwd_hysteresis_s:
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
                and now - self._start_time > self._dead_input_timeout_s):
            missing = []
            if self._last_pose_time is None:
                missing.append('B:/pcl_pose')
            if self._last_perception_time is None:
                missing.append('C:/velodyne_points_no_ground')
            if self._last_scan_time is None:
                missing.append('D:/scan')
            missing += [f'C:{t}' for t, last in self._extra_last.items()
                        if last is None]
            if missing:
                self.get_logger().error(
                    f'DEAD INPUT after '
                    f'{self._dead_input_timeout_s:.0f}s: {missing} — '
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
            self._publish_state(now, active)
        elif self._last_state_pub is None \
                or now - self._last_state_pub >= STATE_HEARTBEAT_S:
            self._publish_state(now, active)
        if active:
            self._pub.publish(Twist())

    def _publish_state(self, now, active):
        # "OK" rather than an empty string so `ros2 topic echo` and bag
        # readers can tell "clear" apart from "no data".
        self._state_pub.publish(String(data=','.join(active) if active else 'OK'))
        self._last_state_pub = now


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

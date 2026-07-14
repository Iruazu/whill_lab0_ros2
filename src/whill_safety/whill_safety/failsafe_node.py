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

Deliberately omitted (post-demo backlog): jump detection (3-frame /pcl_pose
delta), SAFE_HOLD release hysteresis, G4 hardware 3-test acceptance. The
demo is operated with a walker beside the chair whose WHILL joystick
bypasses this path, so we accept the small risk of brief flapping on
condition-clear in exchange for keeping the code minimal for 2026-08-01.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist


FITNESS_MAX = 1.0
FITNESS_WINDOW_S = 2.0
PCL_POSE_TIMEOUT_S = 1.0
LAYER_A_HOLD_S = 1.0
PUBLISH_HZ = 20.0


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

        self._last_reinit_time = None
        self._fitness_bad_since = None
        self._converged_bad_since = None
        self._last_pose_time = None
        self._active_prev = ()

        self.create_subscription(
            Bool, '/reinitialization_requested', self._on_reinit, 10)
        self.create_subscription(
            DiagnosticArray, '/alignment_status', self._on_alignment, 10)
        self.create_subscription(
            PoseStamped, '/pcl_pose', self._on_pcl_pose, 10)

        self._pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(
            f'failsafe_node ready: fitness > {FITNESS_MAX} for '
            f'{FITNESS_WINDOW_S}s | pcl_pose silent > {PCL_POSE_TIMEOUT_S}s | '
            f'A hold {LAYER_A_HOLD_S}s | publish {PUBLISH_HZ:.0f} Hz')

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
        return tuple(out)

    def _tick(self):
        active = self._active_layers()
        if active != self._active_prev:
            if active:
                self.get_logger().warn(
                    f'failsafe ENGAGED — {list(active)}')
            else:
                self.get_logger().info('failsafe RELEASED')
            self._active_prev = active
        if active:
            self._pub.publish(Twist())


def main():
    rclpy.init()
    node = FailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

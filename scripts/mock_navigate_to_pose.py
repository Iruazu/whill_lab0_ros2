#!/usr/bin/env python3
"""Mock NavigateToPose action server for M7 dispatch E2E without a robot.

Stands in for Nav2's bt_navigator so `whill_dispatch/dispatch_node` can be
exercised end-to-end (submit -> queue -> goal -> feedback -> result, plus
cancel) with no live WHILL, no localizer, and no costmap. The real server
is `/navigate_to_pose` (nav2_msgs/action/NavigateToPose) published by
`whill_navigation/nav_launch.py`; this mock takes the same action name and
type so `dispatch_launch.py use_mock:=true` swaps it in transparently.

Behaviour of one goal:
  - accept immediately
  - stream feedback at FEEDBACK_HZ, ticking `distance_remaining` down
    linearly from START_DISTANCE_M to 0 over DURATION_S
  - return SUCCEEDED when it reaches 0
  - honour a cancel request at any point (returns CANCELED)

The linear ramp is deliberately ideal — it is a fixture for wiring, not a
Nav2 emulator. Real feedback is non-monotonic and replan-dependent
(distance can grow); dispatch_node's progress math must survive that, so
it clamps and holds max on its side (see dispatch_node). U1 in the plan
covers measuring the real feedback tomorrow.

Usage (standalone, e.g. against `ros2 action send_goal`):
    python3 scripts/mock_navigate_to_pose.py

    ros2 action send_goal -f /navigate_to_pose \\
        nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0}}}}"

Normally launched via `ros2 launch whill_dispatch dispatch_launch.py
use_mock:=true`, which runs this file with `python3`.

Tunables (env overrides, so a launch/test can speed it up without editing):
    MOCK_NAV_DURATION_S     total time per goal        (default 6.0)
    MOCK_NAV_START_DIST_M   initial distance_remaining (default 5.0)
    MOCK_NAV_FEEDBACK_HZ    feedback rate              (default 5.0)
"""

import os
import signal

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose


def _env_float(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


DURATION_S = _env_float('MOCK_NAV_DURATION_S', 6.0)
START_DISTANCE_M = _env_float('MOCK_NAV_START_DIST_M', 5.0)
FEEDBACK_HZ = _env_float('MOCK_NAV_FEEDBACK_HZ', 5.0)


class MockNavigateToPose(Node):

    def __init__(self):
        super().__init__('mock_navigate_to_pose')

        # Single-goal server: reject a second goal while one runs, matching
        # bt_navigator's default (one navigation at a time). dispatch_node's
        # FIFO queue is what serialises multiple submits, so the server does
        # not need to queue — and rejecting here surfaces a queue bug loudly
        # instead of silently interleaving two goals.
        self._busy = False

        self._server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )
        self.get_logger().info(
            f'mock /navigate_to_pose ready: {START_DISTANCE_M:.1f} m over '
            f'{DURATION_S:.1f} s, feedback {FEEDBACK_HZ:.0f} Hz')

    def _on_goal(self, _goal_request):
        if self._busy:
            self.get_logger().warn('goal rejected — already executing one')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        try:
            steps = max(1, int(DURATION_S * FEEDBACK_HZ))
            dt = DURATION_S / steps
            fb = NavigateToPose.Feedback()

            for i in range(steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('goal canceled')
                    return NavigateToPose.Result()

                remaining = START_DISTANCE_M * (1.0 - i / steps)
                fb.distance_remaining = float(remaining)
                fb.number_of_recoveries = 0
                # estimated_time_remaining left as a rough linear estimate;
                # dispatch_node ignores it, but a real client (RViz) reads it.
                secs = DURATION_S * (1.0 - i / steps)
                fb.estimated_time_remaining = Duration(
                    sec=int(secs), nanosec=int((secs % 1.0) * 1e9))
                goal_handle.publish_feedback(fb)

                # Sleep between ticks, not after the last one. A plain
                # time.sleep would block the executor and starve the cancel
                # check; the rate object yields so is_cancel_requested stays
                # fresh.
                if i < steps:
                    self._sleep(dt)

            goal_handle.succeed()
            self.get_logger().info('goal succeeded')
            return NavigateToPose.Result()
        finally:
            self._busy = False

    def _sleep(self, seconds):
        # rclpy has no cancel-aware sleep inside an execute_callback; a
        # short-timeout wait on a Rate would need a MultiThreadedExecutor to
        # keep servicing the action's own callbacks. This node runs on a
        # MultiThreadedExecutor (see main), so a blocking sleep here still
        # lets the cancel callback fire on another thread and set
        # is_cancel_requested, which the loop polls each tick.
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=seconds))


def _raise_keyboard_interrupt(*_):
    raise KeyboardInterrupt()


def main():
    # Same NO-signal-handler pattern as whill_safety/failsafe_node: let
    # plain Python handlers raise KeyboardInterrupt so `ros2 launch`
    # teardown (SIGTERM) and manual Ctrl-C both exit without the
    # "context is not valid" RCLError leaking out of spin.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    # MultiThreadedExecutor so the cancel callback runs concurrently with
    # the blocking execute loop; a single-threaded executor would deadlock
    # (the sleep in _execute would never yield to the cancel callback).
    from rclpy.executors import MultiThreadedExecutor

    node = MockNavigateToPose()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

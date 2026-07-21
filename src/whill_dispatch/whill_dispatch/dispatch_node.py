#!/usr/bin/env python3
"""M7 dispatch (配車) API boundary between the Web/tablet UI and Nav2.

One node collapses the four responsibilities the platform-pivot §3.5
gives `whill_dispatch` (demo-minimal — split into gateway/job-manager/
state-publisher is left for the M7 full build):

  (a) named waypoint resolution: load waypoints.yaml, map name -> (x,y,yaw)
  (b) job queue: FIFO over /dispatch/submit, one job at a time, cancelable
  (c) NavigateToPose action client: turn the head job into a goal, fold
      feedback (distance_remaining) into a 0..1 progress, reflect result
      into a phase
  (d) vehicle-state publish: fold /pcl_pose + /alignment_status + queue
      state into /dispatch/state at ~5 Hz

The Web side never touches /navigate_to_pose or /cmd_vel* (platform-pivot
§5 #4). It speaks only the four /dispatch/* interfaces over rosbridge, all
standard-typed (JSON-over-std_msgs/String + std_srvs/Trigger, ADR-0012
choice A — no custom rosidl interface for the demo):

  Web -> ROS  /dispatch/submit  (String)  JSON {"waypoint"|"point","type"}
  Web -> ROS  /dispatch/teleop  (String)  JSON {"active"} | {"vx","wz"}
  Web -> ROS  /dispatch/cancel  (Trigger)
  ROS -> Web  /dispatch/state   (String, 5 Hz)   JSON snapshot
  ROS -> Web  /dispatch/waypoints (String, 1 Hz) JSON list

The one ROS-internal topic dispatch publishes is /cmd_vel_teleop
(geometry_msgs/Twist), the String->Twist conversion of /dispatch/teleop into
twist_mux's teleop slot (priority 50, feat/teleop-rescue). Keeping that
conversion here — not in the browser — is what lets the Web side stay on
/dispatch/* only and never touch /cmd_vel* (ADR-0012). See the teleop section
below for the dead-man design (UI finger-up zero + dispatch watchdog +
twist_mux 0.5 s timeout).

Both ROS->Web topics are re-published on a timer rather than latched:
roslibjs subscribes volatile by default and would miss a transient_local
last-message, so periodic resend makes UI attach order irrelevant
(plan §QoS の割り切り).

Parameters:
  waypoints_path (str)  absolute path to waypoints.yaml. dispatch_launch.py
                        resolves docs/maps/<site>/waypoints.yaml and passes
                        it here (same repo-root recovery as nav_launch.py);
                        an empty value makes the node run with no waypoints
                        (useful for a bare smoke test).
  action_name (str)     NavigateToPose action, default /navigate_to_pose.
  state_rate_hz (float) /dispatch/state publish rate, default 5.0.
  waypoints_rate_hz (float) /dispatch/waypoints resend rate, default 1.0.
"""

import json
import math
import signal
from collections import deque

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Phase vocabulary shared with the Web UI. Kept as plain strings (not an
# enum) because they cross the JSON boundary verbatim.
PHASE_IDLE = 'IDLE'
PHASE_QUEUED = 'QUEUED'
PHASE_ACTIVE = 'ACTIVE'
PHASE_SUCCEEDED = 'SUCCEEDED'
PHASE_ABORTED = 'ABORTED'
PHASE_CANCELED = 'CANCELED'

# Manual-rescue teleop (feat/teleop-rescue). The iPad UI drives the chair out
# of a stuck spot through /dispatch/teleop; dispatch converts that to
# /cmd_vel_teleop, which twist_mux arbitrates at priority 50 (safety 100 >
# teleop 50 > nav 10, ADR-0007). The Layer-D pedestrian stop therefore still
# wins over a manual command — nothing here needs to enforce that, it falls out
# of the priority order.
#
# TELEOP_*_MAX are the clamp ceilings applied to untrusted browser input, NOT
# the nominal speed: the UI sends a fixed low speed (~0.2 m/s / 0.4 rad/s) for
# rescue. A payload asking for 99 m/s is clamped here, never forwarded raw.
TELEOP_VX_MAX = 0.3   # m/s
TELEOP_WZ_MAX = 0.6   # rad/s
# dead-man backup #2: if no teleop command arrives for this long while a stream
# is live, dispatch sends one zero-twist and goes silent. Kept below twist_mux's
# 0.5 s input timeout so dispatch brakes the chair *before* the mux would drop
# the slot — the two together mean a browser that stops sending (finger lifted,
# tab frozen, Wi-Fi glitch) cannot leave a stale non-zero teleop command latched.
TELEOP_WATCHDOG_S = 0.4
TELEOP_WATCHDOG_HZ = 20.0
# Idle auto-OFF: separate from the 0.4 s motion watchdog above (which only
# stops the *stream* between button presses). After this long with NO teleop
# command at all, drop `_teleop_active` back to False so a reconnect or a new
# device opening the page does not inherit an ON pad it never toggled — the
# "OFF by default, explicit-toggle-only" safety bias must survive a dropout.
# Long enough not to fight normal rescue (frequent button taps keep it alive).
TELEOP_IDLE_OFF_S = 20.0


def _yaw_to_quat(yaw):
    """Return (z, w) of the quaternion for a yaw about +Z (x=y=0)."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _quat_to_yaw(z, w):
    """Inverse of _yaw_to_quat for a planar (x=y=0) orientation."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class DispatchNode(Node):

    def __init__(self):
        super().__init__('dispatch_node')

        self.declare_parameter('waypoints_path', '')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('state_rate_hz', 5.0)
        self.declare_parameter('waypoints_rate_hz', 1.0)

        waypoints_path = self.get_parameter(
            'waypoints_path').get_parameter_value().string_value
        action_name = self.get_parameter(
            'action_name').get_parameter_value().string_value
        state_hz = self.get_parameter(
            'state_rate_hz').get_parameter_value().double_value
        wp_hz = self.get_parameter(
            'waypoints_rate_hz').get_parameter_value().double_value

        self._frame_id, self._waypoints = self._load_waypoints(waypoints_path)

        # Queue holds jobs not yet started. The active job is tracked
        # separately so queue_len in /dispatch/state means "waiting", not
        # "waiting + running".
        self._queue = deque()
        self._job_seq = 0
        self._active_job = None
        self._active_goal_handle = None
        self._phase = PHASE_IDLE
        # Progress state: D0 is the first *positive* distance_remaining we
        # see, and _progress is held at its running max so replan-driven
        # distance growth (real Nav2) cannot make the bar run backwards.
        self._d0 = None
        self._progress = 0.0

        # Vehicle state folded in from the localizer / alignment gate.
        self._pose = None          # dict {x, y, yaw} or None until first msg
        self._aligned = None       # bool or None until first /alignment_status

        # Manual-rescue teleop state. `_teleop_active` is the explicit ON/OFF
        # toggle (OFF is default — 誤操作防止); it only gates whether motion
        # commands are honored and drives the UI's button-enable, NOT whether
        # the chair moves. The actual motion gate is the dead-man: motion
        # happens only while browser commands keep arriving. `_teleop_streaming`
        # tracks whether we are currently forwarding a command stream, so the
        # watchdog sends exactly one stop-zero on release rather than spamming.
        self._teleop_active = False
        self._teleop_streaming = False
        self._teleop_last_cmd_time = None   # rclpy Time of last motion command

        # QoS note: default (reliable, volatile) on every /dispatch/* pub.
        # roslibjs speaks volatile; the periodic resend below (not latching)
        # is what makes late subscribers catch up.
        self._state_pub = self.create_publisher(String, '/dispatch/state', 10)
        self._waypoints_pub = self.create_publisher(
            String, '/dispatch/waypoints', 10)
        # /cmd_vel_teleop feeds twist_mux's teleop slot (priority 50). The Web
        # UI never publishes /cmd_vel* itself (ADR-0012 boundary); dispatch does
        # the String->Twist conversion here so the mux slot stays fed from
        # inside the /dispatch/* boundary.
        self._cmd_vel_teleop_pub = self.create_publisher(
            Twist, '/cmd_vel_teleop', 10)

        self.create_subscription(
            String, '/dispatch/submit', self._on_submit, 10)
        self.create_subscription(
            String, '/dispatch/teleop', self._on_teleop, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/pcl_pose', self._on_pcl_pose, 10)
        self.create_subscription(
            DiagnosticArray, '/alignment_status', self._on_alignment, 10)

        self.create_service(
            Trigger, '/dispatch/cancel', self._on_cancel)

        self._action_client = ActionClient(self, NavigateToPose, action_name)

        self.create_timer(1.0 / state_hz, self._publish_state)
        self.create_timer(1.0 / wp_hz, self._publish_waypoints)
        # Self-recovery for the "first job submitted before Nav2/mock is up"
        # case: nothing else re-triggers _try_start_next (it only runs on
        # submit and on job completion), so a queued job would stall forever.
        self._server_was_missing = False
        self.create_timer(1.0, self._retry_start)
        self.create_timer(1.0 / TELEOP_WATCHDOG_HZ, self._teleop_watchdog)

        self.get_logger().info(
            f'dispatch_node ready: {len(self._waypoints)} waypoint(s) from '
            f'{waypoints_path or "<none>"}, action {action_name!r}, '
            f'state {state_hz:.0f} Hz / waypoints {wp_hz:.0f} Hz')

    def _set_phase(self, phase):
        # Push a state snapshot the instant the phase changes rather than
        # waiting up to 1/state_rate for the timer. Without this a short-
        # lived phase (QUEUED between submit and goal-accept when the server
        # is already up) can pass entirely between two 5 Hz samples and the
        # UI never sees IDLE -> QUEUED -> ACTIVE. Event-driven push also
        # gives the tablet a crisper transition than polling latency.
        self._phase = phase
        self._publish_state()

    # --- (a) named waypoint resolution -----------------------------------

    def _load_waypoints(self, path):
        """Return (frame_id, {name: {label, x, y, yaw}}) from a yaml file.

        A missing / empty path is not fatal: the node still stands up so a
        bare smoke test (no maps checkout) can exercise the topics. It logs
        loudly instead so an operator does not mistake an empty list for a
        healthy load.
        """
        if not path:
            self.get_logger().warn(
                'waypoints_path is empty — no named waypoints loaded. '
                'submit will reject every name.')
            return 'map', {}
        try:
            with open(path) as f:
                doc = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            # YAMLError matters operationally: waypoints.yaml gets hand-edited
            # in the field (pose survey day), and a stray indent must degrade
            # to "no waypoints" instead of killing the whole dispatch boundary
            # at startup. YAMLError's str() includes the offending line/column.
            self.get_logger().error(
                f'cannot read waypoints_path {path!r}: {e}. '
                f'Running with no waypoints.')
            return 'map', {}

        frame_id = doc.get('frame_id', 'map')
        out = {}
        for wp in doc.get('waypoints', []):
            name = wp.get('name')
            if not name:
                self.get_logger().warn(f'skipping waypoint without name: {wp}')
                continue
            out[name] = {
                'label': wp.get('label', name),
                'x': float(wp.get('x', 0.0)),
                'y': float(wp.get('y', 0.0)),
                'yaw': float(wp.get('yaw', 0.0)),
            }
        return frame_id, out

    # --- (b) job queue ----------------------------------------------------

    def _on_submit(self, msg):
        try:
            req = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'ignoring malformed submit {msg.data!r}: {e}')
            return
        # json.loads happily returns int/list/None/str for non-object JSON
        # ("42", "[1]", "null", ...). Without this guard req.get() raises
        # AttributeError inside the callback and takes the whole node down.
        if not isinstance(req, dict):
            self.get_logger().warn(
                f'ignoring submit payload that is not a JSON object: '
                f'{msg.data!r}')
            return
        job_type = req.get('type', 'goto')

        # Two goal forms cross the boundary (README interface table, v2):
        #   {"waypoint": "<name>", ...}          named, resolved via yaml
        #   {"point": {"x","y","yaw?"}, ...}      arbitrary map-frame coord
        # 'point' wins if both are present — the tablet only ever sends one,
        # but an explicit precedence keeps a hand-crafted payload deterministic.
        # target is a {x,y,yaw} dict either way (the shape _pose_stamped wants);
        # display_name is what /dispatch/state carries in its 'waypoint' field
        # for the UI (a name the UI resolves to a label, or a coord string).
        raw_point = req.get('point')
        if raw_point is not None:
            target = self._parse_point(raw_point)
            if target is None:
                # Reject rather than crash: 'point' arrives from a browser and
                # may be a string, miss x/y, or carry NaN/Inf. _parse_point
                # returns None for all of those and we drop the submit here.
                self.get_logger().warn(
                    f'ignoring submit with invalid point {raw_point!r}')
                return
            display_name = f'({target["x"]:.1f}, {target["y"]:.1f})'
            # FREE-ness of the tapped cell is NOT checked here on purpose. The
            # UI already gates on the map png before enabling submit (first
            # safety net), and Nav2's global planner runs with
            # allow_unknown:false, so an UNKNOWN/OCC coordinate fails to plan
            # and the goal comes back ABORTED (second, authoritative net).
            # Re-reading the map in dispatch would duplicate that check against
            # a downscaled png and risk disagreeing with the planner's grid.
        else:
            name = req.get('waypoint')
            if name not in self._waypoints:
                # Covers both an unknown name and a payload with neither
                # 'point' nor 'waypoint' (name is None -> not in dict).
                self.get_logger().warn(
                    f'ignoring submit for unknown waypoint {name!r} '
                    f'(known: {sorted(self._waypoints)})')
                return
            target = self._waypoints[name]
            display_name = name

        self._job_seq += 1
        job = {'job_id': self._job_seq, 'type': job_type,
               'target': target, 'name': display_name}
        self._queue.append(job)
        self.get_logger().info(
            f'queued job {job["job_id"]} -> {display_name} ({job_type}), '
            f'queue_len now {len(self._queue)}')
        # Reflect QUEUED immediately if nothing is running so the Web UI (and
        # the AC's phase-transition check) observes IDLE/terminal -> QUEUED
        # before the goal is accepted and flips it to ACTIVE.
        if self._active_job is None:
            self._set_phase(PHASE_QUEUED)
        self._try_start_next()

    def _parse_point(self, raw):
        """Validate a submit 'point' object into {x, y, yaw} or None.

        Runs on untrusted browser input, so it never raises: a non-dict,
        missing/ non-numeric x or y, or a non-finite value (float('nan') does
        not raise — it produces NaN, which would then serialize as the
        non-standard "NaN" token and break the UI's JSON.parse) all return
        None so the caller drops the submit instead of the node crashing. yaw
        is optional and defaults to 0.0 (heading control is future work).
        """
        if not isinstance(raw, dict):
            return None
        try:
            x = float(raw['x'])
            y = float(raw['y'])
            yaw = float(raw.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None
        return {'x': x, 'y': y, 'yaw': yaw}

    def _retry_start(self):
        if self._active_job is None and self._queue:
            self._try_start_next()

    def _try_start_next(self):
        if self._active_job is not None or not self._queue:
            return
        if not self._action_client.server_is_ready():
            # Do not pop the job — the 1 s _retry_start timer keeps trying.
            # Warn only on the edge so the retry loop does not spam the log.
            if not self._server_was_missing:
                self.get_logger().warn(
                    'NavigateToPose server not available yet; job stays '
                    'queued (retrying every 1 s)')
                self._server_was_missing = True
            return
        if self._server_was_missing:
            self.get_logger().info('NavigateToPose server appeared')
            self._server_was_missing = False

        job = self._queue.popleft()
        self._active_job = job
        self._d0 = None
        self._progress = 0.0
        self._set_phase(PHASE_QUEUED)  # until the goal is accepted -> ACTIVE

        target = job['target']
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(target)
        self.get_logger().info(
            f'sending goal for job {job["job_id"]} -> {job["name"]} '
            f'({target["x"]:.2f}, {target["y"]:.2f}, yaw {target["yaw"]:.2f})')
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _pose_stamped(self, target):
        ps = PoseStamped()
        ps.header.frame_id = self._frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = target['x']
        ps.pose.position.y = target['y']
        qz, qw = _yaw_to_quat(target['yaw'])
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    # --- (c) NavigateToPose action client --------------------------------

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'goal for job {self._active_job["job_id"]} was REJECTED')
            self._set_phase(PHASE_ABORTED)
            self._finish_active()
            return
        self._active_goal_handle = goal_handle
        self._set_phase(PHASE_ACTIVE)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        dr = feedback_msg.feedback.distance_remaining
        # D0 = first positive distance_remaining. Nav2 emits 0.0 on the first
        # frame or two before the global plan lands; anchoring D0 there would
        # divide by ~0 and peg progress to 1 instantly. Wait for a real value.
        if dr > 0.0 and self._d0 is None:
            self._d0 = dr
        if self._d0:
            p = 1.0 - dr / self._d0
            p = min(1.0, max(0.0, p))
            # Hold the running max: real feedback is non-monotonic (replan
            # can grow distance_remaining), and a bar that jumps backward
            # reads as a fault to a rider. Monotonic non-decreasing is the
            # honest UX here (plan §リスク).
            self._progress = max(self._progress, p)

    def _on_result(self, future):
        status = future.result().status
        job_id = self._active_job['job_id'] if self._active_job else '?'
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._progress = 1.0
            self._set_phase(PHASE_SUCCEEDED)
            self.get_logger().info(f'job {job_id} SUCCEEDED')
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_phase(PHASE_CANCELED)
            self.get_logger().info(f'job {job_id} CANCELED')
        else:
            # STATUS_ABORTED or anything else non-terminal-success.
            self._set_phase(PHASE_ABORTED)
            self.get_logger().warn(f'job {job_id} ABORTED (status {status})')
        self._finish_active()

    def _finish_active(self):
        self._active_job = None
        self._active_goal_handle = None
        # Start the next queued job if any. The terminal phase set above
        # stays visible until _try_start_next flips it to QUEUED/ACTIVE, so
        # a burst of one job still shows its SUCCEEDED/ABORTED/CANCELED.
        self._try_start_next()

    def _on_cancel(self, _request, response):
        if self._active_goal_handle is None:
            response.success = False
            response.message = 'no active job to cancel'
            self.get_logger().info('cancel requested but no active job')
            return response
        job_id = self._active_job['job_id'] if self._active_job else '?'
        self.get_logger().info(f'cancel requested for job {job_id}')
        # Fire and forget: the server's cancel path drives the goal to
        # STATUS_CANCELED, which _on_result observes and turns into
        # PHASE_CANCELED. Waiting on the future here would block the service
        # callback against the executor.
        self._active_goal_handle.cancel_goal_async()
        response.success = True
        response.message = f'cancel sent for job {job_id}'
        return response

    # --- manual-rescue teleop --------------------------------------------

    def _on_teleop(self, msg):
        """Turn a /dispatch/teleop command into /cmd_vel_teleop (or a toggle).

        Two payload shapes cross the boundary, both JSON-over-String so the
        Web side stays inside /dispatch/* (ADR-0012):

          {"active": true|false}       manual-rescue mode ON/OFF. Explicit
                                       toggle, OFF is default (誤操作防止).
                                       Only literal boolean true enables;
                                       anything else disables (safe bias).
          {"vx": <m/s>, "wz": <rad/s>} a motion command. Honored only while
                                       manual mode is active, clamped to the
                                       rescue ceilings. The *absence* of these
                                       commands is dead-man #1 (see
                                       _teleop_watchdog).

        Runs on untrusted browser input, so it never raises — same robustness
        bar as _on_submit / _parse_point.
        """
        try:
            req = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'ignoring malformed teleop {msg.data!r}: {e}')
            return
        if not isinstance(req, dict):
            self.get_logger().warn(
                f'ignoring teleop payload that is not a JSON object: '
                f'{msg.data!r}')
            return

        if 'active' in req:
            # `is True` (not bool()) on purpose: bool("false") is True, and a
            # hand-typed {"active":"false"} must not silently arm teleop. Only
            # the browser's real JSON boolean true enables; all else -> OFF.
            self._set_teleop_active(req['active'] is True)

        if 'vx' not in req and 'wz' not in req:
            return  # a pure toggle (or empty) payload — nothing to drive

        cmd = self._parse_teleop(req)
        if cmd is None:
            self.get_logger().warn(f'ignoring invalid teleop command {msg.data!r}')
            return
        if not self._teleop_active:
            # A motion command while manual mode is OFF is dropped, not driven:
            # the toggle is the 誤操作防止 gate. The UI only enables the
            # direction buttons while ON, so this normally never fires — it is
            # the guard for a stray or hand-crafted payload.
            self.get_logger().warn(
                'dropping teleop motion while manual mode is OFF',
                throttle_duration_sec=2.0)
            return

        vx, wz = cmd
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self._cmd_vel_teleop_pub.publish(t)
        self._teleop_last_cmd_time = self.get_clock().now()
        self._teleop_streaming = True

    def _parse_teleop(self, raw):
        """Validate a teleop motion payload into clamped (vx, wz) or None.

        Same untrusted-input contract as _parse_point: a non-dict, missing/
        non-numeric vx or wz, or a non-finite value returns None so the caller
        drops the command instead of the node crashing or emitting a NaN twist
        onto /cmd_vel_teleop. Finite values are clamped to the rescue ceilings.
        """
        if not isinstance(raw, dict):
            return None
        try:
            vx = float(raw['vx'])
            wz = float(raw['wz'])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return None
        vx = max(-TELEOP_VX_MAX, min(TELEOP_VX_MAX, vx))
        wz = max(-TELEOP_WZ_MAX, min(TELEOP_WZ_MAX, wz))
        return vx, wz

    def _set_teleop_active(self, active):
        if active == self._teleop_active:
            return
        self._teleop_active = active
        if active:
            # Start the idle-OFF clock from the ON moment so a pad toggled on
            # but never driven still auto-OFFs after TELEOP_IDLE_OFF_S (SF-2).
            self._teleop_last_cmd_time = self.get_clock().now()
        if not active:
            # Turning manual mode OFF must brake now and release the slot: one
            # zero-twist, then go silent so twist_mux's 0.5 s timeout drops
            # teleop and navigation (priority 10) or a stopped bus resumes.
            self._stop_teleop()
        self.get_logger().info(f'teleop {"ENABLED" if active else "DISABLED"}')
        # Reflect the toggle in /dispatch/state immediately rather than waiting
        # up to 1/state_rate, so the UI's button-enable tracks the truth.
        self._publish_state()

    def _stop_teleop(self):
        """Send one zero-twist and stop the teleop publish stream.

        Called on manual-mode OFF and by the watchdog when browser commands
        stop arriving. The single zero brakes the chair immediately; going
        silent afterwards lets twist_mux's 0.5 s timeout release the teleop
        slot. Idempotent-ish: only ever emits when a stream was live, so it
        does not spam zeros every watchdog tick.
        """
        self._cmd_vel_teleop_pub.publish(Twist())
        self._teleop_streaming = False

    def _teleop_watchdog(self):
        # dead-man #2. The UI sends a zero on finger-up (dead-man #1); this
        # covers the case where that zero never arrives (tab frozen, Wi-Fi
        # glitch, crash). Fires once per release — _stop_teleop clears
        # _teleop_streaming so we do not re-enter until the next command stream.
        if self._teleop_last_cmd_time is None:
            return
        dt = (self.get_clock().now()
              - self._teleop_last_cmd_time).nanoseconds * 1e-9
        if self._teleop_streaming and dt > TELEOP_WATCHDOG_S:
            self.get_logger().warn(
                f'teleop watchdog: no command for {dt:.2f}s '
                f'(> {TELEOP_WATCHDOG_S}s) — stopping')
            self._stop_teleop()
        # Idle auto-OFF (SF-2): after prolonged silence, clear the ON flag so a
        # reconnect does not inherit a manual-mode pad nobody toggled on.
        if self._teleop_active and dt > TELEOP_IDLE_OFF_S:
            self.get_logger().info(
                f'teleop idle {dt:.0f}s (> {TELEOP_IDLE_OFF_S}s) — auto OFF')
            self._set_teleop_active(False)

    # --- (d) vehicle-state publish ---------------------------------------

    def _on_pcl_pose(self, msg):
        p = msg.pose.pose
        pose = {
            'x': p.position.x,
            'y': p.position.y,
            'yaw': _quat_to_yaw(p.orientation.z, p.orientation.w),
        }
        # A NaN here would be serialized by json.dumps as the non-standard
        # token "NaN", which browser-side JSON.parse rejects — every 5 Hz
        # state update would then throw and silently freeze the UI. Keep the
        # last finite pose instead.
        if not all(math.isfinite(v) for v in pose.values()):
            self.get_logger().warn(
                'dropping non-finite /pcl_pose sample', throttle_duration_sec=5.0)
            return
        self._pose = pose

    def _on_alignment(self, msg):
        for status in msg.status:
            for kv in status.values:
                if kv.key == 'has_converged':
                    self._aligned = str(kv.value).strip().lower() in (
                        'true', '1', 'ok')
                    return

    def _publish_state(self):
        # queue_len counts only jobs still waiting. The active one is
        # reported through job_id/waypoint/phase, not the queue length.
        active_wp = self._active_job['name'] if self._active_job else None
        active_id = self._active_job['job_id'] if self._active_job else None
        snapshot = {
            'job_id': active_id,
            'phase': self._phase,
            'waypoint': active_wp,
            'progress': round(self._progress, 3),
            'queue_len': len(self._queue),
            'pose': self._pose,
            'aligned': self._aligned,
            # Manual-rescue toggle state — drives the UI's direction-button
            # enable. Note this is NOT "the chair is moving": with teleop
            # active but no button held, the dead-man keeps /cmd_vel_teleop
            # silent and the chair stays put.
            'teleop_active': self._teleop_active,
        }
        self._state_pub.publish(String(data=json.dumps(snapshot)))

    def _publish_waypoints(self):
        out = [
            {'name': name, 'label': wp['label'],
             'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw']}
            for name, wp in self._waypoints.items()
        ]
        self._waypoints_pub.publish(String(data=json.dumps(out)))


def _raise_keyboard_interrupt(*_):
    raise KeyboardInterrupt()


def main():
    # Same clean-shutdown pattern as whill_safety/failsafe_node: plain Python
    # signal handlers raise KeyboardInterrupt so `ros2 launch` teardown
    # (SIGTERM) and manual Ctrl-C both avoid the "context is not valid"
    # RCLError that rclpy's own handler leaks out of spin in humble.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    node = DispatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

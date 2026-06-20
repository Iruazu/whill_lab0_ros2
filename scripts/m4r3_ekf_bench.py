#!/usr/bin/env python3
"""Replay a ROS 2 bag and compute end-to-end error on `/odometry/filtered`.

Used to validate Issue #37 (M4R-3) acceptance criterion 3:

    "手押し 10 m 直進試験で `/odometry/filtered.pose.pose.position` の
     終端誤差 ≤ 0.5 m (= 5 %)"

The script reads `/odometry/filtered` directly from a bag (i.e. assumes
you have already recorded a run with the EKF running live, or have
replayed a sensor-only bag through the EKF and re-recorded the output).
No live ROS graph is required; this is pure offline analysis.

Outputs:

- A header line summarising path length, end-pose, straight-line end
  distance, and yaw drift.
- If `--csv` is given, dumps a CSV of `t,x,y,yaw,vx,vyaw` for plotting.

Usage examples:

    # Just print the summary (most common; used to gate Issue #37):
    scripts/m4r3_ekf_bench.py docs/m4-bench-data/m4r3_push_10m_2026-06-22

    # Dump the trajectory for plotting:
    scripts/m4r3_ekf_bench.py docs/m4-bench-data/m4r3_push_10m_2026-06-22 \\
        --csv out.csv

    # Restrict to a window inside the bag (start was static, drift the
    # static portion off so it doesn't count toward path length):
    scripts/m4r3_ekf_bench.py <bag> --t-start 5.0 --t-end 35.0

Assumptions:

- The bag is a ROS 2 (sqlite3 or mcap) bag readable by `rosbag2_py`.
- `/odometry/filtered` is recorded in the bag. Override with `--topic`.
- The trajectory is start/end at the same physical point for the 10 m
  straight push case; end-distance is then the straight-line gap, not
  loop closure. For non-loop tests pass `--no-reset-origin` if the bag
  already starts at (0, 0).
"""

import argparse
import csv
import math
import sys
from pathlib import Path

# rosbag2_py / message classes are only needed at runtime; importing at
# top level keeps `--help` cheap and the import error message obvious if
# robot_localization / ros2 env is not sourced.
try:
    import rclpy.serialization
    import rosbag2_py
    from nav_msgs.msg import Odometry
except ImportError as exc:
    print(f'ERROR: ROS 2 Python env not sourced ({exc}).', file=sys.stderr)
    print('Run `source /opt/ros/humble/setup.bash` and retry.', file=sys.stderr)
    raise SystemExit(2)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from a quaternion (ZYX convention, 2D-mode safe).

    robot_localization with `two_d_mode: true` zeros roll/pitch, so the
    full Euler decomposition reduces to the standard yaw formula.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    """Open the bag; `rosbag2_py` auto-detects sqlite3 vs mcap."""
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('bag', help='Path to the ROS 2 bag directory.')
    p.add_argument('--topic', default='/odometry/filtered',
                   help='Odometry topic to analyse (default /odometry/filtered).')
    p.add_argument('--csv', default=None,
                   help='If set, dump per-sample t,x,y,yaw,vx,vyaw to this CSV.')
    p.add_argument('--t-start', type=float, default=None,
                   help='Discard samples earlier than this (seconds from bag start).')
    p.add_argument('--t-end', type=float, default=None,
                   help='Discard samples later than this (seconds from bag start).')
    p.add_argument('--no-reset-origin', action='store_true',
                   help='Do NOT subtract the first surviving sample from x/y/yaw. '
                        'Default is to reset so the trajectory begins at (0, 0, 0).')
    args = p.parse_args()

    if not Path(args.bag).exists():
        print(f'ERROR: bag path does not exist: {args.bag}', file=sys.stderr)
        return 1

    reader = open_reader(args.bag)

    # Find the message type for the requested topic. We only need
    # nav_msgs/Odometry but verify so a wrong --topic gives a clean error.
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        print(f'ERROR: topic {args.topic!r} not in bag. Available: '
              f'{sorted(topic_types)}', file=sys.stderr)
        return 1
    if topic_types[args.topic] != 'nav_msgs/msg/Odometry':
        print(f'ERROR: topic {args.topic} is {topic_types[args.topic]}, '
              f'expected nav_msgs/msg/Odometry', file=sys.stderr)
        return 1

    samples: list[tuple[float, float, float, float, float, float]] = []
    bag_t0: float | None = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != args.topic:
            continue
        msg: Odometry = rclpy.serialization.deserialize_message(data, Odometry)
        # Use header.stamp rather than bag write time. The EKF stamps at
        # publish time so the two agree to within a tick, but header time
        # is what the upstream consumer (Nav2) sees.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if bag_t0 is None:
            bag_t0 = t
        rel_t = t - bag_t0
        if args.t_start is not None and rel_t < args.t_start:
            continue
        if args.t_end is not None and rel_t > args.t_end:
            continue
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vyaw = msg.twist.twist.angular.z
        samples.append((rel_t, x, y, yaw, vx, vyaw))

    if not samples:
        print('ERROR: no /odometry/filtered samples in selected window.',
              file=sys.stderr)
        return 1

    if not args.no_reset_origin:
        t0, x0, y0, yaw0, _, _ = samples[0]
        # Reset time to 0 too so CSVs from different runs are directly comparable.
        samples = [
            (t - t0, x - x0, y - y0, yaw - yaw0, vx, vyaw)
            for (t, x, y, yaw, vx, vyaw) in samples
        ]

    # Path length: sum of Euclidean steps between consecutive samples.
    path_len = 0.0
    for (_, x_a, y_a, _, _, _), (_, x_b, y_b, _, _, _) in zip(samples, samples[1:]):
        path_len += math.hypot(x_b - x_a, y_b - y_a)

    t_first, x_first, y_first, yaw_first, _, _ = samples[0]
    t_last, x_last, y_last, yaw_last, _, _ = samples[-1]
    end_dist = math.hypot(x_last - x_first, y_last - y_first)
    yaw_drift = yaw_last - yaw_first
    # Wrap to (-pi, pi] so a near-zero physical drift doesn't show as ~2pi.
    while yaw_drift > math.pi:
        yaw_drift -= 2 * math.pi
    while yaw_drift <= -math.pi:
        yaw_drift += 2 * math.pi

    duration = t_last - t_first
    rate = (len(samples) - 1) / duration if duration > 0 else float('nan')

    print(f'Bag: {args.bag}')
    print(f'Topic: {args.topic}')
    print(f'Samples: {len(samples)}  Duration: {duration:.2f} s  '
          f'Mean rate: {rate:.2f} Hz')
    print(f'Path length: {path_len:.3f} m')
    print(f'Start pose:  ({x_first:+.3f}, {y_first:+.3f}, yaw {yaw_first:+.4f} rad)')
    print(f'End pose:    ({x_last:+.3f}, {y_last:+.3f}, yaw {yaw_last:+.4f} rad)')
    print(f'End distance from start (straight-line): {end_dist:.3f} m')
    print(f'Yaw drift (end - start, wrapped): {yaw_drift:+.4f} rad '
          f'({math.degrees(yaw_drift):+.2f} deg)')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'vyaw'])
            w.writerows(samples)
        print(f'Wrote {args.csv}', file=sys.stderr)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

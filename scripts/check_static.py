#!/usr/bin/env python3
"""Sanity-check that a rosbag2 directory really captured a stationary segment.

Reads `/imu/data_raw` from the bag and prints angular-velocity and
linear-acceleration statistics, plus a count of "motion burst" 0.2 s windows
where the gyro RMS exceeds 0.05 rad/s. Used to validate the M3 chair-mounted
static reference bag (`m3_chair_static_*`) and the leading static window
of the drive bag (`m3_chair_motion_*`, first 5 s before the chair starts
moving).

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/check_static.py <bag_directory> [seconds]

If `seconds` is given, only the first that many seconds of /imu/data_raw
are analysed (handy for verifying the leading static window of a
motion bag without scanning the whole drive).
"""

import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def stats(values):
    n = len(values)
    return (
        min(values),
        sum(values) / n,
        max(values),
        math.sqrt(sum(v * v for v in values) / n),
    )


def main(bag_path: str, max_seconds: float | None = None) -> int:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    gyr_mag, acc_mag = [], []
    gx, gy, gz = [], [], []
    ax, ay, az = [], [], []
    t0 = None

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/imu/data_raw":
            continue
        m = deserialize_message(data, Imu)
        if max_seconds is not None:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = t
            if t - t0 > max_seconds:
                break
        gxv, gyv, gzv = (
            m.angular_velocity.x,
            m.angular_velocity.y,
            m.angular_velocity.z,
        )
        axv, ayv, azv = (
            m.linear_acceleration.x,
            m.linear_acceleration.y,
            m.linear_acceleration.z,
        )
        gx.append(gxv); gy.append(gyv); gz.append(gzv)
        ax.append(axv); ay.append(ayv); az.append(azv)
        gyr_mag.append(math.sqrt(gxv * gxv + gyv * gyv + gzv * gzv))
        acc_mag.append(math.sqrt(axv * axv + ayv * ayv + azv * azv))

    n = len(gyr_mag)
    if n == 0:
        print(f"No /imu/data_raw messages found in {bag_path}", file=sys.stderr)
        return 1

    print(f"IMU samples: {n}\n")
    fmt = "  {:24s} min={: .4f}  mean={: .4f}  max={: .4f}  rms={: .4f}"
    print("angular_velocity (rad/s)")
    print(fmt.format("|omega|", *stats(gyr_mag)))
    print(fmt.format("x", *stats(gx)))
    print(fmt.format("y", *stats(gy)))
    print(fmt.format("z", *stats(gz)))
    print()
    print("linear_acceleration (m/s^2)")
    print(fmt.format("|a|", *stats(acc_mag)))
    print(fmt.format("x", *stats(ax)))
    print(fmt.format("y", *stats(ay)))
    print(fmt.format("z", *stats(az)))
    print()

    window = 20  # ~0.2 s at 100 Hz
    threshold = 0.05  # rad/s; well above typical RT IMU noise floor (~0.02)
    burst_count = 0
    total_windows = n // window
    for i in range(0, n - window, window):
        w = gyr_mag[i:i + window]
        rms = math.sqrt(sum(x * x for x in w) / len(w))
        if rms > threshold:
            burst_count += 1
    print(
        f"Motion bursts (0.2 s windows where gyro RMS > {threshold} rad/s): "
        f"{burst_count} of {total_windows}"
    )

    return 0


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        print(f"Usage: {sys.argv[0]} <bag_directory> [seconds]", file=sys.stderr)
        raise SystemExit(2)
    bag = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) == 3 else None
    raise SystemExit(main(bag, secs))

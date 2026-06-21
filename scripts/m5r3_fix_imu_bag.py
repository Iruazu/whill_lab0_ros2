#!/usr/bin/env python3
"""Rewrite a rosbag2 with /imu/data_raw's linear_acceleration negated.

DEPRECATED (Issue #56):
  The runtime fix lives in whill_sensors_bringup/imu_sign_corrector
  (Issue #56). Newly recorded bags taken with the post-#56 launch carry
  /imu/data_rep145 already REP-145-compliant; downstream consumers (EKF,
  the future scan-to-map localizer, GLIM/FAST-LIO SAM bag replays) should
  subscribe to /imu/data_rep145 instead of pre-rewriting the bag.

  This script is retained only for historical bags recorded before #56
  was merged — e.g. docs/m5r-bench-data/2026-06-21-loop-outdoor/bag/
  which carries the raw inverted /imu/data_raw. For those bags the
  rewritten output is still the canonical input to the SLAM comparison.
  Do not use this script on bags recorded after #56.

Why this script exists:
  PCMK-G3X (MPU-9250 + LPC1343F USB firmware) reports linear_acceleration
  as the gravity-acceleration vector itself, not as REP-145 specific
  force. At rest with the IMU mounted +Z up, /imu/data_raw shows
  z ≈ -9.81 instead of the REP-145-expected +9.81 (measured -9.71 on
  the 2026-06-21 outdoor bag, see manifest). The rt_usb_9axisimu_driver
  is a 1:1 byte passthrough — the inversion happens in the LPC1343F
  firmware and is not user-tunable.

  Noetic FAST-LIO tolerated this because IMU_Processing.hpp:196 self-
  estimates the gravity direction (init_state.grav = S2(-mean_acc/...)).
  GLIM does not — its initial T_world_imu absorbs the inverted gravity
  as a ~171 deg rotation about X and the IMU-vs-LiDAR validation stays
  poor (rot=0.6, trans=0.2, vel=0.2) throughout the run, with no
  trajectory file emitted on exit.

Scope:
  This is an M5R-3 (Phase B SLAM comparison) workaround. Both GLIM and
  FAST-LIO SAM evaluation runs read from the rewritten bag so they see
  the same REP-145-compliant data and the comparison is symmetric. The
  permanent fix — a republisher node in whill_sensors_bringup so EKF /
  future scan-to-map localizer / re-recorded bags don't carry the bug —
  is tracked separately and is out of M5R-3 scope.

What gets rewritten:
  /imu/data_raw      linear_acceleration.{x,y,z} *= -1
                     orientation, angular_velocity, covariances untouched
                     (MPU-9250 gyro convention matches REP-103; only accel
                     is inverted by the firmware. See legacy-archaeologist
                     report referenced from PR #48.)
  all other topics   copied verbatim (no deserialize/serialize cost)

Usage:
  python3 scripts/m5r3_fix_imu_bag.py <input-bag-dir> <output-bag-dir>

Exits non-zero if the output directory already exists (refuse to
overwrite) or if /imu/data_raw is missing from the input bag.
"""

import pathlib
import sys

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message

IMU_TOPIC = '/imu/data_raw'


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__.strip().split('\n\nUsage')[1].strip(), file=sys.stderr)
        return 2

    in_dir = pathlib.Path(sys.argv[1]).resolve()
    out_dir = pathlib.Path(sys.argv[2]).resolve()

    if not (in_dir / 'metadata.yaml').is_file():
        print(f'ERROR: {in_dir}/metadata.yaml not found.', file=sys.stderr)
        return 2
    if out_dir.exists():
        # Refuse to overwrite — the rewritten bag is the canonical input
        # for SLAM comparison and silently clobbering it would invalidate
        # any in-flight manifest references.
        print(f'ERROR: {out_dir} already exists; refusing to overwrite.',
              file=sys.stderr)
        return 2

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(in_dir), storage_id='sqlite3'),
        ConverterOptions('', ''),
    )
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(out_dir), storage_id='sqlite3'),
        ConverterOptions('', ''),
    )

    imu_msg_class = None
    topic_types = reader.get_all_topics_and_types()
    for tm in topic_types:
        writer.create_topic(tm)
        if tm.name == IMU_TOPIC:
            imu_msg_class = get_message(tm.type)

    if imu_msg_class is None:
        print(f'ERROR: {IMU_TOPIC} not present in {in_dir}', file=sys.stderr)
        return 2

    n_imu = 0
    n_other = 0
    while reader.has_next():
        topic, raw_data, ts = reader.read_next()
        if topic == IMU_TOPIC:
            msg = deserialize_message(raw_data, imu_msg_class)
            msg.linear_acceleration.x = -msg.linear_acceleration.x
            msg.linear_acceleration.y = -msg.linear_acceleration.y
            msg.linear_acceleration.z = -msg.linear_acceleration.z
            raw_data = serialize_message(msg)
            n_imu += 1
        else:
            n_other += 1
        writer.write(topic, raw_data, ts)

    print(f'rewrote {n_imu} {IMU_TOPIC} messages, '
          f'copied {n_other} other messages')
    print(f'output bag: {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

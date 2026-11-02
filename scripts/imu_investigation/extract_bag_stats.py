#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Extract IMU samples and per-scan LiDAR metadata from a rosbag2 (read-only).

IMU  -> imu.npz : header stamp, bag receive time, acc(3), gyro(3), quat(4)
LiDAR-> scans.npz: header stamp, bag receive time, n_points, time-field min/max,
                   ring min/max, field names (once)

Why sqlite direct read: rosbag2_py SequentialReader deserializes every topic
in order; reading sqlite per-topic lets us skip the 13 GB of clouds when only
IMU is needed (--no-clouds). The bag is opened read-only (URI mode=ro).
"""
import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(DT[f.datatype])
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                     'itemsize': msg.point_step})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_db3')
    ap.add_argument('out_dir')
    ap.add_argument('--imu-topic', default='/imu/data_rep145')
    ap.add_argument('--points-topic', default='/velodyne_points')
    ap.add_argument('--no-clouds', action='store_true')
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(f'file:{a.bag_db3}?mode=ro', uri=True)
    tid = {n: i for i, n in con.execute('select id,name from topics')}

    rows = con.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                       (tid[a.imu_topic],))
    st, rt, acc, gyr, q = [], [], [], [], []
    for t, d in rows:
        m = deserialize_message(d, Imu)
        st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        rt.append(t * 1e-9)
        acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        gyr.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        q.append((m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w))
    np.savez(out / 'imu.npz', stamp=np.array(st), recv=np.array(rt), acc=np.array(acc),
             gyro=np.array(gyr), quat=np.array(q), frame_id=m.header.frame_id,
             acc_cov=np.array(m.linear_acceleration_covariance),
             gyro_cov=np.array(m.angular_velocity_covariance))
    print(f'imu: {len(st)} msgs, frame={m.header.frame_id}', file=sys.stderr)
    if a.no_clouds:
        return

    rows = con.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                       (tid[a.points_topic],))
    st, rt, n, tmin, tmax, rmin, rmax = [], [], [], [], [], [], []
    fields = None
    for i, (t, d) in enumerate(rows):
        m = deserialize_message(d, PointCloud2)
        dt = cloud_dtype(m)
        arr = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
        if fields is None:
            fields = [(f.name, f.datatype, f.offset) for f in m.fields]
            print('fields', fields, 'frame', m.header.frame_id, file=sys.stderr)
        st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        rt.append(t * 1e-9)
        n.append(len(arr))
        if 'time' in arr.dtype.names and len(arr):
            tmin.append(float(arr['time'].min()))
            tmax.append(float(arr['time'].max()))
        else:
            tmin.append(np.nan)
            tmax.append(np.nan)
        rmin.append(int(arr['ring'].min()) if len(arr) else -1)
        rmax.append(int(arr['ring'].max()) if len(arr) else -1)
        if i % 2000 == 0:
            print(i, file=sys.stderr)
    np.savez(out / 'scans.npz', stamp=np.array(st), recv=np.array(rt), n=np.array(n),
             tmin=np.array(tmin), tmax=np.array(tmax), rmin=np.array(rmin), rmax=np.array(rmax),
             fields=np.array(fields, dtype=object))


if __name__ == '__main__':
    main()

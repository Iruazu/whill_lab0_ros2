#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Re-project Velodyne PointCloud2 messages recorded with the wrong calibration.

Use case: a VLP-16 Puck Hi-Res (+-10 deg) was driven with VLP16db.yaml (+-15 deg),
so every point's elevation (and the 11.2 mm vertical-offset term) is wrong.
The range d and azimuth are still correct, so each point can be re-projected
exactly as long as the recorded cloud is in the sensor frame and still carries
the `ring` field.

Math (velodyne_pointcloud rawdata.cpp unpack_vlp16, with dist/focal/horiz
corrections all zero, which this tool verifies in both yaml files):
    xy = d*cos(v) - o*sin(v),  z = d*sin(v) + o*cos(v)
    =>  d = xy*cos(v) + z*sin(v)       (exact; the o terms cancel)
    new: xy' = d*cos(v') - o'*sin(v'), z' = d*sin(v') + o'*cos(v')
    x' = x * xy'/xy,  y' = y * xy'/xy  (azimuth unchanged)
where (v, o) come from the source yaml and (v', o') from the destination yaml.

Ring -> laser mapping is re-derived the same way the driver does it
(calibration.cpp: rings are assigned by sorting vert_correction ascending), so
nothing is hard-coded; the source/destination lasers are matched by laser_id
(the physical firing channel).

The input bag is only read. All non-cloud topics are copied unchanged, with
the same receive timestamps and QoS metadata. Optional --start/--duration cut a
segment (seconds relative to the bag start); topics listed in --latched
(default /tf_static) are always written, re-stamped to the segment start so a
cut segment still carries the static TF.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
MUST_MATCH = ('dist_correction', 'dist_correction_x', 'dist_correction_y',
              'focal_distance', 'focal_slope', 'horiz_offset_correction', 'rot_correction')


def load_calib(path):
    with open(path) as f:
        cal = yaml.safe_load(f)
    lasers = {int(l['laser_id']): l for l in cal['lasers']}
    # ring assignment exactly as velodyne_pointcloud calibration.cpp
    order = sorted(lasers, key=lambda i: lasers[i]['vert_correction'])
    ring_to_id = {r: lid for r, lid in enumerate(order)}
    return lasers, ring_to_id


def build_tables(src_yaml, dst_yaml):
    src, src_r2id = load_calib(src_yaml)
    dst, dst_r2id = load_calib(dst_yaml)
    if set(src) != set(dst):
        sys.exit('laser_id sets differ between calibration files')
    for lid in src:
        for k in MUST_MATCH:
            sval, dval = float(src[lid].get(k, 0.0)), float(dst[lid].get(k, 0.0))
            # rot_correction only shifts azimuth, which we keep, so it may be
            # non-zero as long as both files agree; the rest must be zero.
            if abs(sval - dval) > 1e-12 or (k != 'rot_correction' and abs(sval) > 1e-12):
                sys.exit(f'laser {lid}: field {k} is non-zero or differs; the exact inverse '
                         'implemented here assumes only vert_correction / '
                         'vert_offset_correction change and the other terms are zero.')
    n = len(src)
    sv, so, dv, do = (np.zeros(n) for _ in range(4))
    for r in range(n):
        lid = src_r2id[r]
        sv[r] = src[lid]['vert_correction']
        so[r] = src[lid].get('vert_offset_correction', 0.0)
        dv[r] = dst[lid]['vert_correction']
        do[r] = dst[lid].get('vert_offset_correction', 0.0)
    # The ring index in the *output* cloud should follow the destination's
    # elevation order; for VLP16 vs VLP16 Hi-Res both orders are identical.
    dst_ring_of_src_ring = np.array([
        [rr for rr, lid in dst_r2id.items() if lid == src_r2id[r]][0] for r in range(n)])
    return sv, so, dv, do, dst_ring_of_src_ring


def cloud_view(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(('>' if msg.is_bigendian else '<') + DT[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                   'itemsize': msg.point_step})
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()
    return buf, buf.view(dt)[: msg.width * msg.height]


def reproject(msg, tables, ring_field, stats):
    sv, so, dv, do, ring_map = tables
    buf, arr = cloud_view(msg)
    if len(arr) == 0:
        return msg
    r = arr[ring_field].astype(np.int64)
    x = arr['x'].astype(np.float64)
    y = arr['y'].astype(np.float64)
    z = arr['z'].astype(np.float64)
    xy = np.hypot(x, y)
    v, o, v2, o2 = sv[r], so[r], dv[r], do[r]
    d = xy * np.cos(v) + z * np.sin(v)
    # consistency check: recomputed z from d must equal the recorded z (float32 noise)
    zchk = d * np.sin(v) + o * np.cos(v)
    stats['max_zchk'] = max(stats.get('max_zchk', 0.0), float(np.nanmax(np.abs(zchk - z))))
    xy2 = d * np.cos(v2) - o2 * np.sin(v2)
    z2 = d * np.sin(v2) + o2 * np.cos(v2)
    with np.errstate(invalid='ignore', divide='ignore'):
        s = np.where(xy > 1e-6, xy2 / xy, 0.0)
    arr['x'] = (x * s).astype(arr['x'].dtype)
    arr['y'] = (y * s).astype(arr['y'].dtype)
    arr['z'] = z2.astype(arr['z'].dtype)
    arr[ring_field] = ring_map[r].astype(arr[ring_field].dtype)
    msg.data = buf.tobytes()
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input_bag')
    ap.add_argument('output_bag')
    ap.add_argument('--src-calib', required=True, help='yaml the driver used (e.g. VLP16db.yaml)')
    ap.add_argument('--dst-calib', required=True, help='correct yaml (e.g. VLP16_hires_db.yaml)')
    ap.add_argument('--topics', nargs='+', default=['/velodyne_points'])
    ap.add_argument('--sensor-frame', default='velodyne',
                    help='abort if a cloud is not in this frame (re-projection needs sensor-frame points)')
    ap.add_argument('--ring-field', default='ring')
    ap.add_argument('--include', nargs='*', default=None,
                    help='only copy these topics (default: all)')
    ap.add_argument('--latched', nargs='*', default=['/tf_static'])
    ap.add_argument('--start', type=float, default=0.0, help='seconds from bag start')
    ap.add_argument('--duration', type=float, default=0.0, help='0 = to the end')
    ap.add_argument('--passthrough', action='store_true',
                    help='copy clouds unchanged (for making an otherwise identical control segment)')
    ap.add_argument('--storage', default='sqlite3')
    a = ap.parse_args()

    tables = build_tables(a.src_calib, a.dst_calib)
    print('ring  src_v[deg] src_o[m] -> dst_v[deg] dst_o[m]', file=sys.stderr)
    for i in range(len(tables[0])):
        print(f'{i:4d} {math.degrees(tables[0][i]):9.3f} {tables[1][i]:8.4f} -> '
              f'{math.degrees(tables[2][i]):9.3f} {tables[3][i]:8.4f}', file=sys.stderr)

    if Path(a.output_bag).exists():
        sys.exit(f'{a.output_bag} exists; refusing to overwrite')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.input_bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = reader.get_metadata()
    t0 = meta.starting_time.nanoseconds
    t_lo = t0 + int(a.start * 1e9)
    t_hi = t_lo + int(a.duration * 1e9) if a.duration > 0 else None
    topics = reader.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}
    if a.include is not None:
        keep = set(a.include) | set(a.latched) | set(a.topics)
        topics = [t for t in topics if t.name in keep]
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t.name for t in topics]))

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.output_bag, storage_id=a.storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in topics:
        writer.create_topic(t)
    pc_type = get_message('sensor_msgs/msg/PointCloud2')
    stats, n_pc, n_other = {}, 0, 0
    while reader.has_next():
        name, data, t = reader.read_next()
        if name in a.latched:
            writer.write(name, data, max(t, t_lo))
            n_other += 1
            continue
        if t < t_lo:
            continue
        if t_hi is not None and t > t_hi:
            break
        if name in a.topics and not a.passthrough:
            msg = deserialize_message(data, pc_type)
            if msg.header.frame_id != a.sensor_frame:
                sys.exit(f'cloud frame_id={msg.header.frame_id!r} != {a.sensor_frame!r}; '
                         'points are not in the sensor frame, re-projection would be wrong')
            data = serialize_message(reproject(msg, tables, a.ring_field, stats))
            n_pc += 1
            if n_pc % 1000 == 0:
                print(f'{n_pc} clouds, max |z recompute err| = {stats["max_zchk"]:.2e} m', file=sys.stderr)
        else:
            n_other += 1
        writer.write(name, data, t)
    del writer
    print(f'done: {n_pc} clouds re-projected, {n_other} other msgs copied; '
          f'max |z recompute err| = {stats.get("max_zchk", 0):.2e} m', file=sys.stderr)


if __name__ == '__main__':
    main()

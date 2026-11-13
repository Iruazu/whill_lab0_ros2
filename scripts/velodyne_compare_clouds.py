#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Compare two bags' PointCloud2 topics point by point (#152).

Purpose: show that `velodyne_reproject_bag.py` gives the same points as the
driver would have produced with the correct calibration. The reference bag is
made by replaying /velodyne_packets into velodyne_transform_node with the
Hi-Res yaml (see velodyne_reproject_verify.sh); the candidate bag is the
re-projected /velodyne_points of the same recording.

Clouds are matched by header.stamp, not by order, because the recorder can
drop or reorder messages and a shifted pairing would look like a huge error.
The driver publishes organized clouds (NaN for no return), so points are
compared index by index and the NaN masks must be identical; a mask mismatch
would mean the two paths kept different returns, which the xyz error alone
would hide.

PASS also needs at least --min-matched (0.99) of the reference clouds to have a
candidate with the same stamp, so a recording that lost most clouds cannot pass
on the few that remain.

Memory: the reference bag is held in memory as float64 (about 32 bytes per
point incl. ring: ~2 GB for the 300 s segment, 62 M points); the candidate is
streamed one cloud at a time. Put the smaller bag first. Holding the whole 07-10
map bag (21311 clouds, ~450 M points) as the reference would need ~14 GB plus
the temporaries of the conversion (plan for ~20 GB).
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def iter_clouds(uri, topic):
    # A generator, so the candidate bag (the whole 13 GB map bag in #152) is
    # streamed cloud by cloud instead of held in memory (~14-20 GB as float64).
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    pc_type = get_message('sensor_msgs/msg/PointCloud2')
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, pc_type)
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        dt = np.dtype({'names': [f.name for f in msg.fields],
                       'formats': [('>' if msg.is_bigendian else '<') + DT[f.datatype]
                                   for f in msg.fields],
                       'offsets': [f.offset for f in msg.fields],
                       'itemsize': msg.point_step})
        arr = np.frombuffer(bytes(msg.data), dtype=dt)[: msg.width * msg.height]
        yield key, (np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64),
                    arr['ring'].astype(np.int64) if 'ring' in arr.dtype.names else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('reference_bag')
    ap.add_argument('candidate_bag')
    ap.add_argument('--ref-topic', default='/velodyne_points')
    ap.add_argument('--cand-topic', default='/velodyne_points')
    # float32 has a 24-bit mantissa: one rounding at 130 m (max_range) is up to
    # 0.008 mm, and the recorded input, the re-projection and the driver output
    # each round once, so a few hundredths of a mm is "identical" here. 0.05 mm
    # is still ~1000x below the Hi-Res vs VLP16db error (cm to dm).
    ap.add_argument('--tol-mm', type=float, default=0.05,
                    help='pass threshold on the max xyz difference [mm]')
    # A recorder that dropped most clouds would otherwise "pass" on a handful.
    ap.add_argument('--min-matched', type=float, default=0.99,
                    help='minimum fraction of reference clouds that must have a '
                         'candidate with the same stamp')
    a = ap.parse_args()

    # The reference is held in memory (it should be the smaller bag: the driver
    # output of a short packet bag, or a segment); the candidate is streamed.
    ref = dict(iter_clouds(a.reference_bag, a.ref_topic))
    max_err, n_pts, n_mask_bad, n_ring_bad, n_shape_bad = 0.0, 0, 0, 0, 0
    errs, n_cand, n_common = [], 0, 0
    for k, (q, rc) in iter_clouds(a.candidate_bag, a.cand_topic):
        n_cand += 1
        if k not in ref:
            continue
        n_common += 1
        p, rr = ref[k]
        if p.shape != q.shape:
            n_shape_bad += 1
            continue
        mp, mq = np.isfinite(p).all(1), np.isfinite(q).all(1)
        n_mask_bad += int((mp != mq).sum())
        m = mp & mq
        if rr is not None and rc is not None:
            n_ring_bad += int((rr[m] != rc[m]).sum())
        e = np.linalg.norm(p[m] - q[m], axis=1)
        n_pts += int(m.sum())
        if e.size:
            max_err = max(max_err, float(e.max()))
            errs.append(np.percentile(e, 99.9))
    if n_common == 0:
        sys.exit('no cloud with a common header.stamp; nothing compared')
    matched = n_common / len(ref)
    print(f'clouds: ref={len(ref)} cand={n_cand} common={n_common} (matched {matched:.3f})')
    print(f'points compared: {n_pts}')
    print(f'shape mismatches: {n_shape_bad}, NaN-mask mismatches: {n_mask_bad}, '
          f'ring mismatches: {n_ring_bad}')
    print(f'max |dxyz| = {max_err * 1000:.6f} mm; '
          f'worst per-cloud p99.9 = {max(errs) * 1000 if errs else 0:.6f} mm')
    ok = (max_err * 1000 <= a.tol_mm and n_mask_bad == 0 and n_shape_bad == 0
          and n_ring_bad == 0 and matched >= a.min_matched)
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

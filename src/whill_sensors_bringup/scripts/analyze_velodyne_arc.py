#!/usr/bin/env python3
"""Offline analyzer: derive velodyne_self_filter geometry from a rosbag2.

Replaces the RViz Measure-tool workflow described in
docs/session-2026-05-20.md. The chair sits still for a few seconds while
/velodyne_points is recorded; this script then digests the bag and prints
recommended self_radius / self_z_min / self_z_max so the cylinder swallows
the mount-frame phantom arc without eating the floor.

Why this works: with the chair static the mount and chair-body returns
land at a fixed (r, z) cluster in the velodyne frame regardless of
azimuth or scan. The cluster shows up as a sharp peak in the close-range
radial histogram, well separated from environmental returns which sit
beyond ~2 m. The same data also lets us report the kept-ratio the live
filter would log, so the parameters can be sanity-checked offline before
touching the launch file.

Usage:
    # 1. With the chair seated still and the velodyne driver publishing:
    ros2 bag record /velodyne_points -o /tmp/arc_probe -d 3

    # 2. Then run this analyzer:
    python3 src/whill_sensors_bringup/scripts/analyze_velodyne_arc.py \\
        /tmp/arc_probe
"""

from __future__ import annotations

import argparse
import sys
from typing import List

import numpy as np

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


VELODYNE_TOPIC = '/velodyne_points'


def load_pointclouds(bag_dir: str) -> List[np.ndarray]:
    """Read every /velodyne_points scan in `bag_dir` as an N×3 float32 array."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if VELODYNE_TOPIC not in type_map:
        raise SystemExit(
            f'bag {bag_dir!r} does not contain {VELODYNE_TOPIC} '
            f'(topics: {sorted(type_map)})')

    msg_type = get_message(type_map[VELODYNE_TOPIC])
    clouds: List[np.ndarray] = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != VELODYNE_TOPIC:
            continue
        msg = rclpy.serialization.deserialize_message(data, msg_type)
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            continue
        n = msg.width * msg.height
        if n == 0:
            continue
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

        def field(off: int) -> np.ndarray:
            chunk = np.ascontiguousarray(buf[:, off:off + 4])
            return chunk.view(np.float32).ravel()

        xyz = np.stack(
            [field(offsets['x']), field(offsets['y']), field(offsets['z'])],
            axis=1,
        )
        finite = np.all(np.isfinite(xyz), axis=1)
        clouds.append(xyz[finite])

    return clouds


def analyze(
    clouds: List[np.ndarray],
    close_range_m: float = 2.0,
    bin_m: float = 0.05,
    r_margin_m: float = 0.15,
    z_margin_m: float = 0.05,
) -> None:
    if not clouds:
        raise SystemExit('no /velodyne_points messages found in bag')

    all_pts = np.concatenate(clouds, axis=0)
    r_all = np.sqrt(all_pts[:, 0] ** 2 + all_pts[:, 1] ** 2)
    print(f'loaded {len(clouds)} scans, {all_pts.shape[0]:,} finite points')

    near_mask = r_all < close_range_m
    near_pts = all_pts[near_mask]
    near_r = r_all[near_mask]
    print(f'close-range (r < {close_range_m} m): {near_pts.shape[0]:,} points')
    if near_pts.shape[0] == 0:
        raise SystemExit(
            'no close-range points; chair may not be near anything '
            'or the velodyne origin may be misaligned')

    bins = np.arange(0.0, close_range_m + bin_m, bin_m)
    hist, edges = np.histogram(near_r, bins=bins)

    # Top-5 radial peaks — the dominant one is normally the mount/chair body,
    # but the user should sanity-check it against the runner-up bins.
    order = np.argsort(hist)[::-1]
    print()
    print('top-5 radial bins inside the close-range window:')
    for rank, idx in enumerate(order[:5], start=1):
        print(f'  {rank}. r ∈ [{edges[idx]:.2f}, {edges[idx+1]:.2f}] m  '
              f'count={hist[idx]:,}')

    peak_idx = int(order[0])
    peak_r_low = float(edges[peak_idx])
    peak_r_high = float(edges[peak_idx + 1])

    # Widen the band by ±2 bins to get a stable z distribution; a single
    # 5 cm bin can be sparse on the edges of the arc.
    band_lo = max(peak_idx - 2, 0)
    band_hi = min(peak_idx + 3, len(edges) - 1)
    band_mask = (near_r >= edges[band_lo]) & (near_r < edges[band_hi])
    band_pts = near_pts[band_mask]
    z_band = band_pts[:, 2]
    az_band = np.degrees(np.arctan2(band_pts[:, 1], band_pts[:, 0]))
    z_p5, z_p50, z_p95 = np.percentile(z_band, [5, 50, 95])
    print()
    print(f'z within radial band [{edges[band_lo]:.2f}, {edges[band_hi]:.2f}] m:')
    print(f'  p5 = {z_p5:+.3f}   p50 = {z_p50:+.3f}   p95 = {z_p95:+.3f}')

    # Azimuth coverage tells us whether the arc is forward-only (mount frame
    # in front of LiDAR) or wraps around (full self-return cylinder). If it
    # wraps, the cylinder filter is the right tool; if forward-only, a
    # per-ring or forward-sector filter would be more surgical.
    az_bins = np.arange(-180, 181, 30)
    az_hist, _ = np.histogram(az_band, bins=az_bins)
    print()
    print('azimuth distribution of points in that band (deg → count):')
    for lo, hi, c in zip(az_bins[:-1], az_bins[1:], az_hist):
        bar = '#' * min(40, int(40 * c / max(az_hist.max(), 1)))
        print(f'  [{lo:+4d}, {hi:+4d}) {c:6,}  {bar}')

    rec_radius = round(peak_r_high + r_margin_m, 2)
    rec_z_min = round(max(z_p5 - z_margin_m, -0.5), 2)
    rec_z_max = round(min(z_p95 + z_margin_m, 0.2), 2)

    # Simulate the filter on the loaded data to predict the live kept-ratio.
    z_all = all_pts[:, 2]
    in_cyl = (
        (r_all < rec_radius)
        & (z_all > rec_z_min)
        & (z_all < rec_z_max)
    )
    kept_ratio = 1.0 - in_cyl.sum() / all_pts.shape[0]

    print()
    print('=== recommended velodyne_self_filter params ===')
    print(f'  self_radius: {rec_radius}')
    print(f'  self_z_min:  {rec_z_min}')
    print(f'  self_z_max:  {rec_z_max}')
    print(f'  predicted live kept ratio: {100*kept_ratio:.1f}%  '
          f'(healthy band 80–95%, < 50% = cylinder too greedy)')
    print()
    print('apply via src/whill_sensors_bringup/launch/sensors_launch.py '
          '(velodyne_self_filter Node parameters), then rebuild:')
    print('  colcon build --packages-select whill_sensors_bringup --symlink-install')


def main() -> int:
    p = argparse.ArgumentParser(
        description='Offline analyzer for the WHILL velodyne self-filter geometry.')
    p.add_argument('bag_dir',
                   help='rosbag2 directory containing /velodyne_points '
                        '(produced by `ros2 bag record /velodyne_points -o <dir>`)')
    p.add_argument('--close-range', type=float, default=2.0,
                   help='max radial distance (m) to scan for self-return cluster '
                        '(default 2.0; environmental returns usually live beyond this)')
    p.add_argument('--bin', type=float, default=0.05,
                   help='radial histogram bin width in metres (default 0.05)')
    p.add_argument('--r-margin', type=float, default=0.15,
                   help='extra metres added to the observed peak when recommending '
                        'self_radius (default 0.15)')
    p.add_argument('--z-margin', type=float, default=0.05,
                   help='extra metres padded onto z p5/p95 when recommending '
                        'self_z_min/max (default 0.05)')
    args = p.parse_args()
    clouds = load_pointclouds(args.bag_dir)
    analyze(
        clouds,
        close_range_m=args.close_range,
        bin_m=args.bin,
        r_margin_m=args.r_margin,
        z_margin_m=args.z_margin,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())

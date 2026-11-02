#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Reproduce #147 handoff checks from existing NPZs; no ROS node or bag writes.

Run with PYTHONNOUSERSITE=1 to use Ubuntu's matching NumPy/SciPy packages.
The input directory must contain data/ and tools/imu_vs_lidar.py from the
2026-10-30 investigation. This audits cached ICP, not ground-truth motion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('investigation', type=Path)
    parser.add_argument('output', type=Path, help='new output directory')
    args = parser.parse_args()
    root = args.investigation.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    names = ['odom_orig_30_330', 'odom_hires_30_630',
             'odom_hires_noskew_30_150']
    inputs = [root / 'data' / (name + '.npz') for name in names]
    analysis = root / 'tools/imu_vs_lidar.py'
    inputs += [root / 'data/imu.npz', root / 'data/scans.npz', analysis]
    summary = {'input_sha256': {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        'python': sys.version, 'numpy': np.__version__}
    for name in ['imu', 'scans']:
        with np.load(root / 'data' / (name + '.npz')) as data:
            stamp = data['stamp']
            dt = np.diff(stamp)
            if not np.all(dt > 0):
                raise ValueError(f'{name}: timestamps must increase')
            stats = {
                'count': len(stamp), 'duration_s': float(stamp[-1] - stamp[0]),
                'rate_hz': float((len(stamp) - 1) / (stamp[-1] - stamp[0])),
                'dt_min_median_p99_max_s': np.quantile(dt, [0, .5, .99, 1]).tolist(),
                'receive_delay_min_median_p99_max_s': np.quantile(
                    data['recv'] - stamp, [0, .5, .99, 1]).tolist(),
            }
            if name == 'imu':
                mask = (stamp >= stamp[0] + 2) & (stamp < stamp[0] + 27)
                stats['window_2_27s_count'] = int(mask.sum())
                stats['window_2_27s_acc_mean'] = data['acc'][mask].mean(0).tolist()
                stats['window_2_27s_gyro_mean'] = data['gyro'][mask].mean(0).tolist()
                stats['window_2_27s_gyro_std'] = data['gyro'][mask].std(0).tolist()
            else:
                for key in ['tmin', 'tmax', 'n']:
                    stats[key + '_min_median_max'] = np.quantile(
                        data[key], [0, .5, 1]).tolist()
            summary[name] = stats
    odoms = []
    for name in names:
        with np.load(root / 'data' / (name + '.npz')) as data:
            odoms.append({key: data[key] for key in data.files})
    start = max(data['stamp'][0] for data in odoms)
    end = min(data['stamp'][-1] for data in odoms)
    if start >= end:
        raise ValueError('No common interval')
    summary['common_interval_epoch_s'] = [float(start), float(end)]
    reference_stamps = None
    for name, data in zip(names, odoms):
        mask = (data['stamp'] >= start) & (data['stamp'] <= end)
        stamps = data['stamp'][mask]
        if reference_stamps is not None and not np.array_equal(stamps, reference_stamps):
            raise ValueError('Common interval does not contain identical scan stamps')
        reference_stamps = stamps
        sliced = {key: value[mask] if value.ndim and value.shape[0] == len(mask)
                  else value for key, value in data.items()}
        path = out / (name + '_common.npz')
        np.savez(path, **sliced)
        command = [sys.executable, str(analysis), str(root / 'data/imu.npz'),
                   str(path), '--plot-dir', str(out), '--tag', name + '_common']
        with (out / (name + '_common.txt')).open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    summary['common_scans'] = len(reference_stamps)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(out / 'summary.json')


if __name__ == '__main__':
    main()

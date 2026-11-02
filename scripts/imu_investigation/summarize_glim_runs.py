#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""#147: summarise GLIM runs: final IMU validation, estimated bias, trajectory shape.

The better ratios are cumulative (imu_validation.cpp), so the last printed block
is the run's result. GLIM prints it only while the check fails, so a run that
stops warning reports its last failing block; the warning count shows that.
"""
import glob
import re
import sys
from pathlib import Path

import numpy as np


def summarize(run):
    run = Path(run)
    log = (run / 'glim.log').read_text(errors='replace')
    ratio = re.findall(r'better ratios rot=([\d.]+), trans=([\d.]+), vel=([\d.]+)', log)
    n = re.findall(r'num_validations=(\d+)', log)
    noimu = re.findall(r'No-IMU errors rot=([\d.]+).*?trans=([\d.]+).*?vel=([\d.]+)', log)
    imu = re.findall(r'\] IMU errors rot=([\d.]+).*?trans=([\d.]+).*?vel=([\d.]+)', log)
    bias = []
    for d in sorted(glob.glob(str(run / 'glim' / ('[0-9]' * 6)))):
        m = re.search(r'imu_bias:\s*([^\n]+)', Path(d, 'data.txt').read_text())
        if m:
            bias.append(list(map(float, m.group(1).split())))
    bias = np.array(bias)
    traj = np.loadtxt(run / 'glim' / 'traj_lidar.txt')
    path = np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1).sum()
    return {
        'rc': (run / 'exit-code.txt').read_text().strip(),
        'warn': log.count('IMU prediction is not good'),
        'n': n[-1] if n else '-',
        'ratio': ratio[-1] if ratio else ('-',) * 3,
        'noimu_vel': noimu[-1][2] if noimu else '-',
        'imu_vel': imu[-1][2] if imu else '-',
        'submaps': len(bias),
        'acc_max': float(np.linalg.norm(bias[:, :3], axis=1).max()),
        'acc_end': np.round(bias[-1, :3], 3).tolist(),
        'gyro_end': np.round(bias[-1, 3:], 4).tolist(),
        'path': float(path),
        'z_span': float(np.ptp(traj[:, 3])),
    }


def main():
    for run in sys.argv[1:]:
        s = summarize(run)
        r = s['ratio']
        print(f"{Path(run).name:22s} rc={s['rc']} warn={s['warn']:3d} n={s['n']:>5} "
              f"rot={r[0]} trans={r[1]} vel={r[2]} vel_err imu/cv={s['imu_vel']}/{s['noimu_vel']} "
              f"sub={s['submaps']:3d} |acc|max={s['acc_max']:.3f} acc={s['acc_end']} "
              f"gyro={s['gyro_end']} path={s['path']:.1f} zspan={s['z_span']:.2f}")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compute trajectory-based loop-closure error for M5R-3 (Issue #48).

Used by scripts/m5r3_run_glim.sh and scripts/m5r3_run_fastlio_sam.sh as a
quick, automatable companion to the formal B1 acceptance criterion. B1
itself (docs/ja/plans/2026-06-21-m5r-execution.md §6) is the mean of
three points picked on the same physical wall at start and end of the
loop, measured in CloudCompare. That requires human interaction and the
generated PCD, so it cannot be scripted here.

What this script measures instead is the gap between the FIRST and LAST
sample of the trajectory file emitted by the SLAM. For a loop that
returned to its starting point, this is the SLAM's own self-reported
mismatch between "where I think I am at the end" and "where I think I
started" — orthogonal to (but correlated with) the physical wall
distance from the bag's start and end frames.

The two metrics fail in different ways:

- Wall 3-point: catches systematic drift if the SLAM is internally
  consistent but its frame is rotated/translated vs. the world.
- Trajectory end-to-start: catches accumulated drift inside the SLAM's
  own pose graph — if the SLAM never closed the loop internally, this
  number is large even when the global frame happens to look OK.

Both rows go into ADR-0003's Alternatives table; one is not a
substitute for the other.

Input format: TUM trajectory, i.e. one pose per line as

    timestamp tx ty tz qx qy qz qw

with whitespace separation. This is the upstream format for both GLIM
(traj_lidar.txt) and FAST-LIO SAM (traj.txt at the time of writing).
Lines starting with '#' are treated as comments and skipped.

Usage:

    # Human-readable summary:
    scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt

    # Machine-readable JSON (for transcribing into the ADR table):
    scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt --json
"""

import argparse
import json
import math
import sys
from pathlib import Path


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from a quaternion (ZYX convention).

    Borrowed verbatim from scripts/m4r3_ekf_bench.py — the M5R-3 SLAMs
    are not constrained to 2D, but for an indoor loop on a wheelchair
    the meaningful drift dimension is yaw and the start-vs-end yaw delta
    is a useful sanity check on top of the Euclidean distance.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_tum_line(line: str) -> tuple[float, float, float, float, float, float, float, float] | None:
    """Parse one TUM-format trajectory line, or return None for comments.

    Returns (t, tx, ty, tz, qx, qy, qz, qw). Lines that fail to parse as
    8 floats are reported on stderr and skipped; this lets a partly
    truncated trajectory (e.g. SLAM crashed mid-write) still yield a
    useful start/end distance from whatever did make it to disk.
    """
    stripped = line.strip()
    if not stripped or stripped.startswith('#'):
        return None
    parts = stripped.split()
    if len(parts) != 8:
        print(f'WARNING: skipping malformed line ({len(parts)} fields, expected 8): '
              f'{stripped!r}', file=sys.stderr)
        return None
    try:
        t, tx, ty, tz, qx, qy, qz, qw = (float(x) for x in parts)
    except ValueError as exc:
        print(f'WARNING: skipping unparseable line ({exc}): {stripped!r}',
              file=sys.stderr)
        return None
    return (t, tx, ty, tz, qx, qy, qz, qw)


def load_trajectory(
    path: Path,
) -> list[tuple[float, float, float, float, float, float, float, float]]:
    samples = []
    with path.open() as f:
        for line in f:
            parsed = parse_tum_line(line)
            if parsed is not None:
                samples.append(parsed)
    return samples


def compute_metrics(samples: list[tuple[float, ...]]) -> dict:
    """Compute the start/end metrics packed into the ADR-0003 table.

    All distances are in metres (matching the TUM convention and the
    M5R-3 B1 threshold of 0.5 m). Yaw is reported in both radians and
    degrees because the wrapped (-pi, pi] form is unintuitive at a
    glance and ADR readers will compare against the degree number.
    """
    t_first, x_first, y_first, z_first, qx_f, qy_f, qz_f, qw_f = samples[0]
    t_last, x_last, y_last, z_last, qx_l, qy_l, qz_l, qw_l = samples[-1]

    yaw_first = quat_to_yaw(qx_f, qy_f, qz_f, qw_f)
    yaw_last = quat_to_yaw(qx_l, qy_l, qz_l, qw_l)

    # Loop length: sum of consecutive Euclidean steps. The reported
    # number is sensitive to TUM sample rate (GLIM emits per-keyframe so
    # the inter-sample distance can be ~0.5 m, FAST-LIO SAM tends to
    # emit denser), so loop length is reported but not used as a primary
    # comparison axis.
    loop_length = 0.0
    for (_, xa, ya, za, *_), (_, xb, yb, zb, *_) in zip(samples, samples[1:]):
        loop_length += math.sqrt((xb - xa) ** 2 + (yb - ya) ** 2 + (zb - za) ** 2)

    dx = x_last - x_first
    dy = y_last - y_first
    dz = z_last - z_first
    end_to_start = math.sqrt(dx * dx + dy * dy + dz * dz)

    yaw_drift = yaw_last - yaw_first
    while yaw_drift > math.pi:
        yaw_drift -= 2 * math.pi
    while yaw_drift <= -math.pi:
        yaw_drift += 2 * math.pi

    return {
        'samples': len(samples),
        'start': {
            't': t_first, 'x': x_first, 'y': y_first, 'z': z_first,
            'yaw_rad': yaw_first, 'yaw_deg': math.degrees(yaw_first),
        },
        'end': {
            't': t_last, 'x': x_last, 'y': y_last, 'z': z_last,
            'yaw_rad': yaw_last, 'yaw_deg': math.degrees(yaw_last),
        },
        'loop_length_m': loop_length,
        'end_to_start_m': end_to_start,
        'per_axis': {'dx': dx, 'dy': dy, 'dz': dz},
        'yaw_drift_rad': yaw_drift,
        'yaw_drift_deg': math.degrees(yaw_drift),
    }


def print_human(metrics: dict, path: Path) -> None:
    """Human-readable summary, tuned for cut-and-paste into the ADR.

    The trailing reminder about CloudCompare is intentional: the ADR's
    B1 row needs the wall 3-point number, not this script's output. We
    print it every time so a reviewer cannot miss it.
    """
    s = metrics['start']
    e = metrics['end']
    a = metrics['per_axis']
    print(f'Trajectory: {path}')
    print(f'Samples:    {metrics["samples"]}')
    print(f'Start:      (x={s["x"]:+.3f}, y={s["y"]:+.3f}, z={s["z"]:+.3f}, '
          f'yaw={s["yaw_deg"]:+.2f} deg)')
    print(f'End:        (x={e["x"]:+.3f}, y={e["y"]:+.3f}, z={e["z"]:+.3f}, '
          f'yaw={e["yaw_deg"]:+.2f} deg)')
    print(f'Loop length:           {metrics["loop_length_m"]:.3f} m '
          f'(sum of consecutive sample distances)')
    print(f'End-to-start distance: {metrics["end_to_start_m"]:.3f} m '
          f'(SLAM-internal loop mismatch; B1 補完指標)')
    print(f'Per-axis:              dx={a["dx"]:+.3f}, dy={a["dy"]:+.3f}, '
          f'dz={a["dz"]:+.3f}')
    print(f'Yaw drift:             {metrics["yaw_drift_deg"]:+.2f} deg '
          f'({metrics["yaw_drift_rad"]:+.4f} rad, wrapped)')
    print()
    print('Reminder: this is the SLAM-internal end-to-start distance, NOT')
    print('the formal B1 acceptance metric. B1 = mean of 3 wall points at')
    print('start vs end, measured in CloudCompare on the generated PCD.')
    print('See docs/ja/m5r3-comparison-protocol.md §"ループ誤差計測".')


def main() -> int:
    p = argparse.ArgumentParser(
        description='Compute trajectory end-to-start loop error for M5R-3.',
    )
    p.add_argument('traj', help='Path to TUM-format trajectory file.')
    p.add_argument('--json', action='store_true',
                   help='Emit JSON to stdout instead of the human summary.')
    args = p.parse_args()

    traj_path = Path(args.traj)
    if not traj_path.exists():
        print(f'ERROR: trajectory file not found: {traj_path}', file=sys.stderr)
        return 1

    samples = load_trajectory(traj_path)
    if len(samples) < 2:
        print(f'ERROR: need at least 2 samples to compute end-to-start; '
              f'got {len(samples)}', file=sys.stderr)
        return 1

    metrics = compute_metrics(samples)
    metrics['trajectory_path'] = str(traj_path)

    if args.json:
        json.dump(metrics, sys.stdout, indent=2)
        sys.stdout.write('\n')
    else:
        print_human(metrics, traj_path)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

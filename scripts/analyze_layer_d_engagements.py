#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Offline Layer D (failsafe forward gate) engagement analysis from a bag.

Replays the recorded /scan through the same zone test as failsafe_node
(whill_safety.failsafe_node.forward_zone_blocked) for both Layer D shapes
and reports, per engagement (rising edge of D:forward_blocked):

  - bag time, map pose (/pcl_pose interpolated), commanded mode
    (nav / teleop / idle from the recorded cmd_vel topics)
  - the triggering /scan points (base_link x, y, range) of the first scan
  - whether those points fall on an occupied cell of the static occupancy
    map (=> fixed structure such as walls, poles, parked objects mapped at
    survey time) or on free/unknown cells (=> something not in the map:
    people, vehicles, clutter, or a ghost return)
  - episode length (how long D stayed engaged) and how many /scan points
    the zone held over the episode

The gate is simulated on a 20 Hz tick, exactly like failsafe_node._tick
(D active while now - last_blocked < hysteresis), so the counts match a
node replay. Only Layer D is modelled; Layers A-C are irrelevant here.

Outputs (in --out): engagements_<shape>.csv, a map PNG with both shapes,
and summary.json. The bag is opened read-only.

Example:
  scripts/analyze_layer_d_engagements.py \\
      --bag ~/whill_lab0_ros2/docs/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26 \\
      --map ~/whill_lab0_ros2/docs/maps/campus/occupancy.yaml --out /tmp/ld
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys

import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

try:
    from whill_safety.failsafe_node import (
        FORWARD_CLEAR_HYSTERESIS_S, FORWARD_POINT_COUNT_MIN,
        FORWARD_SECTOR_MAX_M, FORWARD_SECTOR_MIN_M, LAYER_D_RECT_HALF_WIDTH_M,
        PUBLISH_HZ, forward_zone_blocked)
except ImportError:  # pragma: no cover - needs the workspace sourced
    sys.exit('source the workspace (whill_safety) first')

TOPICS = ['/scan', '/pcl_pose', '/cmd_vel_nav', '/cmd_vel_teleop',
          '/cmd_vel_safety']
HALF_ANGLE_RAD = math.radians(30.0)
# Modes: a cmd topic counts as "commanding" within twist_mux's 0.5 s timeout.
CMD_TIMEOUT_S = 0.5


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(path)) \
        else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    out = {t: [] for t in TOPICS}
    cls = {t: get_message(types[t]) for t in present}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        t = t_ns * 1e-9
        if topic == '/scan':
            out[topic].append((t, np.asarray(msg.ranges, dtype=np.float64),
                               msg.angle_min, msg.angle_increment))
        elif topic == '/pcl_pose':
            p = msg.pose.pose
            yaw = math.atan2(2 * (p.orientation.w * p.orientation.z
                                  + p.orientation.x * p.orientation.y),
                             1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
            out[topic].append((t, p.position.x, p.position.y, yaw))
        else:
            out[topic].append((t, msg.linear.x, msg.angular.z))
    return out, present


def zone_points(ranges, angle_min, inc, shape):
    """Same accumulated-angle convention as forward_zone_blocked."""
    ang = angle_min + inc * np.arange(len(ranges))
    ok = np.isfinite(ranges)
    r, a = ranges[ok], ang[ok]
    x, y = r * np.cos(a), r * np.sin(a)
    if shape == 'sector':
        m = (r >= FORWARD_SECTOR_MIN_M) & (r <= FORWARD_SECTOR_MAX_M) \
            & (np.abs(a) <= HALF_ANGLE_RAD)
    else:
        m = (x >= FORWARD_SECTOR_MIN_M) & (x <= FORWARD_SECTOR_MAX_M) \
            & (np.abs(y) <= LAYER_D_RECT_HALF_WIDTH_M)
    return x[m], y[m], r[m]


def simulate(scans, shape):
    """Return [(t_engage, t_release, [scan indices blocked in episode])]."""
    blocked_t, blocked_i = [], []
    for i, (t, rng, amin, inc) in enumerate(scans):
        # Use the node's own function for the decision so the count is the
        # node's; zone_points() only supplies the per-point detail.
        if forward_zone_blocked(rng.tolist(), amin, inc, shape,
                                FORWARD_SECTOR_MIN_M, FORWARD_SECTOR_MAX_M,
                                HALF_ANGLE_RAD, LAYER_D_RECT_HALF_WIDTH_M,
                                FORWARD_POINT_COUNT_MIN):
            blocked_t.append(t)
            blocked_i.append(i)
    if not scans:
        return []
    # 20 Hz tick, as failsafe_node._tick: D is active while
    # tick - last_blocked < hysteresis. Blocked scans that arrive between
    # ticks are attributed to the episode that the next tick sees.
    dt = 1.0 / PUBLISH_HZ
    t1 = scans[-1][0] + 1.0
    episodes, active, k, last, cur = [], False, 0, None, None
    n = 0
    t0 = scans[0][0]
    while True:
        tick = t0 + n * dt  # multiply, not accumulate: no float drift
        if tick > t1:
            break
        batch = []
        while k < len(blocked_t) and blocked_t[k] <= tick:
            last = blocked_t[k]
            batch.append(blocked_i[k])
            k += 1
        now_active = last is not None and tick - last < FORWARD_CLEAR_HYSTERESIS_S
        if now_active and not active:
            cur = [tick, None, []]
            episodes.append(cur)
        elif active and not now_active:
            cur[1] = tick
        if cur is not None and batch:
            # A batch consumed on a releasing tick still belongs to the
            # episode it extended (last within hysteresis of the scan).
            cur[2].extend(batch)
        active = now_active
        n += 1
    if cur is not None and cur[1] is None:
        cur[1] = t1
    return episodes


def interp_pose(poses, t):
    ts = [p[0] for p in poses]
    i = bisect.bisect_left(ts, t)
    if not poses:
        return (math.nan,) * 3
    if i <= 0:
        return poses[0][1:]
    if i >= len(poses):
        return poses[-1][1:]
    a, b = poses[i - 1], poses[i]
    w = (t - a[0]) / max(b[0] - a[0], 1e-9)
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * dyaw)


def mode_at(cmds, t):
    def recent(lst):
        ts = [c[0] for c in lst]
        i = bisect.bisect_right(ts, t) - 1
        return i >= 0 and t - ts[i] <= CMD_TIMEOUT_S, (lst[i] if i >= 0 else None)
    tele, tmsg = recent(cmds['/cmd_vel_teleop'])
    nav, nmsg = recent(cmds['/cmd_vel_nav'])
    if tele:
        return 'teleop', tmsg[1], tmsg[2]
    if nav:
        return 'nav', nmsg[1], nmsg[2]
    return 'idle', 0.0, 0.0


class OccMap:
    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        from PIL import Image
        img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.img = np.asarray(Image.open(img_path).convert('L'))
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = self.img.shape
        # map_server trinary: p = (255 - v)/255 (negate 0)
        p = (255.0 - self.img) / 255.0
        self.occ = p > float(meta['occupied_thresh'])
        self.free = p < float(meta['free_thresh'])

    def rc(self, x, y):
        c = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        r = self.h - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        return r, c

    def near_occupied(self, x, y, radius_m):
        r, c = self.rc(x, y)
        k = int(math.ceil(radius_m / self.res))
        out = []
        for ri, ci in zip(np.atleast_1d(r), np.atleast_1d(c)):
            r0, r1 = max(ri - k, 0), min(ri + k + 1, self.h)
            c0, c1 = max(ci - k, 0), min(ci + k + 1, self.w)
            out.append(bool(r0 < r1 and c0 < c1 and self.occ[r0:r1, c0:c1].any()))
        return np.asarray(out, dtype=bool)

    def cell_class(self, x, y):
        r, c = self.rc(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 'outside'
        if self.occ[r, c]:
            return 'occupied'
        return 'free' if self.free[r, c] else 'unknown'


def analyse(data, shape, occ, match_radius):
    scans = data['/scan']
    safety_ts = [s[0] for s in data['/cmd_vel_safety']]
    rows = []
    for n, (te, tr, idx) in enumerate(simulate(scans, shape), 1):
        t, rng, amin, inc = scans[idx[0]]
        x, y, r = zone_points(rng, amin, inc, shape)
        px, py, pyaw = interp_pose(data['/pcl_pose'], t)
        mx = px + x * math.cos(pyaw) - y * math.sin(pyaw)
        my = py + x * math.sin(pyaw) + y * math.cos(pyaw)
        on_map = occ.near_occupied(mx, my, match_radius) if occ else np.zeros(len(x), bool)
        # Episode-wide point statistics (all blocked scans in the episode).
        ep_pts = 0
        ep_inpath = 0
        for i in idx:
            ex, ey, _ = zone_points(scans[i][1], scans[i][2], scans[i][3], shape)
            ep_pts += len(ex)
            ep_inpath += int(np.sum(np.abs(ey) <= LAYER_D_RECT_HALF_WIDTH_M))
        mode, vx, wz = mode_at(data, t)
        # Did the live failsafe (recorded /cmd_vel_safety) fire around here?
        j = bisect.bisect_left(safety_ts, t - 0.5)
        safety_live = j < len(safety_ts) and safety_ts[j] <= t + 0.5
        imin = int(np.argmin(r)) if len(r) else None
        frac_map = float(on_map.mean()) if len(on_map) else math.nan
        if len(x) == 1 and len(idx) == 1:
            cause = 'single_point_flicker'
        elif frac_map >= 0.5:
            cause = 'mapped_structure'
        else:
            cause = 'unmapped_object'
        lateral = 'in_path' if (len(y) and np.min(np.abs(y)) <= LAYER_D_RECT_HALF_WIDTH_M) \
            else 'beside_path'
        rows.append({
            'n': n,
            't_rel_s': round(t - scans[0][0], 2),
            'duration_s': round(tr - te, 2),
            'blocked_scans': len(idx),
            'map_x': round(px, 2), 'map_y': round(py, 2),
            'yaw_deg': round(math.degrees(pyaw), 1),
            'mode': mode, 'cmd_vx': round(vx, 3), 'cmd_wz': round(wz, 3),
            'live_safety_msg_within_0.5s': safety_live,
            'n_points_first_scan': len(x),
            'min_range_m': round(float(r[imin]), 2) if imin is not None else '',
            'x_at_min_m': round(float(x[imin]), 2) if imin is not None else '',
            'y_at_min_m': round(float(y[imin]), 2) if imin is not None else '',
            'min_abs_y_m': round(float(np.min(np.abs(y))), 2) if len(y) else '',
            'max_abs_y_m': round(float(np.max(np.abs(y))), 2) if len(y) else '',
            'lateral': lateral,
            'episode_points': ep_pts,
            'episode_points_in_rect': ep_inpath,
            'frac_on_mapped_occupied': round(frac_map, 2),
            'cell_at_min': occ.cell_class(mx[imin], my[imin]) if (occ and imin is not None) else '',
            'obj_map_x': round(float(mx[imin]), 2) if imin is not None else '',
            'obj_map_y': round(float(my[imin]), 2) if imin is not None else '',
            'cause': cause,
        })
    return rows


def _draw(ax, occ, poses, results, box, annotate):
    x0, x1, y0, y1 = box
    if occ is not None:
        r0, c0 = occ.rc(x0, y1)
        r1, c1 = occ.rc(x1, y0)
        r0, c0 = max(int(r0), 0), max(int(c0), 0)
        sub = occ.img[r0:int(r1) + 1, c0:int(c1) + 1]
        ext = (occ.ox + c0 * occ.res, occ.ox + (c0 + sub.shape[1]) * occ.res,
               occ.oy + (occ.h - (r0 + sub.shape[0])) * occ.res,
               occ.oy + (occ.h - r0) * occ.res)
        ax.imshow(sub, cmap='gray', vmin=0, vmax=255, extent=ext,
                  interpolation='nearest')
    ax.plot(poses[:, 0], poses[:, 1], '-', color='#3b82f6', lw=1.0,
            label='/pcl_pose trajectory')
    style = {'sector': dict(marker='o', s=40, facecolors='none',
                            edgecolors='#f97316', linewidths=1.4),
             'rectangle': dict(marker='x', s=70, c='#dc2626', linewidths=2.0)}
    for shape, rows in results.items():
        if not rows:
            continue
        ax.scatter([r['map_x'] for r in rows], [r['map_y'] for r in rows],
                   label=f'{shape}: chair pose ({len(rows)})', zorder=5,
                   **style[shape])
        ox = [r['obj_map_x'] for r in rows if r['obj_map_x'] != '']
        oy = [r['obj_map_y'] for r in rows if r['obj_map_y'] != '']
        ax.scatter(ox, oy, marker='^' if shape == 'sector' else 'v', s=22,
                   c='#a855f7' if shape == 'sector' else '#16a34a', zorder=6,
                   label=f'{shape}: nearest trigger point')
    if annotate:
        for r in results.get('rectangle', []):
            ax.annotate(f"R{r['n']}", (r['map_x'], r['map_y']), xytext=(6, -12),
                        textcoords='offset points', color='#dc2626', fontsize=8)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')


def plot(occ, data, results, out_png, title, pad=8.0, zoom_pad=3.0):
    """Overview of the whole run plus a zoom on the engagement area."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    poses = np.array([p[1:3] for p in data['/pcl_pose']])
    rows = [r for rr in results.values() for r in rr]
    ov = (poses[:, 0].min() - pad, poses[:, 0].max() + pad,
          poses[:, 1].min() - pad, poses[:, 1].max() + pad)
    fig, axes = plt.subplots(2, 1, figsize=(10, 11), dpi=100,
                             gridspec_kw={'height_ratios': [1, 1.6]})
    _draw(axes[0], occ, poses, results, ov, annotate=False)
    axes[0].set_title(title, fontsize=10)
    if rows:
        xs = [r['map_x'] for r in rows] + [r['obj_map_x'] for r in rows if r['obj_map_x'] != '']
        ys = [r['map_y'] for r in rows] + [r['obj_map_y'] for r in rows if r['obj_map_y'] != '']
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + zoom_pad
        _draw(axes[1], occ, poses, results,
              (cx - half, cx + half, cy - half * 0.6, cy + half * 0.6),
              annotate=True)
        axes[1].set_title('zoom on engagements (map: occupancy.pgm, '
                          'black = occupied)', fontsize=9)
        axes[1].legend(loc='best', fontsize=8)
    else:
        axes[1].axis('off')
        axes[1].text(0.5, 0.5, 'no Layer D engagement in either shape',
                     ha='center', va='center')
        axes[0].legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--map', help='occupancy yaml (map_server format)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', default=None, help='prefix for output files')
    ap.add_argument('--match-radius', type=float, default=0.25,
                    help='a zone point within this distance of an occupied '
                         'map cell counts as mapped structure (pose error '
                         'of the NDT localizer is ~0.1-0.2 m)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tag = a.tag or os.path.basename(os.path.normpath(a.bag))
    data, present = read_bag(a.bag)
    missing = [t for t in ('/scan', '/pcl_pose') if t not in present]
    if missing:
        sys.exit(f'{a.bag}: missing {missing}')
    occ = OccMap(a.map) if a.map else None
    results = {s: analyse(data, s, occ, a.match_radius)
               for s in ('sector', 'rectangle')}
    fields = list(next((r for rows in results.values() for r in rows), {}).keys())
    for shape, rows in results.items():
        with open(os.path.join(a.out, f'{tag}_engagements_{shape}.csv'), 'w',
                  newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields or ['n'])
            w.writeheader()
            w.writerows(rows)
    dur = data['/scan'][-1][0] - data['/scan'][0][0]
    summary = {'bag': a.bag, 'duration_s': round(dur, 1),
               'scans': len(data['/scan']),
               'live_cmd_vel_safety_msgs': len(data['/cmd_vel_safety'])}
    for shape, rows in results.items():
        def cnt(key):
            d = {}
            for r in rows:
                d[r[key]] = d.get(r[key], 0) + 1
            return d
        summary[shape] = {
            'engagements': len(rows),
            'engaged_time_s': round(sum(r['duration_s'] for r in rows), 1),
            'by_cause': cnt('cause'), 'by_lateral': cnt('lateral'),
            'by_mode': cnt('mode'),
        }
    with open(os.path.join(a.out, f'{tag}_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    plot(occ, data, results, os.path.join(a.out, f'{tag}_map.png'),
         f'{tag}: Layer D engagements, sector {len(results["sector"])} vs '
         f'rectangle {len(results["rectangle"])}')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()

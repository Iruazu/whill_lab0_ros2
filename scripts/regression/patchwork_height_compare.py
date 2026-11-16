#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Patchwork++ の sensor_height を変えたときの地面除去の差を bag の再生で比べる（#192）。

使い方（worktree なら先に src/third_party をリンクし、whill_perception をビルドしておく）:

  cd ~/whill_lab0_ros2
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/regression/patchwork_height_compare.py \\
      --bag ~/bags/2026-11-16-static/empty_start_140915 --heights 0.79 0.771

何をするか:
  - 比べる高さの数だけ patchworkpp_node を同時に起動し、同じ /velodyne_points を
    同時に流す。同じフレームを同じ順で受けるので、差は sensor_height だけから来る。
    1 本ずつ流すと DDS の取りこぼしがノードごとに違い、比較にならない。
  - 床の基準は入力点群だけから作る（1〜8 m の低い点に RANSAC で平面を当てる）。
    Patchwork++ の出力を基準にすると、比べる対象で比べる対象を測ることになる。
  - 指標（Patchwork++ が判定する 1 m < r <= 25 m の範囲だけで数える。範囲外の点は
    コアが無条件に non-ground に入れるので、高さの比較には意味が無い）:
      ground / no_ground の点数、
      床の漏れ = 1〜8 m で床から 5 cm 以内の点のうち no_ground に残った割合、
      物の保持 = 1〜8 m で床から 0.20〜2.0 m の点のうち no_ground に残った割合。
  - Patchwork++ は sensor_height を毎フレーム床の推定で更新する（コアの
    update_elevation_thr）。設定値は初期値なので、先頭の数フレームと残りを分けて出す。

安全:
  - 専用の domain（既定 192）・localhost・CycloneDDS に閉じ、開始時に他のノードが
    いないことを確かめる。再生するのは /velodyne_points だけで、指令の topic は流さない。
  - 何度でも実行できる。出力は --out の JSON（既定は標準出力の表のみ）。
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

# rclpy を読む前に環境を決める。子プロセス（ノード・ros2 bag play）にも同じ値が渡る。
DEFAULT_DOMAIN = '192'

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
DEFAULT_PARAMS = REPO / 'src' / 'whill_perception' / 'config' / 'patchworkpp.yaml'

# Patchwork++ の判定範囲（patchworkpp.yaml の min_range / max_range と同じ）。
R_MIN, R_MAX = 1.0, 25.0
# 床の平面を当てる範囲。遠くは路面の勾配・縁石で 1 枚の平面から外れるので 8 m まで。
NEAR_MAX = 8.0
FLOOR_BAND = 0.05      # 床から 5 cm 以内を「床の点」とする（VLP-16 の測距ノイズ約 3 cm の外側）
OBJ_BAND = (0.20, 2.0)  # 物の点。0.20 m は床の凹凸・縁石の上で、コーン（0.725 m）は入る
# 先頭のフレーム数。コアが sensor_height を自分で更新し終わるまでを分けて見る。
WARMUP_FRAMES = 10


def setup_env(domain):
    os.environ['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    # 運用の DDS XML（NIC 指定）を持ち込まない。localhost だけで足りる。
    os.environ.pop('CYCLONEDDS_URI', None)


def spawn(cmd, log):
    # 自前のプロセスグループにする。ros2 run はシグナルを子へ渡さないことがある。
    return subprocess.Popen(cmd, stdout=open(log, 'w'), stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop(proc):
    if proc.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(wait_s)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def fit_floor(xyz, rng):
    """入力点群の 1〜8 m の低い点に平面を当て、全点の床からの高さを返す。当たらなければ None。"""
    import numpy as np
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    # LiDAR は床から約 0.77 m・ほぼ水平。±0.35 m の帯で、着座の沈み・路面の勾配も入る。
    cand = xyz[(r > R_MIN) & (r <= NEAR_MAX) & (xyz[:, 2] > -1.12) & (xyz[:, 2] < -0.42)]
    if len(cand) < 200:
        return None
    best = None
    for _ in range(100):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            continue
        n /= norm
        # 床はほぼ水平。壁・物の面を拾わないよう法線が鉛直から 15° 以内に限る。
        if abs(n[2]) < 0.966:
            continue
        inl = np.abs((cand - p[0]) @ n) < 0.03
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 200:
        return None
    pts = cand[best]
    c = pts.mean(axis=0)
    n = np.linalg.svd(pts - c)[2][2]
    if n[2] < 0:
        n = -n
    return (xyz - c) @ n


def keys_xy(xyz):
    """点を x・y の float32 のビット列で 1 つの 64bit 整数にする。

    ノードは入力の float をそのまま写すので、no_ground の点は入力の点とビット単位で一致する。
    """
    import numpy as np
    b = np.ascontiguousarray(xyz[:, :2], dtype=np.float32).view(np.uint32)
    return (b[:, 0].astype(np.uint64) << np.uint64(32)) | b[:, 1].astype(np.uint64)


def frame_metrics(inp, ng_xyz):
    import numpy as np
    xyz, h = inp
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    in_rng = (r > R_MIN) & (r <= R_MAX)
    is_ng = np.isin(keys_xy(xyz), keys_xy(ng_xyz))
    m = {
        'in': int(len(xyz)),
        'no_ground_all': int(len(ng_xyz)),
        'in_range': int(in_rng.sum()),
        'no_ground': int((is_ng & in_rng).sum()),
    }
    m['ground'] = m['in_range'] - m['no_ground']
    if h is not None:
        near = (r > R_MIN) & (r <= NEAR_MAX)
        floor = near & (np.abs(h) < FLOOR_BAND)
        obj = near & (h >= OBJ_BAND[0]) & (h <= OBJ_BAND[1])
        m['floor'] = int(floor.sum())
        m['floor_leak'] = int((floor & is_ng).sum())
        m['obj'] = int(obj.sum())
        m['obj_kept'] = int((obj & is_ng).sum())
    return m


def summarize(frames):
    import numpy as np
    out = {'frames': len(frames)}
    if not frames:
        return out
    for k in ('in_range', 'ground', 'no_ground'):
        out[k + '_mean'] = float(np.mean([f[k] for f in frames]))
    fl = [f for f in frames if 'floor' in f]
    if fl:
        tot_floor = sum(f['floor'] for f in fl)
        tot_obj = sum(f['obj'] for f in fl)
        out['floor_leak_pct'] = 100.0 * sum(f['floor_leak'] for f in fl) / max(tot_floor, 1)
        out['obj_kept_pct'] = 100.0 * sum(f['obj_kept'] for f in fl) / max(tot_obj, 1)
        out['floor_pts_mean'] = tot_floor / len(fl)
        out['obj_pts_mean'] = tot_obj / len(fl)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--heights', type=float, nargs='+', default=[0.79, 0.771])
    ap.add_argument('--params', default=str(DEFAULT_PARAMS))
    ap.add_argument('--domain', default=DEFAULT_DOMAIN)
    ap.add_argument('--start-offset', type=float, default=0.0, help='bag の先頭からの秒')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='再生する bag の秒数（0 は最後まで）')
    ap.add_argument('--rate', type=float, default=0.5,
                    help='再生速度。照合の numpy 処理が追いつくよう既定は 0.5')
    ap.add_argument('--log-dir', default='/tmp/patchwork_height_compare')
    ap.add_argument('--out', help='結果の JSON を書く先')
    args = ap.parse_args()

    setup_env(args.domain)
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    bag = str(Path(args.bag).expanduser())
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node('patchwork_height_compare')

    # 開始時に domain が空であることを確かめる。他のノードの点群が混ざると比較が壊れる。
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    others = [n for n, _ in node.get_node_names_and_namespaces()
              if n != 'patchwork_height_compare']
    if others:
        print(f'ERROR: domain {args.domain} に他のノードがいる: {others}', file=sys.stderr)
        sys.exit(2)

    def to_xyz(msg):
        # VLP-16 の点は x,y,z,intensity（float32）と ring（uint16）・time が混ざるので
        # read_points_numpy（全 field 同じ型が前提）は使えない。x・y・z の offset から読む。
        off = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
        cols = [buf[:, off[c]:off[c] + 4].copy().view(np.float32).reshape(-1)
                for c in ('x', 'y', 'z')]
        return np.stack(cols, axis=1)

    rng = np.random.default_rng(0)  # 平面の RANSAC を再現可能にする
    pending = {}   # stamp -> (xyz, heights)。全ノードの出力が揃うまで持つ
    results = {i: {} for i in range(len(args.heights))}
    order = []     # 入力の到着順の stamp

    def stamp_of(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def on_input(msg):
        st = stamp_of(msg)
        xyz = to_xyz(msg)
        pending[st] = (xyz, fit_floor(xyz, rng))
        order.append(st)
        # 古い入力は捨てる（出力が来なかったフレーム）。メモリを有限にする。
        while len(pending) > 50:
            pending.pop(next(iter(pending)))

    def make_on_output(i):
        def cb(msg):
            st = stamp_of(msg)
            inp = pending.get(st)
            if inp is None:
                return
            results[i][st] = frame_metrics(inp, to_xyz(msg))
        return cb

    node.create_subscription(PointCloud2, '/velodyne_points', on_input, qos_profile_sensor_data)
    procs = []
    player = None
    for i, h in enumerate(args.heights):
        node.create_subscription(PointCloud2, f'/pw_cmp/ng_{i}', make_on_output(i),
                                 qos_profile_sensor_data)
        procs.append(spawn(['ros2', 'run', 'whill_perception', 'patchworkpp_node',
                            '--ros-args', '--params-file', args.params,
                            '-p', f'sensor_height:={h}',
                            '-r', f'__node:=pw_cmp_{i}',
                            '-r', 'cloud_in:=/velodyne_points',
                            '-r', f'cloud_no_ground:=/pw_cmp/ng_{i}'],
                           log_dir / f'node_{i}.log'))
    try:
        # 全ノードが購読を始めるまで待つ。先に再生すると先頭フレームがノードごとに欠ける。
        deadline = time.time() + 30.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            pubs = [node.count_publishers(f'/pw_cmp/ng_{i}') for i in range(len(procs))]
            if all(p >= 1 for p in pubs):
                break
        else:
            print('ERROR: patchworkpp_node が起動しない。ログ: ' + str(log_dir), file=sys.stderr)
            sys.exit(3)
        t_wait = time.time() + 2.0
        while time.time() < t_wait:
            rclpy.spin_once(node, timeout_sec=0.1)

        play = ['ros2', 'bag', 'play', bag, '--topics', '/velodyne_points',
                '-r', str(args.rate)]
        if args.start_offset > 0:
            play += ['--start-offset', str(args.start_offset)]
        player = spawn(play, log_dir / 'play.log')
        # Humble の ros2 bag play には再生時間の指定が無いので、壁時計で止める。
        limit = args.duration / args.rate if args.duration > 0 else None
        t0 = time.time()
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if limit is not None and time.time() - t0 > limit:
                stop(player)
                break
        t_tail = time.time() + 2.0
        while time.time() < t_tail:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        # 途中で例外が出ても再生とノードを残さない。残ると次の実行の「domain が空」の確認で止まる。
        if player is not None:
            stop(player)
        for p in procs:
            stop(p)

    # 全ノードが処理したフレームだけで比べる。到着順で先頭の WARMUP_FRAMES を分ける。
    common = [st for st in order if all(st in results[i] for i in results)]
    summary = {
        'bag': bag, 'start_offset': args.start_offset, 'duration': args.duration,
        'rate': args.rate, 'input_frames': len(order), 'common_frames': len(common),
        'heights': {},
    }
    for i, h in enumerate(args.heights):
        frames = [results[i][st] for st in common]
        summary['heights'][f'{h:g}'] = {
            'all': summarize(frames),
            'warmup': summarize(frames[:WARMUP_FRAMES]),
            'steady': summarize(frames[WARMUP_FRAMES:]),
        }
    # 同じフレームでの no_ground 点数の差（高さどうしの差の大きさを直接見る）
    if len(args.heights) >= 2 and common:
        a = np.array([results[0][st]['no_ground'] for st in common])
        b = np.array([results[1][st]['no_ground'] for st in common])
        d = b - a
        summary['no_ground_diff_1_minus_0'] = {
            'mean': float(d.mean()), 'abs_max': int(np.abs(d).max()),
            'frames_identical': int((d == 0).sum()),
            'steady_mean': float(d[WARMUP_FRAMES:].mean()) if len(d) > WARMUP_FRAMES else None,
        }

    print(f'bag: {bag}')
    print(f'入力 {len(order)} フレーム、全ノード共通 {len(common)} フレーム')
    hdr = f'{"h":>6} {"区間":<7} {"frames":>6} {"ground":>8} {"no_gnd":>8} {"床漏れ%":>8} {"物保持%":>8}'
    print(hdr)
    for h, s in summary['heights'].items():
        for part in ('warmup', 'steady', 'all'):
            v = s[part]
            if not v['frames']:
                continue
            print(f'{h:>6} {part:<7} {v["frames"]:>6} {v["ground_mean"]:>8.0f} '
                  f'{v["no_ground_mean"]:>8.0f} {v.get("floor_leak_pct", float("nan")):>8.3f} '
                  f'{v.get("obj_kept_pct", float("nan")):>8.2f}')
    if 'no_ground_diff_1_minus_0' in summary:
        print('no_ground の差（2 番目 − 1 番目）:', summary['no_ground_diff_1_minus_0'])
    if args.out:
        Path(args.out).write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    if not common:
        print('ERROR: 共通のフレームが無い', file=sys.stderr)
        sys.exit(4)


if __name__ == '__main__':
    main()

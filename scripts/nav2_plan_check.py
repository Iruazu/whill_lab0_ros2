#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""nav2_plan_check.py — 配車地点の全組で、Nav2 の global planner を地図だけで動かして経路を比べる（#154）。

map_route_check.py は「FREE を車体の幅だけ削っても 4 地点が 1 つの成分にある」までしか見ない。
inflation のコストで経路が縁から離れるか、縁ぎりぎりを通るかは、Nav2 の planner を動かさないと
分からない（README 154w 節の「確かめていないこと」）。このスクリプトは、地図ごとに map_server と
planner_server（global_costmap を内に持つ）だけを起動し、waypoints.yaml の全ての順序つきの組で
ComputePathToPose（use_start=true）を呼ぶ。最初の --map を基準にして、後の地図の経路がどれだけ
動いたかも出す。

使い方（ROS 2 Humble を source したシェルで。ロボットは要らない）:
    source /opt/ros/humble/setup.bash
    python3 scripts/nav2_plan_check.py \\
        --map docs/maps/campus-m8/occupancy_candidate_hand.yaml \\
        --map docs/maps/campus-m8/occupancy_candidate.yaml \\
        --waypoints docs/maps/campus-m8/waypoints.yaml \\
        --out-dir /tmp/nav2_plan_check

前提と安全:
  - 起動するのは map_server・planner_server・lifecycle_manager・static_transform_publisher だけ。
    controller・velocity_smoother・twist_mux は起動しないので、cmd_vel は誰も出さない。
  - 子プロセスは --domain（既定 87）の ROS_DOMAIN_ID で動かす。実機のスタックは 0 で動くので、
    0 を渡すと止める。同じ domain で /map を出すと、走行中の map_server の地図と混ざるため。
    起動する前に、その domain に map_server・planner_server などが居ないことも確かめ、居れば止める。
  - RMW は rmw_cyclonedds_cpp、CYCLONEDDS_URI は runbook と同じ configs/cyclonedds-runtime-min.xml。
  - planner と global_costmap のパラメータは src/whill_navigation/config/nav2_params.yaml をそのまま読む
    （static_layer + inflation_layer、allow_unknown: false、NavFn）。global_costmap には obstacle_layer が
    無いので、センサの入力は要らない。map → base_link は static TF の恒等で与える（costmap の起動に要る
    だけで、経路の始点は ComputePathToPose の start で渡す）。
  - 計画の前に、届いた global costmap が読ませた pgm と一致すること（OCC は全て LETHAL、FREE に
    NO_INFORMATION が無い）を確かめる。一致しないまま --startup-timeout を過ぎたら止める。
  - 終わると（失敗・Ctrl-C・SIGTERM・SIGHUP でも）起動した子プロセスを全部止める。

出力（--out-dir）: plans.json（地図ごと・組ごとの成否、長さ、OCC までの最短距離、経路上の最大コスト、
基準からのずれ、経路の点列）と、経路を重ねた原寸の切り出し plans_crop.png（基準 = 青、後の地図 = 赤。
成功した組が 1 つも無ければ書かない）。表は標準出力にも出す。1 つでも計画に失敗すると終了コード 1。
"""
import argparse
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree
import yaml

# 原寸の pgm は PIL の decompression-bomb の上限を超える。手元の地図に限って外す。
Image.MAX_IMAGE_PIXELS = None

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_PARAMS = REPO / 'src/whill_navigation/config/nav2_params.yaml'
DEFAULT_DDS_XML = REPO / 'configs/cyclonedds-runtime-min.xml'
# 同じ domain に既にあると、/map・/compute_path_to_pose が混ざって結果の出所が分からなくなるノード。
CONFLICTING_NODES = ('map_server', 'planner_server', 'global_costmap', 'lifecycle_manager_plan_check')
LETHAL, NO_INFORMATION = 254, 255


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--map', action='append', required=True, type=pathlib.Path,
                   help='占有格子の yaml。複数可。最初の物を基準にずれを測る。')
    p.add_argument('--waypoints', type=pathlib.Path, required=True)
    p.add_argument('--params', type=pathlib.Path, default=DEFAULT_PARAMS)
    p.add_argument('--out-dir', type=pathlib.Path, required=True)
    p.add_argument('--domain', type=int, default=87, help='子プロセスの ROS_DOMAIN_ID（0 は不可）。')
    p.add_argument('--dds-xml', type=pathlib.Path, default=DEFAULT_DDS_XML)
    p.add_argument('--startup-timeout', type=float, default=120.0,
                   help='costmap が地図を読み終えるまで待つ上限 [s]。原寸の地図は膨張の計算に数秒かかる。')
    p.add_argument('--plan-timeout', type=float, default=60.0, help='1 組の計画を待つ上限 [s]。')
    p.add_argument('--crop-margin-m', type=float, default=5.0)
    a = p.parse_args()
    if a.domain == 0:
        p.error('--domain 0 は実機のスタックと同じ domain なので使わない')
    return a


def set_ros_env(a):
    # rclpy を import する前に決める。子プロセスにも同じ環境を渡す。
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = f'file://{a.dds_xml.resolve()}'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)


def exit_on_signal(signum, _frame):
    # SIGTERM・SIGHUP（端末を閉じた）でも with の __exit__ と finally を通し、子を残さない。
    # 既定の動作のままだと Python は後始末をせずに死に、map_server などが domain に残る。
    raise SystemExit(128 + signum)


def load_map(yaml_path):
    m = yaml.safe_load(yaml_path.read_text())
    img = np.array(Image.open(yaml_path.parent / m['image']).convert('L'))
    # map_server の trinary（negate 0）：p = (255 - v) / 255。OCC は p > occupied_thresh。
    p = (255.0 - img.astype(np.float32)) / 255.0
    occ = p > float(m.get('occupied_thresh', 0.65))
    free = p < float(m.get('free_thresh', 0.196))
    return {'res': float(m['resolution']), 'origin': m['origin'][:2], 'img': img, 'occ': occ, 'free': free}


def world_to_px(mp, xy):
    res, (ox, oy) = mp['res'], mp['origin']
    H = mp['img'].shape[0]
    c = np.floor((xy[:, 0] - ox) / res).astype(int)
    r = (H - 1 - np.floor((xy[:, 1] - oy) / res)).astype(int)
    return c, r


def in_bounds(mp, xy):
    H, W = mp['img'].shape
    c, r = world_to_px(mp, np.asarray(xy, dtype=float))
    return c, r, (c >= 0) & (c < W) & (r >= 0) & (r < H)


def costmap_to_pgm_rows(data, W, H):
    """costmap_raw の data（行は map の −y 側＝下から）を、pgm と同じ上からの行の (H, W) に直す。"""
    return np.frombuffer(bytes(data), dtype=np.uint8).reshape(H, W)[::-1]


def costmap_mismatch(cost, mp):
    """global costmap（pgm の行の向きに直した物）が、読ませた pgm と食い違うセルを数える。

    static_layer は OCC を LETHAL（254）にし、inflation はそれを下げない。FREE は 0 か inflation のコストで、
    NO_INFORMATION（255）にはならない。両方 0 なら、この地図を読み終えた costmap とみなす。上下を返し
    忘れる・別の地図の costmap を受ける・全面 UNKNOWN の初期の costmap を受けると、0 にならない。
    """
    return {'occ_not_lethal': int((cost[mp['occ']] != LETHAL).sum()),
            'unknown_on_free': int((cost[mp['free']] == NO_INFORMATION).sum())}


def check_domain_empty(node, wait_s=3.0):
    """起動する前に、同じ domain に map_server・planner_server などが居ないことを確かめる。"""
    import rclpy
    t0 = time.time()
    # discovery が済むまで少し回す。居るのに見えないうちに起動すると、二重の map_server になる。
    while time.time() - t0 < wait_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    found = sorted({f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces()
                    if n in CONFLICTING_NODES})
    if found:
        raise RuntimeError(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")} に既に {", ".join(found)} が居る。'
                           '止めるか、--domain で空いている domain を選ぶ')


class Stack:
    """1 枚の地図に対する map_server + planner_server。with を抜けると子を全部止める。"""

    def __init__(self, a, map_yaml, log_dir):
        self.a, self.map_yaml, self.log_dir, self.procs = a, map_yaml, log_dir, []

    def _spawn(self, name, cmd):
        log = open(self.log_dir / f'{name}.log', 'w')
        try:
            # 新しいセッションにして、ros2 run が起こす孫プロセスまでまとめて止められるようにする。
            pr = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.procs.append((name, pr, log))

    def __enter__(self):
        # with は __enter__ が例外で抜けると __exit__ を呼ばない。途中の起動の失敗・シグナルでも、
        # 起動済みの子を残さないよう、ここで止める。
        try:
            params = str(self.a.params.resolve())
            self._spawn('static_tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                                      '--frame-id', 'map', '--child-frame-id', 'base_link'])
            # nav2_params.yaml の map_server.yaml_filename は空（nav_launch が差し込む）ので、ここで上書きする。
            self._spawn('map_server', ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                                       '--params-file', params,
                                       '-p', f'yaml_filename:={self.map_yaml.resolve()}'])
            self._spawn('planner_server', ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args',
                                           '--params-file', params])
            self._spawn('lifecycle_manager', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                                              '--ros-args', '-r', '__node:=lifecycle_manager_plan_check',
                                              '-p', 'autostart:=true',
                                              '-p', 'node_names:=[map_server, planner_server]'])
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *exc):
        for _, pr, _ in self.procs:
            if pr.poll() is None:
                try:
                    os.killpg(pr.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 10.0
        for _, pr, log in self.procs:
            try:
                pr.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(pr.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                pr.wait()
            log.close()
        self.procs = []
        return False


def plan_one(node, client, goal, timeout, mp, cost, rec):
    import rclpy
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done():
        # 受け付けの返事が来ていないので、取り消す相手（goal handle）もまだ無い。
        rec['status'] = 'send_timeout'
        return rec
    gh = fut.result()
    if gh is None or not gh.accepted:
        rec['status'] = 'rejected'
        return rec
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=timeout)
    if not rf.done():
        # 残すと planner が次の組の要求を後回しにし、次の組の結果と planning_time を乱す。
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cf, timeout_sec=5.0)
        rec['status'] = 'result_timeout_canceled'
        return rec
    res = rf.result()
    rec['status'] = int(res.status)
    if len(res.result.path.poses) > 1:
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in res.result.path.poses])
        rec.update(success=True, xy=xy.round(3).tolist(),
                   planning_s=round(res.result.planning_time.sec + res.result.planning_time.nanosec * 1e-9, 3))
        c, r, ok = in_bounds(mp, xy)
        rec['max_cost'] = int(cost[r[ok], c[ok]].max()) if ok.any() else None
    return rec


def plan_all(a, map_yaml, mp, wps, log_dir):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.msg import Costmap

    out = {'map': str(map_yaml), 'pairs': []}
    rclpy.init()
    try:
        node = rclpy.create_node('nav2_plan_check')
        check_domain_empty(node)
        with Stack(a, map_yaml, log_dir):
            costmap = {}

            def on_costmap(msg):
                costmap['msg'] = msg

            # costmap_raw は 1 Hz で全面を出す。volatile で受ければ transient_local の発行側とも合う。
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(Costmap, '/global_costmap/costmap_raw', on_costmap, qos)
            client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
            H, W = mp['img'].shape
            t0 = time.time()
            # 読ませた地図と一致する costmap が届くまで待つ。大きさや値の有無だけでは足りない：2026-11-19 の
            # 初回は、地図の大きさで全面 UNKNOWN（255）の costmap が先に届き、その間に投げた最初の 4 組が
            # 失敗した（同じ組は後で成功する）。待っても一致しなければ、別の地図か向きの誤りとして止める。
            cost, mism = None, None
            while True:
                rclpy.spin_once(node, timeout_sec=0.5)
                msg = costmap.pop('msg', None)
                if msg is not None and msg.metadata.size_x == W and msg.metadata.size_y == H:
                    cost = costmap_to_pgm_rows(msg.data, W, H)
                    mism = costmap_mismatch(cost, mp)
                    if not any(mism.values()):
                        break
                if time.time() - t0 > a.startup_timeout:
                    raise RuntimeError(f'{map_yaml}: {a.startup_timeout} s 待っても、読ませた地図と一致する '
                                       f'global costmap が来ない（最後の食い違い {mism}。{log_dir} のログを見る）')
            if not client.wait_for_server(timeout_sec=a.startup_timeout):
                raise RuntimeError('compute_path_to_pose の action server が無い')
            out['startup_s'] = round(time.time() - t0, 1)
            out['costmap_mismatch'] = mism

            def pose(w):
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x, ps.pose.position.y = float(w['x']), float(w['y'])
                ps.pose.orientation.z = math.sin(float(w.get('yaw', 0.0)) / 2)
                ps.pose.orientation.w = math.cos(float(w.get('yaw', 0.0)) / 2)
                return ps

            for s in wps:
                for g in wps:
                    if s['name'] == g['name']:
                        continue
                    goal = ComputePathToPose.Goal(goal=pose(g), start=pose(s), planner_id='GridBased',
                                                  use_start=True)
                    out['pairs'].append(plan_one(node, client, goal, a.plan_timeout, mp, cost,
                                                 {'start': s['name'], 'goal': g['name'], 'success': False}))
    finally:
        rclpy.try_shutdown()
    return out


def path_metrics(mp, dist_occ_m, dist_nonfree_m, rec):
    xy = np.asarray(rec['xy'])
    seg = np.hypot(*np.diff(xy, axis=0).T)
    rec['length_m'] = round(float(seg.sum()), 2)
    c, r, ok = in_bounds(mp, xy)
    # 地図の外の点は距離を測れない（負の添字は反対の端を読んでしまう）。数だけ残して内側で測る。
    rec['n_outside_map'] = int((~ok).sum())
    if not ok.any():
        return
    xy, c, r = xy[ok], c[ok], r[ok]
    d = dist_occ_m[r, c]
    rec['min_dist_occ_m'] = round(float(d.min()), 3)
    i = int(np.argmin(d))
    rec['min_dist_occ_at'] = [round(float(xy[i, 0]), 2), round(float(xy[i, 1]), 2)]
    # inflation_radius 0.6 m の内側を通る経路長の割合。0.45 m（robot_radius）未満は inscribed。
    rec['frac_within_0_6m'] = round(float((d < 0.6).mean()), 3)
    rec['n_within_0_45m'] = int((d < 0.45).sum())
    # FREE でない所（UNKNOWN か OCC）までの距離。inflation は UNKNOWN を膨らませないので、OCC が遠くても
    # 経路は FREE と UNKNOWN の境（手塗りの縁）に寄りうる。154w で縁を広げて効くのはこちら。
    dn = dist_nonfree_m[r, c]
    rec['min_dist_nonfree_m'] = round(float(dn.min()), 3)
    rec['p05_dist_nonfree_m'] = round(float(np.percentile(dn, 5)), 3)
    rec['frac_nonfree_within_0_6m'] = round(float((dn < 0.6).mean()), 3)


def shift_metrics(base_xy, xy):
    base_xy, xy = np.asarray(base_xy), np.asarray(xy)
    d_new = cKDTree(base_xy).query(xy)[0]
    d_old = cKDTree(xy).query(base_xy)[0]
    return {'mean_m': round(float(d_new.mean()), 3), 'p95_m': round(float(np.percentile(d_new, 95)), 3),
            'hausdorff_m': round(float(max(d_new.max(), d_old.max())), 3)}


def draw_crop(mp, results, margin_m, path):
    ok_pairs = [p for res in results for p in res['pairs'] if p['success']]
    if not ok_pairs:
        return None
    pts = np.concatenate([np.asarray(p['xy']) for p in ok_pairs])
    c, r, ok = in_bounds(mp, pts)
    if not ok.any():
        return None
    c, r = c[ok], r[ok]
    m = int(round(margin_m / mp['res']))
    H, W = mp['img'].shape
    c0, c1 = max(0, c.min() - m), min(W, c.max() + m)
    r0, r1 = max(0, r.min() - m), min(H, r.max() + m)
    # 背景は基準の地図：OCC = 黒、FREE = 白、UNKNOWN = 灰。
    occ, free = mp['occ'][r0:r1, c0:c1], mp['free'][r0:r1, c0:c1]
    rgb = np.full(occ.shape + (3,), 160, np.uint8)
    rgb[free] = 255
    rgb[occ] = 0
    im = Image.fromarray(rgb)
    dr = ImageDraw.Draw(im)
    colours = [(0, 90, 255), (230, 0, 0), (0, 160, 0), (200, 0, 200)]
    for k, res in enumerate(results):
        for p in res['pairs']:
            if p['success']:
                pc, pr = world_to_px(mp, np.asarray(p['xy']))
                dr.line(list(zip((pc - c0).tolist(), (pr - r0).tolist())), fill=colours[k % 4], width=1)
    im.save(path, optimize=True)
    return [int(c0), int(r0), int(c1), int(r1)]


def main():
    a = parse_args()
    set_ros_env(a)
    signal.signal(signal.SIGTERM, exit_on_signal)
    signal.signal(signal.SIGHUP, exit_on_signal)
    a.out_dir.mkdir(parents=True, exist_ok=True)
    wps = yaml.safe_load(a.waypoints.read_text())['waypoints']
    results, maps = [], []
    for k, map_yaml in enumerate(a.map):
        mp = load_map(map_yaml)
        log_dir = a.out_dir / f'logs_{k}'
        log_dir.mkdir(exist_ok=True)
        res = plan_all(a, map_yaml, mp, wps, log_dir)
        dist = ndimage.distance_transform_edt(~mp['occ']) * mp['res']
        dist_nf = ndimage.distance_transform_edt(mp['free']) * mp['res']
        for rec in res['pairs']:
            if rec['success']:
                path_metrics(mp, dist, dist_nf, rec)
        results.append(res)
        maps.append(mp)
    if len(results) > 1:
        base = {(p['start'], p['goal']): p for p in results[0]['pairs']}
        for res in results[1:]:
            for p in res['pairs']:
                b = base.get((p['start'], p['goal']))
                if p['success'] and b and b['success']:
                    p['shift_from_first'] = shift_metrics(b['xy'], p['xy'])
                    p['length_delta_m'] = round(p['length_m'] - b['length_m'], 2)
    # 全組が失敗しても plans.json は書く（失敗の status を残す）。絵だけ省く。
    crop = draw_crop(maps[0], results, a.crop_margin_m, a.out_dir / 'plans_crop.png')
    summary = {'params': str(a.params), 'domain': a.domain, 'crop_px_c0_r0_c1_r1': crop, 'maps': results}
    (a.out_dir / 'plans.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False))

    print('map\tstart->goal\tok\tlength_m\tmin_dist_occ_m\tfrac<0.6m\tmin/p05_dist_nonfree_m\t'
          'nonfree_frac<0.6m\tmax_cost\tshift_mean/p95/haus_m')
    for res in results:
        for p in res['pairs']:
            sh = p.get('shift_from_first')
            print('\t'.join(map(str, [
                pathlib.Path(res['map']).name, f"{p['start']}->{p['goal']}", p['success'],
                p.get('length_m', '-'), p.get('min_dist_occ_m', '-'), p.get('frac_within_0_6m', '-'),
                f"{p.get('min_dist_nonfree_m', '-')}/{p.get('p05_dist_nonfree_m', '-')}",
                p.get('frac_nonfree_within_0_6m', '-'), p.get('max_cost', '-'),
                f"{sh['mean_m']}/{sh['p95_m']}/{sh['hausdorff_m']}" if sh else '-'])))
    return 0 if all(p['success'] for res in results for p in res['pairs']) else 1


if __name__ == '__main__':
    sys.exit(main())

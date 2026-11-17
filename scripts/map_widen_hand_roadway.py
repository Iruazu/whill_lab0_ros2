#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_widen_hand_roadway.py — 手塗りの roadway を、補正後の地理院の縁（202d）まで外へ広げた候補を作る（#154、154w）。

ユーザーの判断（2026-11-17、判断票の回答）に従う：
  Q1  広げるのは、202d の区間のうち status=snapped かつ kind が step / structure（赤・橙）に面する所だけ。
  Q2  左上の広場、斜めの舗装路の入口、発進点西の駐輪・駐車の帯、北の道の駐車場の入口は含めない
      （--exclude-box の既定値。補正後の面・赤橙の区間に入っていないことを実測で確かめ、入っていれば箱で落とす）。
  Q3  北の道の東の路肩の帯は、補正後の縁のまま（補正後の面の外なので、何もしなくても入らない）。
  Q4  強度だけで寄せた区間（kind=intensity）は採らない（未補正と同じ扱い）。
  Q5  補正後の地理院の面は、手塗りの確定版の候補づくりに使う。
  前提 202c の強度 < 20 の自動抽出は、背の低い植栽を含むので根拠に使わない（読まない）。

広げてよい帯（widen_band）：
  1. 補正後の地理院の道路の面（gsi_snapped_road.png）の中で、手塗りの roadway の外。
  2. 手塗りからの距離が --max-widen-m 以内（202d の寄せる量の上限 1.5 m と同じ既定。GSI の縁が
     手塗りから遠い所は、縁の位置そのものより手塗りの意図（入口・広場を塗らない）が効いているとみなす）。
  3. そのセルに最も近い補正後の縁の区間が赤・橙で、最も近い「それ以外」（青・桃・灰）の区間より
     --tie-m 以上近い。境目で迷うセルは広げない側に倒す（安全側）。
  4. 段・構造物（v2 の occupied_step / occupied_structure）・塩の候補（salt_candidates）のセルを
     --barrier-dilate-m だけ太らせて除く。帯を free の塗りにも入れるので、除かないと機械の障害物を消してしまう。
     太らせるのは、static.pcd が 10 cm のボクセルなので、植栽・生垣の構造物が 1 セルおきの点の並びになり、
     太らせないと点の間のセルが帯に残って植え込みの中へ入り込むため（初回の実行の目視、x 104〜106、y 18〜19）。
  5. 除外の箱（--exclude-box）の中を除く。
  6. 残りの 8 連結の成分のうち、手塗りに接するものだけを残す（離れた飛び地は足さない）。
  内側へ（手塗りを削る方向）は変えない。

合成は README の 3 節と同じ順（gen_auto_free_mask.py → compose_occupancy.py）。帯は roadway と free の
両方に足す（--free-policy paint、既定）。旧地図の直接の拡幅と同じ扱いで、roadway だけだと free_evidence
（レイの通過）の無いセルは UNKNOWN のまま残り、帯がまだらになる。帯から段・構造物・塩を先に除いてあるので、
free に足しても機械の障害物は消えない。--free-policy evidence にすると roadway だけに足す（比較用）。

既定の地図（occupancy_candidate.yaml、v2/candidate_final.pgm、v2/candidates/）・static.pcd・bag は読むだけで変えない。
出力は --out-dir（小さい画像と数値、Git 管理）、--work-dir（地図本体・原寸のマスク、Git 外）、
--gimp-dir の 19_widened_candidate.png（Git 外）。
"""

import argparse
import csv
import json
import math
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'
HOME = pathlib.Path.home()

OCC, FREE, UNKNOWN = 0, 254, 205
TRUSTED_KINDS = ('step', 'structure')

# Q2 の 4 か所。x の範囲（と広場・入口の y）はユーザーの回答の座標のまま。回答に y の無い 2 か所は、
# 手塗りの南の縁の外を覆い、道の反対側（北の縁）の帯に掛からない値を原寸の切り出しで決めた
# （発進点の道の南の縁は y ≈ −1.5〜−2、北の道の南の縁は y ≈ 159.5〜160、北の縁は y ≈ 163.5）。
DEFAULT_EXCLUDE = [
    ('plaza_nw', -10.0, -2.0, 3.0, 15.0),            # 左上の広場
    ('diagonal_entrance', 30.0, 45.0, -8.0, -2.0),   # 斜めの舗装路の入口
    ('start_west_parking', -10.0, 0.0, -9.0, -1.0),  # 発進点西の駐輪・駐車の帯（南の縁）
    ('north_parking_entrance', 18.0, 44.0, 150.0, 160.5),  # 北の道の駐車場の入口（南）
]
# 集計の範囲。start_bldg3 は 202d の BOXES と同じ。north_road は 202d の箱を、目視の切り出し（x 96 まで）に合わせて東へ延ばした
REGIONS = {'start_bldg3': (-10.0, 70.0, -15.0, 15.0), 'north_road': (-125.0, 96.0, 145.0, 175.0)}
CROPS = [('start', -15.0, 45.0, -9.0, 4.0), ('north', -12.0, 96.0, 151.0, 168.0)]
KIND_COLOR = {'step': (230, 0, 0), 'structure': (255, 140, 0), 'intensity': (0, 110, 255),
              'none': (255, 120, 200), 'far': (150, 150, 150)}
BAND_COLOR = (255, 0, 255)


def parse_args(argv=None):
    b = HOME / 'bags/m8-experiments'
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml')
    p.add_argument('--hand-roadway', type=pathlib.Path, default=D / 'v2/candidates/candidate_roadway_merged.png')
    p.add_argument('--hand-free', type=pathlib.Path, default=D / 'v2/candidates/candidate_free_merged.png')
    p.add_argument('--keepout', type=pathlib.Path, default=D / 'v2/candidates/candidate_keepout_mask.png')
    p.add_argument('--baseline-map', type=pathlib.Path, default=D / 'occupancy_candidate.yaml',
                   help='比べる相手（今の既定の地図）。読むだけ。')
    p.add_argument('--waypoints', type=pathlib.Path, default=D / 'waypoints.yaml')
    p.add_argument('--gsi-road', type=pathlib.Path, default=b / '202d/gsi_snapped_road.png',
                   help='202d の補正後の地理院の道路の面（原寸、白・不透明 = 道）。')
    p.add_argument('--segments', type=pathlib.Path, default=b / '202d/edge_segments.csv',
                   help='202d の全区間の表（map_gsi_edge_snap.py の出力）。')
    p.add_argument('--nav2-params', type=pathlib.Path, default=REPO / 'src/whill_navigation/config/nav2_params.yaml')
    p.add_argument('--max-widen-m', type=float, default=1.5, help='手塗りから外へ広げる距離の上限 [m]。')
    p.add_argument('--tie-m', type=float, default=0.05,
                   help='赤・橙の区間が、それ以外の区間よりこれ以上近いセルだけを広げる [m]。')
    p.add_argument('--barrier-dilate-m', type=float, default=0.05,
                   help='段・構造物・塩を太らせる半径 [m]。1 セル（0.05）で 10 cm ボクセルの点の間を閉じる。')
    p.add_argument('--sample-m', type=float, default=0.025, help='区間を点に分ける間隔 [m]（最近傍の判定用）。')
    p.add_argument('--exclude-box', nargs=5, action='append', default=None,
                   metavar=('NAME', 'X0', 'X1', 'Y0', 'Y1'),
                   help='広げない箱（map 座標）。1 つでも渡すと既定の 4 か所（Q2）を置き換える。')
    p.add_argument('--free-policy', choices=('paint', 'evidence'), default='paint',
                   help='paint：帯を roadway と free の両方に足す（既定）。evidence：roadway だけ（free_evidence の'
                        '無いセルは UNKNOWN のまま）。')
    p.add_argument('--out-dir', type=pathlib.Path, default=D / 'v2/experiments/154w')
    p.add_argument('--work-dir', type=pathlib.Path, default=b / '154w')
    p.add_argument('--gimp-dir', type=pathlib.Path, default=b / 'gimp-layers')
    p.add_argument('--no-gimp', action='store_true', help='GIMP 用の 19 番を書かない。')
    p.add_argument('--underlay-intensity', type=pathlib.Path, default=b / 'gimp-layers/04_intensity.png')
    p.add_argument('--underlay-photo', type=pathlib.Path, default=b / 'gimp-layers/01_gsi_photo.png')
    p.add_argument('--intensity-npz', type=pathlib.Path, default=b / '202c/ground_raster.npz',
                   help='確認だけに使う地面の反射強度（202c）。帯の中の芝・植栽（明るい側）を数える。根拠には使わない。')
    p.add_argument('--grass-intensity', type=float, default=20.0,
                   help='これ以上を「舗装でない」（芝・植栽）とみなす（202c の壁と同じ）。暗い側は背の低い植栽で'
                        '誤るので、明るい側だけを確認に使う。')
    p.add_argument('--min-seg-cells', type=int, default=4,
                   help='区間を「広げた」と数える、その区間に割り当てた帯のセル数の下限（0.01 m²）。')
    return p.parse_args(argv)


# ---------------------------------------------------------------- 入出力

def load_grid(layers_yaml):
    doc = yaml.safe_load(layers_yaml.read_text())
    g = doc['grid']
    return doc, int(g['width']), int(g['height']), float(g['resolution']), \
        float(g['origin'][0]), float(g['origin'][1])


def load_mask(path, hw):
    """compose_occupancy.load_sidecar_mask と同じ解釈（白・不透明が塗り）。"""
    im = Image.open(path)
    m = np.array(im.convert('L')) >= 128
    if im.mode == 'RGBA':
        m &= np.array(im.getchannel('A')) > 0
    if m.shape != hw:
        raise SystemExit(f'{path}: 形 {m.shape} が格子 {hw} と違う')
    return m


def load_opaque(path, hw):
    """compose_occupancy.load_rgba_opaque と同じ（v2 の機械レイヤは α=255 が証拠）。"""
    a = np.array(Image.open(path).convert('RGBA'))[..., 3] == 255
    if a.shape != hw:
        raise SystemExit(f'{path}: 形 {a.shape} が格子 {hw} と違う')
    return a


def save_mask_rgba(path, mask):
    """手塗りの候補と同じ形式（白・不透明 / 透明）。compose_occupancy にそのまま渡せる。"""
    rgba = np.zeros(mask.shape + (4,), np.uint8)
    rgba[mask] = 255
    Image.fromarray(rgba, 'RGBA').save(path, optimize=True)


def read_segments(path):
    """202d の区間の表を読み、補正後の位置の区間（中点 + offset × 外向きの法線、長さ len_m）にする。
    頂点の実際の位置は両隣の区間の移動の平均なので、区間の端で数 cm ずれうる（最近傍の判定には十分）。"""
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            rows.append(r)
    n = len(rows)
    mid = np.array([[float(r['x']), float(r['y'])] for r in rows]).reshape(n, 2)
    nv = np.array([[float(r['out_nx']), float(r['out_ny'])] for r in rows]).reshape(n, 2)
    off = np.array([float(r['offset_m']) for r in rows])
    L = np.array([float(r['len_m']) for r in rows])
    status = np.array([r['status'] for r in rows])
    kind = np.array([r['kind'] for r in rows])
    dup = np.array([r['dup'] == 'True' for r in rows])
    trusted = (status == 'snapped') & np.isin(kind, TRUSTED_KINDS)
    return {'mid': mid + off[:, None] * nv, 'nv': nv, 'len': L, 'off': off, 'status': status,
            'kind': kind, 'dup': dup, 'trusted': trusted, 'line': np.array([int(r['line']) for r in rows]),
            'seg': np.array([int(r['seg']) for r in rows])}


def sample_segments(seg, step):
    """区間を step 間隔の点に分ける。返り値は (点 [N,2]、点の元の区間の番号 [N])。"""
    pts, idx = [], []
    for i in range(len(seg['len'])):
        t = np.array([-seg['nv'][i, 1], seg['nv'][i, 0]])
        k = max(2, int(math.ceil(seg['len'][i] / step)) + 1)
        s = np.linspace(-seg['len'][i] / 2, seg['len'][i] / 2, k)
        pts.append(seg['mid'][i] + s[:, None] * t)
        idx.append(np.full(k, i))
    if not pts:
        return np.zeros((0, 2)), np.zeros(0, int)
    return np.vstack(pts), np.concatenate(idx)


def cell_xy(rows, cols, H, res, ox, oy):
    return ox + (cols + 0.5) * res, oy + (H - rows - 0.5) * res


def box_mask(hw, res, ox, oy, box):
    """map 座標の箱 (x0, x1, y0, y1) に中心が入るセル。"""
    H, W = hw
    x0, x1, y0, y1 = box
    c0, c1 = max(0, int(math.ceil((x0 - ox) / res - 0.5))), min(W - 1, int(math.floor((x1 - ox) / res - 0.5)))
    r0 = max(0, int(math.ceil(H - 0.5 - (y1 - oy) / res)))
    r1 = min(H - 1, int(math.floor(H - 0.5 - (y0 - oy) / res)))
    m = np.zeros(hw, bool)
    if c0 <= c1 and r0 <= r1:
        m[r0:r1 + 1, c0:c1 + 1] = True
    return m


# ---------------------------------------------------------------- 広げる帯

def widen_band(hand, gsi, barrier, exclude, pts, pts_trusted, res, ox, oy, max_widen_m, tie_m, barrier_dilate_m=0.0):
    """広げてよい帯を返す（モジュールの説明の 1〜6）。

    hand, gsi, barrier, exclude：同じ形の bool の格子。pts：補正後の縁の点（map 座標 [N,2]）、
    pts_trusted：その点が赤・橙の区間か。返り値は (帯、途中の段階の数、帯のセルの最も近い赤・橙の点の番号)。"""
    H = hand.shape[0]
    stages = {}
    # 手塗りからの距離。cv2 の距離変換は 0 のセルまでの距離を測るので、手塗りを 0 にする
    d_hand = cv2.distanceTransform((~hand).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    outside = gsi & ~hand
    stages['gsi_outside_hand'] = int(outside.sum())
    cand = outside & (d_hand <= max_widen_m + 1e-9)
    stages['within_cap'] = int(cand.sum())
    rr, cc = np.nonzero(cand)
    keep = np.zeros(rr.size, bool)
    near_idx = np.full(rr.size, -1)
    if rr.size and pts_trusted.any():
        x, y = cell_xy(rr, cc, H, res, ox, oy)
        q = np.column_stack([x, y])
        dT, iT = cKDTree(pts[pts_trusted]).query(q)
        if (~pts_trusted).any():
            dU, _ = cKDTree(pts[~pts_trusted]).query(q)
        else:
            dU = np.full(rr.size, np.inf)
        keep = dT + tie_m < dU
        near_idx = np.flatnonzero(pts_trusted)[iT]
    nearest = np.zeros_like(hand)
    nearest[rr[keep], cc[keep]] = True
    stages['nearest_trusted'] = int(nearest.sum())
    r_ = int(round(barrier_dilate_m / res))
    if r_ > 0:
        barrier = cv2.dilate(barrier.astype(np.uint8), cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * r_ + 1, 2 * r_ + 1))).astype(bool)
    b = nearest & ~barrier
    stages['minus_barrier'] = int(b.sum())
    b &= ~exclude
    stages['minus_exclude'] = int(b.sum())
    # 手塗りに接する成分だけ（8 連結）。手塗りを 1 セル太らせた所と重なる成分
    n, lab = cv2.connectedComponents(b.astype(np.uint8), connectivity=8)
    touch = cv2.dilate(hand.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    ok = np.zeros(n, bool)
    ok[np.unique(lab[b & touch])] = True
    ok[0] = False
    band = ok[lab]
    stages['band'] = int(band.sum())
    stages['dropped_islands'] = stages['minus_exclude'] - stages['band']
    # 帯のセルごとの最も近い赤・橙の点（区間の集計用）
    idx_img = np.full(hand.shape, -1, np.int64)
    idx_img[rr, cc] = near_idx
    return band, stages, idx_img, d_hand


# ---------------------------------------------------------------- 合成と確認

def run(cmd):
    print('  $', ' '.join(str(c) for c in cmd), flush=True)
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise SystemExit(f'失敗（{r.returncode}）: {cmd[1]}\n{r.stderr[-2000:]}')
    return r


def compose(layers_yaml, roadway, free, keepout, vd):
    """README 3 節と同じ順（gen_auto_free_mask → compose_occupancy）。"""
    s = REPO / 'scripts'
    vd.mkdir(parents=True, exist_ok=True)
    r = run([sys.executable, s / 'gen_auto_free_mask.py', '--layers-yaml', layers_yaml, '--roadway-mask', roadway,
             '--output-free-mask-auto', vd / 'free_mask_auto.png', '--output-salt-csv', vd / 'salt.csv'])
    if r.returncode:
        raise SystemExit(f'gen_auto_free_mask 失敗: {r.stderr[-2000:]}')
    r = run([sys.executable, s / 'compose_occupancy.py', '--layers-yaml', layers_yaml, '--roadway-mask', roadway,
             '--free-mask', free, '--free-mask-auto', vd / 'free_mask_auto.png', '--keepout-mask', keepout,
             '--output-pgm', vd / 'final.pgm'])
    if r.returncode:
        raise SystemExit(f'compose_occupancy 失敗: {r.stderr[-2000:]}')
    return vd / 'final.yaml'


def route_check(map_yaml, waypoints):
    r = run([sys.executable, REPO / 'scripts/map_route_check.py', map_yaml, waypoints])
    return json.loads(r.stdout)


def load_pgm(map_yaml):
    m = yaml.safe_load(pathlib.Path(map_yaml).read_text())
    return np.array(Image.open(pathlib.Path(map_yaml).parent / m['image']))


def nav2_footprint(path):
    """nav2_params.yaml の global_costmap の robot_radius と inflation_radius を読む（経路計画は global 側）。"""
    doc = yaml.safe_load(path.read_text())
    gc = doc['global_costmap']['global_costmap']['ros__parameters']
    lc = doc['local_costmap']['local_costmap']['ros__parameters']
    return {'global_robot_radius': float(gc['robot_radius']),
            'global_inflation_radius': float(gc['inflation_layer']['inflation_radius']),
            'global_cost_scaling_factor': float(gc['inflation_layer']['cost_scaling_factor']),
            'local_robot_radius': float(lc['robot_radius']),
            'local_inflation_radius': float(lc['inflation_layer']['inflation_radius'])}


def pct(a, q):
    return round(float(np.percentile(a, q)), 3) if len(a) else None


# ---------------------------------------------------------------- 画像

def crop_slice(H, res, ox, oy, box):
    x0, x1, y0, y1 = box
    c0, c1 = int((x0 - ox) / res), int(math.ceil((x1 - ox) / res))
    r0, r1 = int(H - (y1 - oy) / res), int(math.ceil(H - (y0 - oy) / res))
    return slice(max(0, r0), r1), slice(max(0, c0), c1)


def to_px(xy, H, res, ox, oy, r0, c0):
    return [((x - ox) / res - 0.5 - c0, (oy + H * res - y) / res - 0.5 - r0) for x, y in xy]


def draw_crop(under, sl, layers, seg, boxes, H, res, ox, oy, path):
    """原寸（5 cm/画素）の切り出し。layers：(マスク, 色, α) の順に重ねる。区間は補正後の位置を根拠の色で描く。"""
    rs, cs = sl
    base = np.array(Image.open(under).convert('RGB'))[rs, cs].astype(np.float32)
    for m, col, a in layers:
        mm = m[rs, cs]
        base[mm] = base[mm] * (1 - a) + np.array(col, np.float32) * a
    im = Image.fromarray(base.clip(0, 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    r0, c0 = rs.start, cs.start
    for i in range(len(seg['len'])):
        t = np.array([-seg['nv'][i, 1], seg['nv'][i, 0]])
        P = [seg['mid'][i] - t * seg['len'][i] / 2, seg['mid'][i] + t * seg['len'][i] / 2]
        px = to_px(P, H, res, ox, oy, r0, c0)
        if not any(-50 < u < im.size[0] + 50 and -50 < v < im.size[1] + 50 for u, v in px):
            continue
        st = seg['status'][i]
        k = seg['kind'][i] if st == 'snapped' else ('far' if st == 'far' else 'none')
        d.line(px, fill=KIND_COLOR.get(k, KIND_COLOR['none']), width=2)
    for name, x0, x1, y0, y1 in boxes:
        px = to_px([(x0, y1), (x1, y0)], H, res, ox, oy, r0, c0)
        d.rectangle([px[0], px[1]], outline=(0, 255, 255), width=2)
        d.text((px[0][0] + 4, px[0][1] + 4), name, fill=(0, 255, 255))
    im.save(path, optimize=True)
    return path.name


def edge_of(m):
    return m & ~cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)


# ---------------------------------------------------------------- main

def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    boxes = [(b[0], *map(float, b[1:])) for b in args.exclude_box] if args.exclude_box else DEFAULT_EXCLUDE
    doc, W, H, res, ox, oy = load_grid(args.layers_yaml)
    hw = (H, W)
    ld = args.layers_yaml.parent
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.work_dir.mkdir(parents=True, exist_ok=True)

    hand = load_mask(args.hand_roadway, hw)
    hand_free = load_mask(args.hand_free, hw)
    gsi = load_mask(args.gsi_road, hw)
    step = load_opaque(ld / doc['layers']['occupied_step'], hw)
    struct = load_opaque(ld / doc['layers']['occupied_structure'], hw)
    salt = load_opaque(ld / doc['layers']['salt_candidates'], hw)
    machine_free = np.array(Image.open(ld / doc['layers']['free_evidence']).convert('RGBA'))[..., 3] > 0
    barrier = step | struct | salt
    exclude = np.zeros(hw, bool)
    box_m = {}
    for name, *bx in boxes:
        box_m[name] = box_mask(hw, res, ox, oy, bx)
        exclude |= box_m[name]

    seg = read_segments(args.segments)
    pts, pidx = sample_segments(seg, args.sample_m)
    pts_trusted = seg['trusted'][pidx]
    print(f'[区間] {len(seg["len"]):,}（赤・橙 {int(seg["trusted"].sum()):,}）、点 {len(pts):,}', flush=True)

    band, stages, idx_img, d_hand = widen_band(hand, gsi, barrier, exclude, pts, pts_trusted, res, ox, oy,
                                               args.max_widen_m, args.tie_m, args.barrier_dilate_m)
    print(f'[帯] {stages}', flush=True)

    # Q2 の箱：補正後の面・赤橙の最近傍・箱で落とした量を実測で残す（箱が効いたかどうかの記録）
    band_nobox, _, _, _ = widen_band(hand, gsi, barrier, np.zeros(hw, bool), pts, pts_trusted, res, ox, oy,
                                     args.max_widen_m, args.tie_m, args.barrier_dilate_m)
    q2 = {}
    for name, *bx in boxes:
        m = box_m[name]
        q2[name] = {'box': bx, 'gsi_outside_hand_m2': round(float((gsi & ~hand & m).sum()) * res * res, 2),
                    'gsi_outside_hand_within_cap_m2':
                        round(float((gsi & ~hand & m & (d_hand <= args.max_widen_m)).sum()) * res * res, 2),
                    'band_without_box_m2': round(float((band_nobox & m).sum()) * res * res, 2),
                    'band_m2': round(float((band & m).sum()) * res * res, 2)}
    print(f'[Q2] {json.dumps(q2, ensure_ascii=False)}', flush=True)

    # ---- 合成
    roadway_w = hand | band
    free_w = hand_free | band if args.free_policy == 'paint' else hand_free
    save_mask_rgba(args.work_dir / 'roadway_widened.png', roadway_w)
    save_mask_rgba(args.work_dir / 'free_widened.png', free_w)
    save_mask_rgba(args.work_dir / 'band.png', band)
    new_yaml = compose(args.layers_yaml, args.work_dir / 'roadway_widened.png', args.work_dir / 'free_widened.png',
                       args.keepout, args.work_dir / 'compose')
    base_img = load_pgm(args.baseline_map)
    new_img = load_pgm(new_yaml)
    routes = {'baseline': route_check(args.baseline_map, args.waypoints),
              'widened': route_check(new_yaml, args.waypoints)}
    for k_, r_ in routes.items():
        wp = ' '.join(f'{w["name"]}:{w["free"]}/{w["component"]}/{w["snap_m"]}' for w in r_['waypoints'])
        print(f'[経路] {k_}: all_connected={r_["all_connected"]} {wp}', flush=True)

    # ---- 面積
    a = res * res
    base_free, new_free = base_img == FREE, new_img == FREE
    gained = new_free & ~base_free
    lost = base_free & ~new_free
    area = {'whole': None}
    area.update({k_: box_mask(hw, res, ox, oy, v) for k_, v in REGIONS.items()})
    areas = {}
    for k_, m in area.items():
        mm = np.ones(hw, bool) if m is None else m
        areas[k_] = {'band_m2': round(float((band & mm).sum()) * a, 2),
                     'free_gained_m2': round(float((gained & mm).sum()) * a, 2),
                     'free_lost_m2': round(float((lost & mm).sum()) * a, 2),
                     'free_baseline_m2': round(float((base_free & mm).sum()) * a, 1),
                     'free_widened_m2': round(float((new_free & mm).sum()) * a, 1)}
    band_unknown_evidence = int((band & ~machine_free).sum())
    # 帯の中の芝・植栽（強度が明るい側）。202d と同じく明るい側だけを使う（暗い側は背の低い植栽で誤る）
    grass = np.zeros(hw, bool)
    if args.intensity_npz.is_file():
        with np.load(args.intensity_npz) as z:
            im_ = z['intensity_mean']
            grass = np.isfinite(im_) & (z['intensity_count'] > 0) & (im_ >= args.grass_intensity)
        del im_
    occ_changed = int(((base_img == OCC) != (new_img == OCC)).sum())

    # ---- 区間ごとの集計（帯のセルを、最も近い赤・橙の区間に割り当てる）
    seg_of = np.full(idx_img.shape, -1, np.int64)
    sel = band & (idx_img >= 0)
    seg_of[sel] = pidx[idx_img[sel]]
    ids, counts = np.unique(seg_of[sel], return_counts=True)
    occ_new = new_img == OCC
    d_occ = cv2.distanceTransform((~occ_new).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    d_occ_base = cv2.distanceTransform((base_img != OCC).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    fp = nav2_footprint(args.nav2_params)
    infl, rr_ = fp['global_inflation_radius'], fp['global_robot_radius']
    hand_edge = edge_of(base_free)
    # 増えた FREE のうち、車体の中心が置ける所（OCC から robot_radius 以上）と、inflation の外（コスト 0）
    usable = {'gained_ge_robot_radius_m2': round(float((gained & (d_occ >= rr_)).sum()) * a, 2),
              'gained_ge_inflation_m2': round(float((gained & (d_occ >= infl)).sum()) * a, 2),
              'band_grass_m2': round(float((band & grass).sum()) * a, 2)}
    seg_rows = []
    for i, n_ in zip(ids, counts):
        if n_ < args.min_seg_cells or seg['dup'][i]:
            continue
        m = seg_of == i
        widths = d_hand[m]
        # この区間の帯の、外の縁（新しい FREE の縁）から OCC までの最短距離
        e = m & edge_of(new_free | m)
        de = d_occ[e] if e.any() else d_occ[m]
        x, y = seg['mid'][i]
        # 同じ区間の前の縁（今の既定の地図の FREE の縁）の OCC までの距離。区間の中点から 2 m 以内の縁
        rr0, cc0 = int(H - 1 - math.floor((y - oy) / res)), int(math.floor((x - ox) / res))
        w2 = int(2.0 / res)
        sub = hand_edge[max(0, rr0 - w2):rr0 + w2 + 1, max(0, cc0 - w2):cc0 + w2 + 1]
        dsub = d_occ_base[max(0, rr0 - w2):rr0 + w2 + 1, max(0, cc0 - w2):cc0 + w2 + 1][sub]
        seg_rows.append({'line': int(seg['line'][i]), 'seg': int(seg['seg'][i]), 'x': round(float(x), 2),
                         'y': round(float(y), 2), 'kind': str(seg['kind'][i]), 'offset_m': float(seg['off'][i]),
                         'band_cells': int(n_), 'width_max_m': round(float(widths.max()), 3),
                         'width_median_m': round(float(np.median(widths)), 3),
                         'grass_frac': round(float(grass[m].mean()), 3),
                         'new_edge_to_occ_min_m': round(float(de.min()), 3),
                         'old_edge_to_occ_min_m': round(float(dsub.min()), 3) if dsub.size else None})
    wmax = np.array([r['width_max_m'] for r in seg_rows])
    by_kind = {k_: sum(1 for r in seg_rows if r['kind'] == k_) for k_ in TRUSTED_KINDS}
    tight = [r for r in seg_rows if r['new_edge_to_occ_min_m'] < infl]
    tight_r = [r for r in seg_rows if r['new_edge_to_occ_min_m'] < rr_]
    newly_tight = [r for r in tight if r['old_edge_to_occ_min_m'] is None or r['old_edge_to_occ_min_m'] >= infl]
    # 隣り合う区間（同じ線で seg が続く）をまとめて列挙する
    sites = []
    for r in sorted(tight, key=lambda r: (r['line'], r['seg'])):
        if sites and sites[-1]['line'] == r['line'] and r['seg'] - sites[-1]['seg_to'] <= 2:
            s_ = sites[-1]
            s_['seg_to'] = r['seg']
            s_['n'] += 1
            s_['xs'].append(r['x'])
            s_['ys'].append(r['y'])
            s_['min_new'] = min(s_['min_new'], r['new_edge_to_occ_min_m'])
            if r['old_edge_to_occ_min_m'] is not None:
                s_['min_old'] = min(s_['min_old'], r['old_edge_to_occ_min_m'])
            s_['wmax'] = max(s_['wmax'], r['width_max_m'])
            s_['kinds'].add(r['kind'])
        else:
            sites.append({'line': r['line'], 'seg_from': r['seg'], 'seg_to': r['seg'], 'n': 1, 'xs': [r['x']],
                          'ys': [r['y']], 'min_new': r['new_edge_to_occ_min_m'],
                          'min_old': r['old_edge_to_occ_min_m'] if r['old_edge_to_occ_min_m'] is not None else 9.9,
                          'wmax': r['width_max_m'], 'kinds': {r['kind']}})
    site_rows = [{'line': s_['line'], 'segs': f'{s_["seg_from"]}-{s_["seg_to"]}', 'n_segments': s_['n'],
                  'x_range': [min(s_['xs']), max(s_['xs'])], 'y_range': [min(s_['ys']), max(s_['ys'])],
                  'new_edge_to_occ_min_m': s_['min_new'],
                  'old_edge_to_occ_min_m': None if s_['min_old'] == 9.9 else s_['min_old'],
                  'widen_max_m': s_['wmax'], 'kinds': sorted(s_['kinds'])} for s_ in sites]
    with open(args.out_dir / 'widened_segments.csv', 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=['line', 'seg', 'x', 'y', 'kind', 'offset_m', 'band_cells', 'width_max_m',
                                           'width_median_m', 'grass_frac', 'new_edge_to_occ_min_m',
                                           'old_edge_to_occ_min_m'])
        wr.writeheader()
        wr.writerows(seg_rows)

    # ---- 画像（原寸の切り出し。帯 = 桃、手塗りの roadway = 青の薄塗り、段 = 赤、構造物 = 橙）
    png = {}
    # 帯は薄く塗って縁を濃く描く（下絵の芝・舗装が見えるように）
    lay = [(hand, (0, 90, 255), 0.25), (band, BAND_COLOR, 0.35), (edge_of(band), BAND_COLOR, 1.0),
           (step, (255, 0, 0), 0.9),
           (struct, (255, 150, 0), 0.9), (edge_of(hand), (0, 60, 255), 1.0)]
    for cname, *cb in CROPS:
        sl = crop_slice(H, res, ox, oy, cb)
        for uname, up in (('intensity', args.underlay_intensity), ('photo', args.underlay_photo)):
            if up.is_file():
                png[f'{cname}_{uname}'] = draw_crop(up, sl, lay, seg, boxes, H, res, ox, oy,
                                                    args.out_dir / f'{cname}_crop_{uname}_1to1.png')
    # 全体の 1/8（帯だけを太らせて見えるようにする）
    k = 8
    ov = np.full((H // k, W // k, 3), 255, np.uint8)
    ds = lambda m: m[:H // k * k, :W // k * k].reshape(H // k, k, W // k, k).any(axis=(1, 3))  # noqa: E731
    ov[ds(base_free)] = (190, 210, 255)
    ov[ds(occ_new)] = (60, 60, 60)
    ov[cv2.dilate(ds(band).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)] = BAND_COLOR
    Image.fromarray(ov).save(args.out_dir / 'overview_band_8.png', optimize=True)
    png['overview'] = 'overview_band_8.png'

    if not args.no_gimp:
        args.gimp_dir.mkdir(parents=True, exist_ok=True)
        g = np.zeros((H, W, 4), np.uint8)
        g[band] = (255, 0, 255, 255)
        Image.fromarray(g, 'RGBA').save(args.gimp_dir / '19_widened_candidate.png', compress_level=3)
        print(f'[GIMP] {args.gimp_dir}/19_widened_candidate.png', flush=True)

    summary = {
        'params': {k_: (str(v) if isinstance(v, pathlib.Path) else v) for k_, v in vars(args).items()
                   if k_ != 'exclude_box'},
        'exclude_boxes': [list(b) for b in boxes], 'regions': REGIONS,
        'segments': {'total': int(len(seg['len'])), 'trusted': int(seg['trusted'].sum()),
                     'trusted_step': int((seg['trusted'] & (seg['kind'] == 'step')).sum()),
                     'trusted_structure': int((seg['trusted'] & (seg['kind'] == 'structure')).sum())},
        'stages_cells': stages, 'q2_boxes': q2, 'areas': areas,
        'band_cells_without_free_evidence': band_unknown_evidence,
        'band_cells': int(band.sum()), 'occ_cells_changed': occ_changed, 'usable': usable,
        'grass_segments_gt_0_3': [{k2: r[k2] for k2 in ('line', 'seg', 'x', 'y', 'kind', 'band_cells', 'grass_frac')}
                                  for r in seg_rows if r['grass_frac'] > 0.3],
        'widened_segments': {'count': len(seg_rows), 'by_kind': by_kind,
                             'width_max_median_m': pct(wmax, 50), 'width_max_p90_m': pct(wmax, 90),
                             'width_max_max_m': pct(wmax, 100)},
        'nav2': fp,
        'clearance': {'segments_new_edge_lt_inflation': len(tight), 'segments_new_edge_lt_robot_radius': len(tight_r),
                      'segments_newly_lt_inflation': len(newly_tight), 'sites': site_rows},
        'routes': routes,
        'counts': {'baseline': {str(v): int((base_img == v).sum()) for v in (OCC, FREE, UNKNOWN)},
                   'widened': {str(v): int((new_img == v).sum()) for v in (OCC, FREE, UNKNOWN)}},
        'png': png, 'elapsed_s': round(time.time() - t0, 1),
    }
    (args.out_dir / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps({k_: summary[k_] for k_ in ('areas', 'widened_segments', 'band_cells_without_free_evidence',
                                                 'occ_cells_changed', 'usable')}, ensure_ascii=False, indent=1))
    print(f'[clearance] < inflation {len(tight)}（新たに {len(newly_tight)}）、< robot_radius {len(tight_r)}、'
          f'まとめて {len(site_rows)} か所')
    print(f'[完了] {summary["elapsed_s"]} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())

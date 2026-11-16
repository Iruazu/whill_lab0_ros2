#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_roadway_from_curbs.py — 縁石を点群から抽出し、走行可能域を自動で作る試作（#202）。

手塗りの白リスト（candidate_roadway_mask.png）の代わりになるかを確かめるための実験用。
既定の地図は変えない。出力は --out-dir（小さい確認用の画像と数値、Git 管理）と
--work-dir（合成した pgm などの大きい中間物、Git 管理外）に分けて置く。

手順:
  1. static.pcd から 5 cm 格子の地面高さ（各セルの下位百分位）と点数の面を作る。
     局所最小から --ground-band 以内の点だけを使い、植栽・壁の点が地面に混ざらないようにする。
  2. 地面の面を軽く平滑化し、--window 内の高さの幅（最大−最小）が
     [--curb-dz-min, --curb-dz-max] で、かつ勾配が --curb-slope-min 以上のセルを縁石候補にする。
     構造物層（occupied_structure.png、地面から 0.1〜2.2 m）があるセルは植栽・壁として除く。
  3. 軌跡（--traj-stride ごと）のセルを種に、地面が見えていて縁石・構造物でないセルを
     4 近傍で領域成長する。軌跡から --max-dist より遠いセルには広げない（広場への漏れ止め）。
     最後に closing / opening で整える。
  4. 手塗り候補と比べ、IoU・precision・recall と差分の画像を出す（領域別も）。
  5. --compose を付けると、既存の v2 の合成（gen_auto_free_mask.py → compose_occupancy.py）を
     自動の roadway で回し、map_route_check.py で配車4地点のつながりを確かめる。

v2 の Rule 1（出力に形態演算をしない）は機械レイヤ（障害物）の規則で、ここで作るのは
人が塗っていた白リストの代わりなので、closing / opening を使う（白リストは手でも太く塗られている）。
"""

import argparse
import json
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'

# 領域別の比較に使う箱（map 座標 [m]、x0, x1, y0, y1）。
# start_road は発進点〜bldg3 の区間で、#202 の発端（段差層が縁石を線として取れていない）の場所。
# seimon は軌跡から 7.2 m 離れた配車地点で、旧地図では直接の手塗りで FREE にした所。
# north_road は y≈160 m の東西の道（手塗りの最北辺）。
REGIONS = {
    'start_road': (-10.0, 70.0, -15.0, 15.0),
    'seimon': (-100.0, -75.0, -20.0, 5.0),
    'north_road': (-125.0, 75.0, 145.0, 175.0),
}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pcd', type=pathlib.Path, default=D / 'static.pcd',
                   help='正本の static.pcd（読むだけ）。worktree には無いので、無ければ main 側を探す。')
    p.add_argument('--ground-npz', type=pathlib.Path, default=None,
                   help='PCD の代わりに、map_ground_raster_from_bag.py の ground_raster.npz（bag の'
                        'スキャンを軌跡で重ねた密な地面の面）を使う。下位百分位（z_p）と、セルの最小から'
                        '度数分布の範囲内の点数（count_low）をそのまま使い、局所最小からの帯（--ground-band）'
                        'は z_p で判定する。格子と百分位が v2_layers.yaml・--ground-percentile と同じことを確かめる。')
    p.add_argument('--traj', type=pathlib.Path, default=D / 'traj_lidar.txt')
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml',
                   help='格子（origin・解像度・大きさ）と構造物層・下絵の場所。')
    p.add_argument('--hand-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_roadway_mask.png',
                   help='比べる相手の手塗りの白リスト。')
    p.add_argument('--waypoints', type=pathlib.Path, default=D / 'waypoints.yaml')
    p.add_argument('--out-dir', type=pathlib.Path, default=D / 'v2/experiments/202',
                   help='Git 管理する小さい出力（1/4 の確認画像、マスク、数値）。')
    p.add_argument('--work-dir', type=pathlib.Path,
                   default=pathlib.Path.home() / 'bags/m8-experiments/202',
                   help='大きい中間物（合成した pgm、原寸の差分など）。Git 管理外。')

    # --- 1. 地面の面 ---
    p.add_argument('--resolution', type=float, default=None,
                   help='既定は v2_layers.yaml の値（0.05 m）。格子を揃えないと手塗りと比べられない。')
    p.add_argument('--chunk-points', type=int, default=2_000_000,
                   help='PCD を読むときの 1 回の点数。7.6 M 点でも中間配列を数百 MB に抑える。')
    p.add_argument('--local-min-radius', type=float, default=0.5,
                   help='局所最小を取る半径 [m]。縁石の上（歩道）のセルが、隣の車道の高さに'
                        '引っぱられすぎない大きさ（縁石の段差を消さない）にとどめる。')
    p.add_argument('--ground-band', type=float, default=0.5,
                   help='局所最小からこの高さ [m] 以内の点だけで地面を求める。'
                        '植栽・車・壁の点を百分位に混ぜないため。縁石（≤0.25 m）は残る。')
    p.add_argument('--ground-percentile', type=float, default=5.0,
                   help='地面高さに使う下位百分位 [%%]。v2_layers.yaml の ground_percentile と同じ。')
    p.add_argument('--min-points', type=int, default=1,
                   help='地面が見えたとみなす帯内の点数。static.pcd は 0.1 m のボクセルの中心'
                        '（x, y, z とも 0.05 + 0.1 k の値だけ。2026-11-16 に全点で確認）なので、'
                        '5 cm のセルには高さ 10 cm ごとに高々 1 点しか入らず、地面のセルの大半は'
                        '1 点。v2 の 3 にすると地面の 7%% しか残らない（118 k / 1.71 M セル）。')
    p.add_argument('--hole-fill', type=float, default=0.3,
                   help='点の無いセルを近い値で埋める最大距離 [m]。static.pcd は 0.1 m の'
                        'ボクセルの中心で、軌跡から離れた路面では VLP-16 のリングの間隔が'
                        '0.5 m 前後に開く。0.15 では軌跡から 3〜4 m 先で地面が途切れ、'
                        'seimon・bldg3 が roadway に入らなかった（campus-m8 README §3「縁石からの自動抽出の試作」）。'
                        'これより広い穴（建物の中・死角）は「見えていない」のまま残す。')
    p.add_argument('--smooth-ksize', type=int, default=5,
                   help='地面の面の中央値フィルタの大きさ（セル、3 か 5）。中央値は段の縁を'
                        'なまらせずに 1 セルの外れ値を消すので、縁石の検出に向く。')

    # --- 2. 縁石 ---
    p.add_argument('--window', type=float, default=0.5,
                   help='高さの幅（最大−最小）を見る窓の直径 [m]。#202 の「0.5 m 以内」。')
    p.add_argument('--curb-dz-min', type=float, default=0.08,
                   help='縁石とみなす高さ差の下限 [m]。WHILL CR2 の乗り越え（5 cm 程度）より'
                        '上で、路面の凹凸・百分位の揺れ（2〜3 cm）と分けられる値。')
    p.add_argument('--curb-dz-max', type=float, default=0.25,
                   help='縁石とみなす高さ差の上限 [m]。これより大きい段は「高い段」として'
                        '縁石の線には入れないが、--block-high-steps で通れない所に入れる。')
    p.add_argument('--trend-window', type=float, default=0.8,
                   help='量子化の揺れをならす平均の窓 [m]（一辺）。z が 10 cm 刻みなので、路面が'
                        '量子化の境目にあると、隣り合うセルが 10 cm 上下にばらばらに揺れ、'
                        '縁石と同じ 1 段に見える（2026-11-16 に発進点〜bldg3 で確認）。この窓の'
                        '平均では揺れは打ち消し合い、縁石は段のまま残る。0.5 m の窓で見た平均の'
                        '面の幅は、10 cm の縁石で約 0.1×0.5/窓。1.0 m だと 0.048 m で'
                        '--trend-dz-min 0.05 を下回り、1 段の縁石が一つも出なかった'
                        '（tests/test_roadway_curbs.py）。0.8 m で 0.059 m。'
                        'なお、坂の等高線（2%% の坂なら 5 m ごとの 10 cm の段）は、窓より段の'
                        '間隔が広いので落とせない（campus-m8 README §3「縁石からの自動抽出の試作」の「効かなかったこと」）。')
    p.add_argument('--trend-dz-min', type=float, default=0.05,
                   help='平均の面で --window 内に要る高さの幅 [m]。現物の揺れは平均しても'
                        '0.04〜0.05 m 残る所があり、1 段（10 cm）の縁石の 0.059 m と近い。'
                        '0.04 にすると道の中に偽の縁石が増え、0.06 にすると縁石の線が減って'
                        '外へ漏れる（campus-m8 README §3「縁石からの自動抽出の試作」 の掃引）。')
    p.add_argument('--curb-slope-min', type=float, default=0.4,
                   help='縁石の面の勾配の下限（Sobel、高さ/距離）。窓の幅の条件だけでは段の'
                        '両側 0.25 m の帯になるので、段の面そのものに細らせる。10 cm の段は'
                        '理想的に 1.0、1/12 のスロープは 0.08。')
    p.add_argument('--no-block-high-steps', dest='block_high_steps', action='store_false',
                   help='既定では窓内の高さ差が --curb-dz-max を超えるセル（高い段、急な斜面）'
                        'も領域成長で通さない。外すと縁石の帯だけで区切る。')
    p.add_argument('--barrier-dilate', type=float, default=0.05,
                   help='縁石・構造物の壁を太らせる半径 [m]。点の疎らさで線が 1 セル途切れた'
                        '所から漏れないようにする（4 近傍の成長と合わせて斜めの隙間も塞ぐ）。')

    # --- 3. 領域成長 ---
    p.add_argument('--traj-stride', type=float, default=1.0,
                   help='種にする軌跡点の間隔 [m]。v2 の traj_stride と同じ。')
    p.add_argument('--seed-snap', type=float, default=1.0,
                   help='軌跡のセルが通れないとき、この距離 [m] 以内の通れるセルへ種を寄せる。'
                        'LiDAR の直下は見えない（死角）ので、軌跡の真下に地面の点が無いことがある。')
    p.add_argument('--max-dist', type=float, default=8.0,
                   help='軌跡からの最大距離 [m]。縁石の無い広場・駐車場へ広がりすぎないための'
                        '上限。seimon は最寄りの軌跡から 7.2 m なので、それより大きくする。'
                        '12 m（#202 の例）より 8 m のほうが precision が上がり、recall は'
                        '変わらなかった（campus-m8 README §3「縁石からの自動抽出の試作」 の掃引）。')
    p.add_argument('--gap-radius', type=float, default=0.25,
                   help='通れるセルをこの半径 [m] で削ってから成長させ、成長後に同じ半径だけ'
                        '通れるセルの中で戻す。縁石の線は点の疎らさ・量子化で途切れ途切れなので、'
                        '幅 2r より狭い切れ目からは漏れないようにする。車体の半幅（0.35 m、'
                        'map_route_check の --radius）より小さく取る。0.35 では bldg3 の前が'
                        '偽の縁石で切れ、0.5 以上では種の 2 割以上が通れなくなった（campus-m8 README §3「縁石からの自動抽出の試作」）。'
                        '0 で無効。')
    p.add_argument('--close-radius', type=float, default=0.6,
                   help='成長後の closing の半径 [m]。路面の塩（小さい構造物）と、量子化の揺れで'
                        '残った偽の縁石で空いた穴を塞ぐ。0.3 では道の中に穴が残った。')
    p.add_argument('--open-radius', type=float, default=0.2,
                   help='closing 後の opening の半径 [m]。0.4 m より細い漏れ（車体 0.7 m は'
                        '通れない幅）を切る。')
    p.add_argument('--fill-holes-area', type=float, default=2.0,
                   help='この面積 [m²] より小さい内側の穴を埋める。')

    # --- 5. 合成 ---
    p.add_argument('--compose', action='store_true',
                   help='v2 の合成と map_route_check.py も回す（pgm は --work-dir へ）。')
    p.add_argument('--hand-free-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_free_merged.png',
                   help='合成の変種 B で使う手塗りの free（塩の消去）。')
    p.add_argument('--hand-roadway-merged', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_roadway_merged.png',
                   help='合成の変種 C（自動 ∪ 手塗り）で使う手塗りの roadway。')
    p.add_argument('--keepout-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_keepout_mask.png')
    return p.parse_args(argv)


# ---------------------------------------------------------------- 入出力

def resolve_pcd(path):
    """worktree には PCD が無い（.gitignore）。同じ相対パスを main 側で探す。"""
    if path.is_file():
        return path
    common = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--path-format=absolute',
                             '--git-common-dir'], capture_output=True, text=True).stdout.strip()
    if common:
        alt = pathlib.Path(common).parent / path.resolve().relative_to(REPO)
        if alt.is_file():
            return alt
    raise SystemExit(f'PCD が見つからない: {path}')


def open_pcd_xyz(path):
    """binary の x y z float32 の PCD を memmap で開く（全点を一度に複製しない）。"""
    header = {}
    with path.open('rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise SystemExit(f'{path}: DATA 行が無い')
            key, _, val = line.decode('ascii', 'replace').strip().partition(' ')
            header[key] = val
            if key == 'DATA':
                offset = f.tell()
                break
    if header['DATA'] != 'binary' or header['FIELDS'].split()[:3] != ['x', 'y', 'z']:
        raise SystemExit(f'{path}: binary の x y z だけを想定（{header}）')
    n_fields = len(header['FIELDS'].split())
    if header['TYPE'].split() != ['F'] * n_fields or header['SIZE'].split() != ['4'] * n_fields:
        raise SystemExit(f'{path}: float32 の欄だけを想定')
    n = int(header['POINTS'])
    return np.memmap(path, dtype=np.float32, mode='r', offset=offset, shape=(n, n_fields))


def load_grid(layers_yaml):
    doc = yaml.safe_load(layers_yaml.read_text())
    g = doc['grid']
    return doc, int(g['width']), int(g['height']), float(g['resolution']), \
        float(g['origin'][0]), float(g['origin'][1])


def load_mask(path, hw):
    """compose_occupancy.load_sidecar_mask と同じ解釈（白・不透明が塗り）。"""
    im = Image.open(path)
    gray = np.array(im.convert('L'))
    m = gray >= 128
    if im.mode == 'RGBA':
        m &= np.array(im.getchannel('A')) > 0
    if m.shape != hw:
        raise SystemExit(f'{path}: 形 {m.shape} が格子 {hw} と違う')
    return m


def save_mask_rgba(path, mask):
    """手塗りの候補と同じ形式（白・不透明 / 透明）で書く。compose_occupancy にそのまま渡せる。"""
    H, W = mask.shape
    rgba = np.zeros((H, W, 4), np.uint8)
    rgba[mask] = 255
    Image.fromarray(rgba, 'RGBA').save(path, optimize=True)


def load_traj_xy(path):
    a = np.loadtxt(path, usecols=(1, 2), comments='#')
    return a


def downsample_xy(xy, stride):
    kept = [xy[0]]
    for q in xy[1:]:
        if np.hypot(*(q - kept[-1])) >= stride:
            kept.append(q)
    return np.asarray(kept)


def xy_to_rc(xy, H, res, ox, oy):
    """pcd_to_occupancy_v2 と同じ向き（画像の 0 行が map の y 最大）。"""
    c = np.floor((xy[:, 0] - ox) / res).astype(np.int64)
    r = (H - 1 - np.floor((xy[:, 1] - oy) / res)).astype(np.int64)
    return r, c


def disk(radius_cells):
    k = 2 * int(radius_cells) + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))


# ---------------------------------------------------------------- 1. 地面の面

def ground_raster(pts, H, W, res, ox, oy, args):
    """各セルの地面高さ（float32、無いセルは NaN）と帯内の点数（uint16）を返す。"""
    n_cells = H * W
    cell_min = np.full(n_cells, np.inf, np.float32)
    cells, zs = [], []
    # 1 回目：セル番号と z を塊ごとに作り、セルごとの最小を取る。
    for s in range(0, len(pts), args.chunk_points):
        blk = np.asarray(pts[s:s + args.chunk_points, :3])
        r, c = xy_to_rc(blk[:, :2].astype(np.float64), H, res, ox, oy)
        ok = (r >= 0) & (r < H) & (c >= 0) & (c < W) & np.isfinite(blk[:, 2])
        cell = (r[ok] * W + c[ok]).astype(np.int32)   # 6640×6295 < 2^31
        z = blk[ok, 2].astype(np.float32)
        np.minimum.at(cell_min, cell, z)
        cells.append(cell)
        zs.append(z)
    cell = np.concatenate(cells)
    z = np.concatenate(zs)
    del cells, zs

    # 局所最小：セルの最小を円板で収縮（inf は「点無し」として最小に影響しない）。
    rr = max(1, int(round(args.local_min_radius / res)))
    local_min = cv2.erode(cell_min.reshape(H, W), disk(rr),
                          borderType=cv2.BORDER_CONSTANT, borderValue=float(np.inf)).ravel()
    keep = z <= local_min[cell] + args.ground_band
    cell, z = cell[keep], z[keep]
    del local_min, cell_min

    # 2 回目：セル→z の順に並べ、各セルの下位百分位を添字で取る（pandas の quantile より軽い）。
    order = np.lexsort((z, cell))
    cell, z = cell[order], z[order]
    del order
    uniq, start, cnt = np.unique(cell, return_index=True, return_counts=True)
    q = args.ground_percentile / 100.0
    pick = start + np.floor(q * (cnt - 1)).astype(np.int64)
    ground = np.full(n_cells, np.nan, np.float32)
    ground[uniq] = z[pick]
    count = np.zeros(n_cells, np.uint16)
    count[uniq] = np.minimum(cnt, 65535).astype(np.uint16)
    n_in = int(len(z))
    del cell, z
    return ground.reshape(H, W), count.reshape(H, W), n_in


def ground_raster_from_npz(path, H, W, res, ox, oy, args):
    """密な地面の面（npz）から、ground_raster() と同じ形の (ground, count, n_in) を作る。

    npz の z_p はセルの全点での下位百分位で、PCD の経路（局所最小から --ground-band 以内の点だけの
    百分位）とは母集団が違う。密な面では地面の点が多く、植栽・壁の点が混ざっても下位 5%% は
    ほぼ地面に残るので、帯は点ではなくセルの値（z_p）に掛ける：z_p が局所最小（z_min の円板の収縮）
    から --ground-band より高いセルは、地面が見えていないとみなす（建物の中・生垣の上など）。
    """
    with np.load(path) as f:
        meta = json.loads(str(f['meta']))
        zp, zmin, cnt = f['z_p'], f['z_min'], f['count_low']
    if (meta['width'], meta['height']) != (W, H) or abs(meta['resolution'] - res) > 1e-9 \
            or np.abs(np.array(meta['origin'][:2]) - [ox, oy]).max() > 1e-9:
        raise SystemExit(f'{path}: 格子 {meta} が v2_layers.yaml と違う')
    if abs(meta['percentile'] - args.ground_percentile) > 1e-9:
        raise SystemExit(f'{path}: 百分位 {meta["percentile"]} が --ground-percentile '
                         f'{args.ground_percentile} と違う（npz を作り直す）')
    rr = max(1, int(round(args.local_min_radius / res)))
    local_min = cv2.erode(np.where(np.isfinite(zmin), zmin, np.inf).astype(np.float32), disk(rr),
                          borderType=cv2.BORDER_CONSTANT, borderValue=float(np.inf))
    ok = np.isfinite(zp) & (zp <= local_min + args.ground_band)
    ground = np.where(ok, zp, np.nan).astype(np.float32)
    count = np.where(ok, np.minimum(cnt, 65535), 0).astype(np.uint16)
    return ground, count, int(cnt[ok].sum())


def fill_and_smooth(ground, count, res, args):
    """点の少ないセルを近い値で埋め、中央値で平滑化する。

    seen  : 帯内の点が --min-points 以上のセル。
    known : seen と、seen から --hole-fill 以内の埋めたセル（領域成長はここだけ通る）。
    """
    seen = count >= args.min_points
    dist, (ir, ic) = ndimage.distance_transform_edt(~seen, return_indices=True)
    filled = ground[ir, ic]
    del ir, ic
    known = dist * res <= args.hole_fill
    # 中央値は未知のセルにも最近傍の値が入った面で取り、後で known で切る
    # （NaN を入れると cv2.medianBlur が壊れる）。
    k = args.smooth_ksize if args.smooth_ksize in (3, 5) else 5
    smooth = cv2.medianBlur(filled.astype(np.float32), k)
    return smooth, seen, known


# ---------------------------------------------------------------- 2. 縁石

def detect_curbs(ground_s, known, structure, res, window=0.5, dz_min=0.08, dz_max=0.25,
                 slope_min=0.4, trend_window=1.0, trend_dz_min=0.04):
    """縁石候補と高い段を返す（どちらも bool の 2 次元）。

    ground_s  : 平滑化した地面の面（全セルに値がある。未知は known で外す）。
    known     : 地面が見えたセル。
    structure : 構造物層（植栽・壁）。縁石の線からは除く。
    窓内の高さの幅だけだと段の両側に帯ができるので、勾配（Sobel）で段の面に細らせる。
    z が 10 cm 刻みの点群では、路面が量子化の境目にあるとセルごとに 1 段（10 cm）揺れ、
    縁石と区別がつかない。そこで、known だけで取った --trend-window の平均の面でも --window 内に
    trend_dz_min の幅があることを要る（揺れは平均で打ち消し合い、縁石は段のまま幅が残る）。
    坂の等高線（間隔が窓より広い 1 段）は局所的には縁石と同じ形なので、これでは落ちない。
    trend_dz_min<=0 で無効。
    """
    r = max(1, int(round(window / 2 / res)))
    kern = disk(r)
    zmax = cv2.dilate(ground_s, kern)
    zmin = cv2.erode(ground_s, kern)
    # 未知のセルの値（最近傍で埋めた値）を窓に入れないよう、known だけで幅を取り直す。
    big = np.float32(1e6)
    zmax_k = cv2.dilate(np.where(known, ground_s, -big).astype(np.float32), kern)
    zmin_k = cv2.erode(np.where(known, ground_s, big).astype(np.float32), kern)
    zr = np.minimum(zmax, zmax_k) - np.maximum(zmin, zmin_k)
    gx = cv2.Sobel(ground_s, cv2.CV_32F, 1, 0, ksize=3) / (8 * res)
    gy = cv2.Sobel(ground_s, cv2.CV_32F, 0, 1, ksize=3) / (8 * res)
    slope = np.hypot(gx, gy)
    steep = known & (slope >= slope_min)
    if trend_dz_min > 0:
        tk = max(3, int(round(trend_window / res)) | 1)
        kf = known.astype(np.float32)
        num = cv2.boxFilter(np.where(known, ground_s, 0).astype(np.float32), -1, (tk, tk),
                            normalize=False)
        den = cv2.boxFilter(kf, -1, (tk, tk), normalize=False)
        trend = np.where(den > 0, num / np.maximum(den, 1), 0).astype(np.float32)
        tr = cv2.dilate(np.where(known, trend, -big).astype(np.float32), kern) - \
            cv2.erode(np.where(known, trend, big).astype(np.float32), kern)
        steep &= tr >= trend_dz_min
    curb = steep & (zr >= dz_min) & (zr <= dz_max) & ~structure
    high = steep & (zr > dz_max)
    return curb, high


# ---------------------------------------------------------------- 3. 領域成長

def grow_roadway(passable, seeds_rc, traj_rc, res, args):
    H, W = passable.shape
    tmask = np.ones((H, W), bool)
    ok = (traj_rc[0] >= 0) & (traj_rc[0] < H) & (traj_rc[1] >= 0) & (traj_rc[1] < W)
    tmask[traj_rc[0][ok], traj_rc[1][ok]] = False
    dist = ndimage.distance_transform_edt(tmask).astype(np.float32) * res
    allowed = passable & (dist <= args.max_dist)
    gr = int(round(args.gap_radius / res))
    if gr > 0:
        # 削った通れる域で成長する（切れ目 < 2r を越えない）。戻すのは後で。
        core = cv2.erode(allowed.astype(np.uint8), disk(gr)).astype(bool)
    else:
        core = allowed
    # 4 近傍：縁石の線の斜めの隙間を越えないため。
    lab, _ = ndimage.label(core)
    sr, sc = seeds_rc
    ok = (sr >= 0) & (sr < H) & (sc >= 0) & (sc < W)
    sr, sc = sr[ok].copy(), sc[ok].copy()
    # 通れないセルの種を、--seed-snap 以内で最も近い通れるセルへ寄せる。
    snap = int(round(args.seed_snap / res))
    for i in np.nonzero(~core[sr, sc])[0]:
        r0, c0 = max(0, sr[i] - snap), max(0, sc[i] - snap)
        win = core[r0:sr[i] + snap + 1, c0:sc[i] + snap + 1]
        ys, xs = np.nonzero(win)
        if len(ys):
            j = int(np.argmin(np.hypot(ys + r0 - sr[i], xs + c0 - sc[i])))
            if np.hypot(ys[j] + r0 - sr[i], xs[j] + c0 - sc[i]) <= snap:
                sr[i], sc[i] = ys[j] + r0, xs[j] + c0
    seed_labels = lab[sr, sc]
    n_seed_hit = int((seed_labels > 0).sum())
    keep_ids = np.unique(seed_labels[seed_labels > 0])
    grown = np.isin(lab, keep_ids)
    del lab
    if gr > 0:
        # 削った分を戻す：成長した域を r だけ太らせ、元の通れる域に限る。
        grown = cv2.dilate(grown.astype(np.uint8), disk(gr)).astype(bool) & allowed

    m = grown.astype(np.uint8)
    if args.close_radius > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, disk(round(args.close_radius / res)))
    if args.open_radius > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, disk(round(args.open_radius / res)))
    road = m.astype(bool)
    # 小さい内側の穴を埋める（外側の背景と繋がらない背景の成分で、面積が小さいもの）。
    holes_lab, nh = ndimage.label(~road)
    if nh:
        area = np.bincount(holes_lab.ravel())
        border = np.unique(np.concatenate([holes_lab[0], holes_lab[-1],
                                           holes_lab[:, 0], holes_lab[:, -1]]))
        small = area * res * res < args.fill_holes_area
        small[0] = False
        small[border] = False
        road |= small[holes_lab]
    del holes_lab
    # opening で切れた小片を落とし、種を含む成分だけ残す。
    lab, _ = ndimage.label(road)
    ids = np.unique(lab[sr, sc])
    road = np.isin(lab, ids[ids > 0])
    return road, n_seed_hit, len(sr), dist


# ---------------------------------------------------------------- 4. 比較

def compare(auto, hand):
    tp = int((auto & hand).sum())
    a, h = int(auto.sum()), int(hand.sum())
    union = a + h - tp
    return {'auto_cells': a, 'hand_cells': h, 'both_cells': tp,
            'iou': round(tp / union, 4) if union else None,
            'precision': round(tp / a, 4) if a else None,
            'recall': round(tp / h, 4) if h else None}


def region_slice(box, H, res, ox, oy):
    x0, x1, y0, y1 = box
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
    r0, r1 = int(H - 1 - (y1 - oy) / res), int(H - 1 - (y0 - oy) / res)
    return slice(max(0, r0), r1 + 1), slice(max(0, c0), c1 + 1)


# ---------------------------------------------------------------- 6. 確認画像

def shrink_rgb(rgb, f):
    H, W = rgb.shape[:2]
    return np.array(Image.fromarray(rgb).resize((W // f, H // f), Image.BOX))


def shrink_any(mask, f):
    """1/f にするとき、ブロック内に 1 セルでもあれば立てる（細い線を消さない）。"""
    H, W = mask.shape
    h, w = H // f, W // f
    return mask[:h * f, :w * f].reshape(h, f, w, f).any(axis=(1, 3))


def save_png(path, rgb):
    Image.fromarray(rgb).save(path, optimize=True)
    return path.stat().st_size


def review_images(out_dir, under_max, hill, curb, high, road, hand, traj_rc,
                  start_box, H, res, ox, oy, f=4):
    sizes = {}
    # (a) 縁石の候補を最大高さの下絵に重ねる（1/4）。
    base = (shrink_rgb(under_max, f).astype(np.float32) * 0.6).astype(np.uint8)
    c4, h4 = shrink_any(curb, f), shrink_any(high, f)
    # 高い段は 1/4 にすると面のように広がって縁石の線を隠すので、暗い紫で下に敷く。
    base[h4] = (110, 0, 110)
    base[c4] = (255, 40, 40)
    sizes['curbs_over_maxheight_q.png'] = save_png(out_dir / 'curbs_over_maxheight_q.png', base)

    # (b) 自動と手塗りの重ね（1/4）：両方=白、自動だけ=赤、手塗りだけ=青。
    g = shrink_rgb(hill, f)[..., 0].astype(np.float32) * 0.35
    img = np.stack([g, g, g], -1).astype(np.uint8)
    a4 = shrink_rgb((road * 255).astype(np.uint8), f) >= 128
    h4m = shrink_rgb((hand * 255).astype(np.uint8), f) >= 128
    img[a4 & h4m] = (230, 230, 230)
    img[a4 & ~h4m] = (255, 50, 50)
    img[~a4 & h4m] = (60, 120, 255)
    sizes['roadway_auto_vs_hand_q.png'] = save_png(out_dir / 'roadway_auto_vs_hand_q.png', img)

    # (c) 発進点まわりの原寸の切り出し：最大高さの下絵に、自動の roadway を緑で薄く、
    # 手塗りの縁を青、縁石を赤、高い段を紫、軌跡を黄で重ねる。
    rs, cs = region_slice(start_box, H, res, ox, oy)
    crop = under_max[rs, cs].astype(np.float32) * 0.7
    rc = road[rs, cs]
    crop[rc] = crop[rc] * 0.5 + np.array([0, 160, 60]) * 0.5
    crop = crop.astype(np.uint8)
    hc = hand[rs, cs]
    edge = hc & ~cv2.erode(hc.astype(np.uint8), disk(1)).astype(bool)
    crop[edge] = (60, 140, 255)
    crop[high[rs, cs]] = (255, 0, 255)
    crop[curb[rs, cs]] = (255, 30, 30)
    tr, tc = traj_rc[0] - rs.start, traj_rc[1] - cs.start
    ok = (tr >= 0) & (tr < crop.shape[0]) & (tc >= 0) & (tc < crop.shape[1])
    crop[tr[ok], tc[ok]] = (255, 230, 0)
    sizes['start_crop_1to1.png'] = save_png(out_dir / 'start_crop_1to1.png', crop)
    return sizes


# ---------------------------------------------------------------- 5. 合成と経路

def run(cmd):
    print('  $', ' '.join(str(c) for c in cmd))
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True)


def compose_and_check(variant, roadway, free_mask, args, layers_yaml, work):
    """README §3 と同じ順（gen_auto_free_mask → compose_occupancy）で合成し、経路を確かめる。"""
    vd = work / variant
    vd.mkdir(parents=True, exist_ok=True)
    s = REPO / 'scripts'
    r = run([sys.executable, s / 'gen_auto_free_mask.py', '--layers-yaml', layers_yaml,
             '--roadway-mask', roadway, '--output-free-mask-auto', vd / 'free_mask_auto.png',
             '--output-salt-csv', vd / 'salt.csv'])
    if r.returncode:
        raise SystemExit(f'gen_auto_free_mask 失敗: {r.stderr[-2000:]}')
    cmd = [sys.executable, s / 'compose_occupancy.py', '--layers-yaml', layers_yaml,
           '--roadway-mask', roadway, '--free-mask-auto', vd / 'free_mask_auto.png',
           '--keepout-mask', args.keepout_mask, '--output-pgm', vd / 'final.pgm']
    if free_mask is not None:
        cmd += ['--free-mask', free_mask]
    r = run(cmd)
    if r.returncode:
        raise SystemExit(f'compose_occupancy 失敗: {r.stderr[-2000:]}')
    r = run([sys.executable, s / 'map_route_check.py', vd / 'final.yaml', args.waypoints])
    res = json.loads(r.stdout)
    img = np.array(Image.open(vd / 'final.pgm'))
    res['occ_free_unknown_pct'] = [round(100 * float((img == v).mean()), 2) for v in (0, 254, 205)]
    res['map'] = str(vd / 'final.yaml')
    return res


# ---------------------------------------------------------------- main

def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    doc, W, H, res, ox, oy = load_grid(args.layers_yaml)
    if args.resolution is not None and abs(args.resolution - res) > 1e-9:
        raise SystemExit('--resolution は v2_layers.yaml と同じにする（手塗りと格子を揃えるため）')
    layers_dir = args.layers_yaml.parent
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.work_dir.mkdir(parents=True, exist_ok=True)
    timing = {}

    if args.ground_npz is not None:
        pcd_path = args.ground_npz
        print(f'[1] 地面の面: {pcd_path}（密な面の npz）')
        ground, count, n_band = ground_raster_from_npz(pcd_path, H, W, res, ox, oy, args)
    else:
        pcd_path = resolve_pcd(args.pcd)
        pts = open_pcd_xyz(pcd_path)
        print(f'[1] 地面の面: {pcd_path}（{len(pts):,} 点）')
        ground, count, n_band = ground_raster(pts, H, W, res, ox, oy, args)
        del pts
    smooth, seen, known = fill_and_smooth(ground, count, res, args)
    del ground
    timing['ground_s'] = round(time.time() - t0, 1)
    print(f'    帯内の点 {n_band:,}、地面が見えたセル {int(seen.sum()):,}、'
          f'埋めた後 {int(known.sum()):,}（{timing["ground_s"]} s）')

    t1 = time.time()
    structure = load_mask_alpha(layers_dir / doc['layers']['occupied_structure'], (H, W))
    curb, high = detect_curbs(smooth, known, structure, res, args.window,
                              args.curb_dz_min, args.curb_dz_max, args.curb_slope_min,
                              args.trend_window, args.trend_dz_min)
    timing['curb_s'] = round(time.time() - t1, 1)
    print(f'[2] 縁石候補 {int(curb.sum()):,} セル、高い段 {int(high.sum()):,} セル、'
          f'構造物 {int(structure.sum()):,} セル（{timing["curb_s"]} s）')

    t1 = time.time()
    traj = load_traj_xy(args.traj)
    traj_rc = xy_to_rc(traj, H, res, ox, oy)
    seeds_rc = xy_to_rc(downsample_xy(traj, args.traj_stride), H, res, ox, oy)
    barrier = curb | structure
    if args.block_high_steps:
        barrier |= high
    bd = int(round(args.barrier_dilate / res))
    if bd > 0:
        barrier = cv2.dilate(barrier.astype(np.uint8), disk(bd)).astype(bool)
    passable = known & ~barrier
    road, n_seed_hit, n_seed, dist = grow_roadway(passable, seeds_rc, traj_rc, res, args)
    timing['grow_s'] = round(time.time() - t1, 1)
    print(f'[3] roadway_auto {int(road.sum()):,} セル、種 {n_seed_hit}/{n_seed} が通れるセル'
          f'（{timing["grow_s"]} s）')

    save_mask_rgba(args.out_dir / 'roadway_auto.png', road)
    save_mask_rgba(args.out_dir / 'curb_mask.png', curb)

    hand = load_mask(args.hand_mask, (H, W))
    metrics = {'whole': compare(road, hand)}
    # 軌跡から --max-dist 以内の手塗りだけで見た recall。手塗りには走っていない道（中央の
    # 南北の道など）も入っていて、軌跡を種にする限りそこは原理的に取れないので分けて示す。
    near = hand & (dist <= args.max_dist)
    metrics['whole']['recall_near_traj'] = round(int((road & near).sum()) / max(1, int(near.sum())), 4)
    metrics['whole']['hand_cells_near_traj'] = int(near.sum())
    del dist
    for name, box in REGIONS.items():
        rs, cs = region_slice(box, H, res, ox, oy)
        metrics[name] = compare(road[rs, cs], hand[rs, cs]) | {'box_xy': list(box)}
    # 原寸の差分（自動だけ=赤、手塗りだけ=青）。大きいので work へ。
    diff = np.zeros((H, W, 3), np.uint8)
    diff[road & hand] = 255
    diff[road & ~hand] = (255, 0, 0)
    diff[~road & hand] = (0, 0, 255)
    Image.fromarray(diff).save(args.work_dir / 'diff_auto_vs_hand.png')
    del diff
    for k, v in metrics.items():
        print(f'[4] {k:11s} IoU {v["iou"]}  precision {v["precision"]}  recall {v["recall"]}')

    under_max = np.array(Image.open(layers_dir / doc['layers']['underlay_maxheight']).convert('RGB'))
    hill = np.array(Image.open(layers_dir / doc['layers']['underlay_hillshade']).convert('RGB'))
    sizes = review_images(args.out_dir, under_max, hill, curb, high, road, hand, traj_rc,
                          REGIONS['start_road'], H, res, ox, oy)
    del under_max, hill

    routes = {}
    if args.compose:
        t1 = time.time()
        auto_png = args.out_dir / 'roadway_auto.png'
        # A：完全に自動（手塗りの free を使わない）。
        routes['A_auto'] = compose_and_check('A_auto', auto_png, None, args,
                                             args.layers_yaml, args.work_dir)
        # B：自動の roadway ＋ 手塗りの free（塩の消去と直接の拡幅）。roadway だけを差し替えた比較。
        routes['B_auto_handfree'] = compose_and_check('B_auto_handfree', auto_png,
                                                      args.hand_free_mask, args,
                                                      args.layers_yaml, args.work_dir)
        # C：自動 ∪ 手塗りの roadway ＋ 手塗りの free（併用案）。
        union_png = args.work_dir / 'roadway_union.png'
        save_mask_rgba(union_png, road | load_mask(args.hand_roadway_merged, (H, W)))
        routes['C_union_handfree'] = compose_and_check('C_union_handfree', union_png,
                                                       args.hand_free_mask, args,
                                                       args.layers_yaml, args.work_dir)
        timing['compose_s'] = round(time.time() - t1, 1)
        for k, v in routes.items():
            comps = [w['component'] for w in v['waypoints']]
            print(f'[5] {k:17s} all_connected={v["all_connected"]} 成分={comps} '
                  f'OCC/FREE/UNK %={v["occ_free_unknown_pct"]}')

    timing['total_s'] = round(time.time() - t0, 1)
    params = {k: (short(v) if isinstance(v, pathlib.Path) else v) for k, v in vars(args).items()}
    params['pcd_resolved'] = short(pcd_path)   # --ground-npz のときはその npz
    for v in routes.values():
        v['map'] = short(pathlib.Path(v['map']))
    summary = {'params': params, 'metrics': metrics, 'routes': routes, 'timing': timing,
               'cells': {'seen': int(seen.sum()), 'known': int(known.sum()),
                         'curb': int(curb.sum()), 'high_step': int(high.sum()),
                         'structure': int(structure.sum()), 'roadway_auto': int(road.sum())},
               'seeds': {'passable': n_seed_hit, 'total': n_seed},
               'review_png_bytes': sizes}
    (args.out_dir / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'所要 {timing["total_s"]} s。summary: {args.out_dir / "summary.json"}')
    return 0


def short(path):
    """summary.json に絶対パス（worktree・ホーム）を残さない。repo 内は相対、ホームは ~ にする。"""
    path = pathlib.Path(path).resolve()
    for base, pre in ((REPO, ''), (pathlib.Path.home(), '~/')):
        try:
            return pre + str(path.relative_to(base))
        except ValueError:
            pass
    return str(path)


def load_mask_alpha(path, hw):
    """機械レイヤ（RGBA、不透明が該当）を bool にする。"""
    a = np.array(Image.open(path).getchannel('A')) > 0
    if a.shape != hw:
        raise SystemExit(f'{path}: 形 {a.shape} が格子 {hw} と違う')
    return a


if __name__ == '__main__':
    sys.exit(main())

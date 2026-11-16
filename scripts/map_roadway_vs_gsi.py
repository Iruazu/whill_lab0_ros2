#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_roadway_vs_gsi.py — 走行可能域のマスクを GSI の道路の面と比べ、舗装の面積を数える（#202、202c）。

#205（scripts/map_georef_gsi.py）の report と同じ道路の面（GSI の道路縁と中心線から作る。
点群を使わない独立な参照）を作り、複数のマスク（手塗り・自動）について IoU・precision・recall を
同じ区切り（全体、軌跡から 8 m 以内、発進点〜bldg3、seimon、北の道）で出す。
評価の格子は #205 と同じ 0.1 m（v2 の 5 cm を 2×2 の平均 ≥ 0.5 で縮める）。

さらに、反射強度（map_ground_raster_from_bag.py の npz）で「舗装」（地面の強度が閾値未満）を
決め、軌跡の近くで「舗装だが手塗りに入っていない」面積と、名前を付けた箱（斜めの舗装路、
左上の広場）で各マスク・舗装・GSI の占める割合を数える。舗装の判定は
map_roadway_from_curbs.intensity_gate と同じ（同じ引数）で、壁の逆が舗装。

map_georef_gsi.py と georef.yaml は #205（PR #207）で main に入った。既定でそれを読む
（読むだけ。#205 の出力は変えない）。202c の記録の数値は、#205 のブランチから取り出した
同じ内容の写し（main と cmp で一致）で出した。
道路の面は --road-cache に保存し、次からは作り直さない（タイル・変換が変わったら消す）。
"""

import argparse
import importlib.util
import json
import pathlib
import sys
import time

import numpy as np
from PIL import Image
from scipy import ndimage

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'
sys.path.insert(0, str(REPO / 'scripts'))

import map_roadway_from_curbs as mr  # noqa: E402

# 名前を付けた箱（map 座標 [m]、x0, x1, y0, y1）。202c の目視で漏れが残った 2 か所。
# diagonal_path：発進点の南東、植栽の島の間を南へ下りる斜めの舗装路。
# plaza_topleft：発進点の北西の、段の無い舗装の広場。
NAMED_BOXES = {
    'diagonal_path': (30.0, 45.0, -8.0, -2.0),
    'plaza_topleft': (-10.0, -2.0, 3.0, 15.0),
}
GSI_BOXES = {'start_road': (-10, 70, -15, 15), 'seimon': (-100, -75, -20, 5),
             'north_road': (-125, 75, 145, 175)}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--gsi-module', type=pathlib.Path, default=REPO / 'scripts/map_georef_gsi.py')
    p.add_argument('--georef-yaml', type=pathlib.Path, default=D / 'georef.yaml')
    p.add_argument('--raw-dir', type=pathlib.Path,
                   default=pathlib.Path.home() / 'bags/m8-experiments/205/raw')
    p.add_argument('--road-cache', type=pathlib.Path,
                   default=pathlib.Path.home() / 'bags/m8-experiments/202c/gsi_ref/gsi_road_area_k2.npz')
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml')
    p.add_argument('--traj', type=pathlib.Path, default=D / 'traj_lidar.txt')
    p.add_argument('--hand-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_roadway_mask.png')
    p.add_argument('--mask', action='append', default=[], metavar='NAME=PATH',
                   help='比べるマスク（白・不透明が塗り）。何度でも。手塗りは自動で入る。')
    p.add_argument('--overlay', action='append', default=[], metavar='NAME',
                   help='1/4 の GSI との重ね絵を作るマスクの名前。')
    p.add_argument('--ground-npz', type=pathlib.Path,
                   default=pathlib.Path.home() / 'bags/m8-experiments/202c/ground_raster.npz')
    p.add_argument('--paved-thresholds', type=float, nargs='+', default=[15.0, 20.0, 25.0])
    p.add_argument('--near-m', type=float, default=8.0, help='#202・#205 と同じ「軌跡の近く」の幅')
    p.add_argument('--metric-scale', type=int, default=2)
    p.add_argument('--gap-m', type=float, default=0.3)
    p.add_argument('--max-half-width-m', type=float, default=10.0)
    p.add_argument('--vector-zoom', type=int, default=16)
    p.add_argument('--out-dir', type=pathlib.Path, default=D / 'v2/experiments/202c/intensity_gate')
    p.add_argument('--json', type=pathlib.Path, default=None,
                   help='既定は --out-dir/gsi_and_paved.json')
    return p.parse_args(argv)


def load_module(path):
    spec = importlib.util.spec_from_file_location('map_georef_gsi', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def gsi_road(g, args, W, H, res, ox, oy):
    """#205 の cmd_report と同じ手順で GSI の道路の面（0.1 m）と道路縁・中心線を作る。"""
    if args.road_cache.is_file():
        with np.load(args.road_cache) as f:
            return f['road'], f['edges'], f['centers'], json.loads(str(f['info']))
    T = g.load_georef(args.georef_yaml)
    Tinv = np.linalg.inv(T)
    roads = g.load_vector(args.raw_dir.expanduser(), args.vector_zoom)['road']
    edges_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 22]
    cent_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 27]
    k = args.metric_scale
    e_m = g.rasterize_EN(edges_f, Tinv, W, H, res, ox, oy, width=1, scale=k)
    c_m = g.rasterize_EN(cent_f, Tinv, W, H, res, ox, oy, width=1, scale=k)
    road, info = g.gsi_road_area(e_m, c_m, res * k, args.gap_m, args.max_half_width_m)
    info |= {'scale': k, 'gap_m': args.gap_m, 'max_half_width_m': args.max_half_width_m}
    args.road_cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.road_cache, road=road, edges=e_m, centers=c_m, info=json.dumps(info))
    return road, e_m, c_m, info


def paved_mask(npz, H, W, res, thr):
    """舗装（地面の強度が thr 未満）と、強度が分かるセル。intensity_gate と同じ引数で判定する。"""
    a = mr.parse_args(['--max-intensity', str(thr)])
    gate, smooth = mr.intensity_gate(npz, H, W, res, a)
    known = np.isfinite(smooth)
    return known & ~gate, known


def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    g = load_module(args.gsi_module)
    _, W, H, res, ox, oy = mr.load_grid(args.layers_yaml)
    k = args.metric_scale
    rm = res * k
    road, e_m, c_m, info = gsi_road(g, args, W, H, res, ox, oy)

    masks = {'hand_candidate': mr.load_mask(args.hand_mask, (H, W))}
    for spec in args.mask:
        name, _, path = spec.partition('=')
        masks[name] = mr.load_mask(pathlib.Path(path), (H, W))

    tm = g.traj_mask(args.traj, H, W, res, ox, oy, k)
    near_k = ndimage.distance_transform_edt(~tm) * rm <= args.near_m
    out = {'gsi_road_area': info | {'cells': int(road.sum()), 'cells_near_traj': int((road & near_k).sum())},
           'gsi': {}, 'named_boxes': {}, 'paved': {}}
    mk = {n: g.downsample_mean(m, k) >= 0.5 for n, m in masks.items()}
    for name, m in mk.items():
        e = {'whole': g.overlap(m, road), f'near_traj_{args.near_m:g}m': g.overlap(m, road, near_k)}
        for bn, bx in GSI_BOXES.items():
            e[bn] = g.overlap(m, road, g.box_mask(m.shape, H, res, ox, oy, k, bx))
        out['gsi'][name] = e
        print(f'[GSI] {name:28s} 全体 {e["whole"]["iou"]}/{e["whole"]["precision"]}/{e["whole"]["recall"]}  '
              f'8m {e[f"near_traj_{args.near_m:g}m"]["iou"]}/{e[f"near_traj_{args.near_m:g}m"]["precision"]}/'
              f'{e[f"near_traj_{args.near_m:g}m"]["recall"]}  発進点 {e["start_road"]["iou"]}')

    # 舗装（原寸 5 cm）。軌跡からの距離は原寸で取り直す。
    traj = mr.load_traj_xy(args.traj)
    tr, tc = mr.xy_to_rc(traj, H, res, ox, oy)
    tmask = np.ones((H, W), bool)
    ok = (tr >= 0) & (tr < H) & (tc >= 0) & (tc < W)
    tmask[tr[ok], tc[ok]] = False
    near = ndimage.distance_transform_edt(tmask) * res <= args.near_m
    del tmask
    road_full = np.repeat(np.repeat(road, k, 0), k, 1)
    road_full = np.pad(road_full, ((0, H - road_full.shape[0]), (0, W - road_full.shape[1])))
    hand = masks['hand_candidate']
    a2 = res * res
    for thr in args.paved_thresholds:
        paved, known = paved_mask(args.ground_npz, H, W, res, thr)
        pn = paved & near
        e = {'paved_near_m2': round(float(pn.sum()) * a2, 1),
             'intensity_known_near_m2': round(float((known & near).sum()) * a2, 1),
             'hand_near_m2': round(float((hand & near).sum()) * a2, 1),
             'hand_near_paved_share': round(float((hand & pn).sum())
                                            / max(1, int((hand & near & known).sum())), 4),
             'paved_not_hand_near_m2': round(float((pn & ~hand).sum()) * a2, 1),
             'paved_not_hand_near_in_gsi_m2': round(float((pn & ~hand & road_full).sum()) * a2, 1),
             'gsi_near_paved_share': round(float((road_full & pn).sum())
                                           / max(1, int((road_full & near & known).sum())), 4)}
        for name, m in masks.items():
            if name != 'hand_candidate':
                e[f'{name}_not_hand_near_m2'] = round(float((m & ~hand & near).sum()) * a2, 1)
                e[f'{name}_not_hand_paved_near_m2'] = round(float((m & ~hand & pn).sum()) * a2, 1)
        out['paved'][f'lt{thr:g}'] = e
        print(f'[舗装 <{thr:g}] {json.dumps(e, ensure_ascii=False)}')
        for bn, box in NAMED_BOXES.items():
            rs, cs = mr.region_slice(box, H, res, ox, oy)
            n = (rs.stop - rs.start) * (cs.stop - cs.start)
            b = out['named_boxes'].setdefault(bn, {'box_xy': list(box), 'cells': n})
            b[f'paved_lt{thr:g}_share'] = round(float(paved[rs, cs].sum()) / n, 4)
    for bn, box in NAMED_BOXES.items():
        rs, cs = mr.region_slice(box, H, res, ox, oy)
        n = (rs.stop - rs.start) * (cs.stop - cs.start)
        b = out['named_boxes'][bn]
        b['gsi_share'] = round(float(road_full[rs, cs].sum()) / n, 4)
        for name, m in masks.items():
            b[f'{name}_share'] = round(float(m[rs, cs].sum()) / n, 4)
        print(f'[箱] {bn}: {json.dumps(b, ensure_ascii=False)}')

    # 1/4 の重ね絵（#205 の gsi_roads_vs_*_q.png と同じ色）：GSI の道路の面だけ = 薄い緑、
    # マスクだけ = 赤、両方 = 灰、GSI の道路縁 = 水色、中心線 = 黄。
    args.out_dir.mkdir(parents=True, exist_ok=True)
    q = 4
    road_q = g.downsample_mean(road, q // k) >= 0.5
    eq = g.downsample_any(e_m, q // k)
    cq = g.downsample_any(c_m, q // k)
    sizes = {}
    for name in args.overlay:
        m = g.downsample_mean(masks[name], q) >= 0.5
        h, w = min(m.shape[0], road_q.shape[0]), min(m.shape[1], road_q.shape[1])
        m, rq = m[:h, :w], road_q[:h, :w]
        rgb = np.full((h, w, 3), 255, np.uint8)
        rgb[rq] = (205, 235, 205)
        rgb[m & ~rq] = (240, 120, 120)
        rgb[m & rq] = (120, 120, 120)
        rgb[eq[:h, :w]] = (0, 160, 220)
        rgb[cq[:h, :w]] = (230, 190, 0)
        path = args.out_dir / f'gsi_vs_{name}_q.png'
        Image.fromarray(rgb).save(path, optimize=True)
        sizes[path.name] = path.stat().st_size

    out['params'] = {kk: (mr.short(v) if isinstance(v, pathlib.Path) else v) for kk, v in vars(args).items()}
    out['png_bytes'] = sizes
    out['elapsed_s'] = round(time.time() - t0, 1)
    js = args.json or args.out_dir / 'gsi_and_paved.json'
    js.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(f'所要 {out["elapsed_s"]} s。{js}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

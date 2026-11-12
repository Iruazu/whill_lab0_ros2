#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Web の背景地図（map.png / map_meta.json）が Nav2 の地図 YAML と揃っているかを確かめる（#171）。

背景地図は scripts/m7_make_web_map.py が YAML とその画像から作る別成果物で、地図を
作り直すたびに手で再生成している。忘れると、タップした地点と Nav2 のゴールがずれる。
2026-07-22 には、メタ情報だけ v2・画像は旧地図という組み合わせが実際に起きた。

確かめること:
  - map_meta.json の出典（source_pgm）が、YAML の image と同じファイルか
  - origin が同じか。resolution が YAML の resolution × downscale_factor か
  - width / height と map.png の寸法が、元画像を factor で割った値か
  - map.png の画素が、元画像を m7_make_web_map.py と同じ方法で縮小したものと一致するか
    （寸法を変えずに地図を塗り直した場合は、これでしか気づけない）
  - negate と free_thresh が、app.js の FREE_THRESHOLD の前提と矛盾しないか

一致すれば 0、食い違いがあれば 1、入力が読めなければ 2 で終わる。
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import sys

import numpy as np
from PIL import Image
import yaml

# 元画像は PIL の decompression-bomb の上限を超える。信頼できる手元の成果物に限って外す。
Image.MAX_IMAGE_PIXELS = None

REPO = Path(__file__).resolve().parents[1]
# 現用デモの地図。start_dispatch_demo.sh が nav_launch に map_variant:=v2 を渡す。
DEFAULT_SITE = 'campus'
DEFAULT_VARIANT = 'v2'
# map_server の trinary の既定値。UNKNOWN の画素はこの値で書かれている。
UNKNOWN_VALUE = 205


class InputError(Exception):
    """入力が読めない。食い違いとは分けて終了コード 2 にする。"""


def load_inputs(yaml_path, web_dir):
    try:
        nav = yaml.safe_load(yaml_path.read_text())
        meta = json.loads((web_dir / 'map_meta.json').read_text())
    except (OSError, ValueError, yaml.YAMLError) as e:
        raise InputError(f'読めません: {e}') from e
    for key in ('image', 'resolution', 'origin'):
        if key not in nav:
            raise InputError(f'{yaml_path} に {key} がありません')
    for key in ('image', 'width', 'height', 'resolution', 'origin', 'source_pgm',
                'downscale_factor'):
        if key not in meta:
            raise InputError(f'{web_dir / "map_meta.json"} に {key} がありません'
                             '（scripts/m7_make_web_map.py で作り直す）')
    return nav, meta


def free_threshold_of(app_js):
    m = re.search(r'^\s*const\s+FREE_THRESHOLD\s*=\s*(\d+)\s*;', app_js, re.MULTILINE)
    return int(m.group(1)) if m else None


def find_problems(yaml_path, web_dir, maps_root):
    """食い違いの説明の一覧と、注意の一覧を返す。"""
    nav, meta = load_inputs(yaml_path, web_dir)
    problems, notes = [], []

    # map_server と同じ規約で、image は YAML のディレクトリからの相対。
    image = (yaml_path.parent / nav['image']).resolve()
    if not image.is_file():
        raise InputError(f'YAML の画像がありません: {image}')
    # source_pgm はリポジトリ相対の docs/maps/... で書かれる。WHILL_MAPS_ROOT で別の地図
    # ディレクトリを指すときも比べられるよう、地図ディレクトリの中の相対パスで比べる。
    source = Path(meta['source_pgm'])
    try:
        source_in_maps = source.relative_to('docs/maps')
    except ValueError:
        source_in_maps = None
    try:
        image_in_maps = image.relative_to(maps_root.resolve())
    except ValueError:
        image_in_maps = None
    source_resolved = (maps_root / source_in_maps).resolve() if source_in_maps else None
    if source_resolved != image:
        problems.append(f'Web の地図の出典が違います: map_meta.json は {source}、'
                        f'YAML の画像は {image_in_maps or image}')

    origin = [float(v) for v in nav['origin']]
    meta_origin = [float(v) for v in meta['origin']]
    if len(origin) != len(meta_origin) or any(
            not math.isclose(a, b, abs_tol=1e-9) for a, b in zip(origin, meta_origin)):
        problems.append(f'origin が違います: YAML {origin}、map_meta.json {meta_origin}')

    factor = int(meta['downscale_factor'])
    if factor < 1:
        raise InputError(f'downscale_factor が不正です: {factor}')
    res = float(nav['resolution'])
    if not math.isclose(float(meta['resolution']), res * factor, rel_tol=1e-9):
        problems.append(f'resolution が違います: YAML {res} × factor {factor} = {res * factor}、'
                        f'map_meta.json {meta["resolution"]}')

    src = Image.open(image)
    w, h = src.size
    expected_size = (max(1, w // factor), max(1, h // factor))
    if (int(meta['width']), int(meta['height'])) != expected_size:
        problems.append(f'map_meta.json の寸法が違います: {meta["width"]}x{meta["height"]}、'
                        f'元画像 {w}x{h} を {factor} で割ると {expected_size[0]}x{expected_size[1]}')
    png_path = web_dir / meta['image']
    try:
        png = Image.open(png_path)
    except OSError as e:
        raise InputError(f'Web の地図画像が読めません: {e}') from e
    if png.size != expected_size:
        problems.append(f'{png_path.name} の寸法が違います: {png.size[0]}x{png.size[1]}、'
                        f'期待 {expected_size[0]}x{expected_size[1]}')
    else:
        # m7_make_web_map.py と同じ縮小。同じ PIL なら画素まで一致する（2026-11-12 に確認）。
        expected = np.asarray(src.convert('L').resize(expected_size, Image.BOX), dtype=np.int16)
        actual = np.asarray(png.convert('L'), dtype=np.int16)
        differ = int(np.count_nonzero(expected != actual))
        if differ:
            problems.append(f'{png_path.name} の画素が元画像と違います: {differ} 画素'
                            f'（最大差 {int(np.abs(expected - actual).max())}）。'
                            '地図を塗り直した後に作り直していない')

    # 割り切れないと、resize が縦横を factor ちょうどで縮めない。app.js は res_eff を
    # 縦横とも使うので、原点から遠い端ほどずれる。今の campus も縦が割り切れない。
    for name, n in (('幅', w), ('高さ', h)):
        if n % factor:
            err = (n - (n // factor) * factor) * res
            notes.append(f'元画像の{name} {n} が factor {factor} で割り切れません。'
                         f'Web の座標は原点の反対の端で最大 {err:.2f} m ずれます')

    # app.js の FREE_THRESHOLD は、FREE=254・UNKNOWN=205・negate 0 の画像を前提にしている。
    if int(nav.get('negate', 0)) != 0:
        problems.append('YAML が negate: 1 です。Web の FREE 判定（app.js）は negate: 0 の画像を前提にしている')
    if nav.get('mode', 'trinary') != 'trinary':
        problems.append(f'YAML の mode が {nav["mode"]} です。Web の FREE 判定は trinary を前提にしている')
    app_js = web_dir / 'app.js'
    if app_js.is_file() and 'free_thresh' in nav:
        threshold = free_threshold_of(app_js.read_text())
        # map_server は占有率 (255 - v) / 255 < free_thresh を FREE にする。
        nav_free_min = 255.0 * (1.0 - float(nav['free_thresh']))
        if threshold is None:
            problems.append('app.js に FREE_THRESHOLD が見つかりません')
        elif threshold < nav_free_min or threshold <= UNKNOWN_VALUE:
            # Web が Nav2 より広く FREE と見ると、Nav2 が拒む地点を選べてしまう。
            problems.append(f'app.js の FREE_THRESHOLD {threshold} が、Nav2 の FREE の下限 '
                            f'{nav_free_min:.1f}（free_thresh {nav["free_thresh"]}）か '
                            f'UNKNOWN {UNKNOWN_VALUE} 以下です')
    return problems, notes


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', default=DEFAULT_SITE)
    ap.add_argument('--variant', default=DEFAULT_VARIANT,
                    help='nav_launch の map_variant。空なら occupancy.yaml')
    ap.add_argument('--maps-root', type=Path,
                    default=Path(os.environ.get('WHILL_MAPS_ROOT', REPO / 'docs' / 'maps')))
    ap.add_argument('--yaml', type=Path, help='YAML を直接指定する（--site / --variant より優先）')
    ap.add_argument('--web-dir', type=Path, default=REPO / 'src' / 'whill_dispatch' / 'web',
                    help='map.png と map_meta.json の置き場。実機では install 側も確かめられる')
    args = ap.parse_args(argv)

    name = f'occupancy_{args.variant}.yaml' if args.variant else 'occupancy.yaml'
    yaml_path = args.yaml or args.maps_root / args.site / name
    try:
        problems, notes = find_problems(yaml_path, args.web_dir, args.maps_root)
    except InputError as e:
        print(f'地図の同期: 確かめられません: {e}', file=sys.stderr)
        return 2
    for note in notes:
        print(f'注意: {note}')
    if problems:
        print(f'地図の同期: 食い違いがあります（{yaml_path} と {args.web_dir}）', file=sys.stderr)
        for p in problems:
            print(f'  - {p}', file=sys.stderr)
        print('  直す: python3 scripts/m7_make_web_map.py --yaml '
              f'{yaml_path} --out-dir {args.web_dir}', file=sys.stderr)
        return 1
    print(f'地図の同期: ok（{yaml_path} と {args.web_dir}）')
    return 0


if __name__ == '__main__':
    sys.exit(main())

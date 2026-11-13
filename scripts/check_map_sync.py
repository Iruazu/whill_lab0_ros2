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
  - map.png の画素が、元画像を m7_make_web_map.py と同じ方法で縮小したものとほぼ一致するか
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
# 現用デモの地図。2026-11-13 に campus-m8 を既定にした（#190）。--variant を省くと、
# site ごとに start_dispatch_demo.sh が nav_launch へ渡す map_variant と同じものを使う。
# campus-m8 は手塗りの確定前で occupancy.yaml が無く、候補の candidate を読む。
# 旧 campus（fallback）と、表に無い site は従来どおり v2。対応を変えるときは
# start_dispatch_demo.sh の demo_variant_for と nav_launch.py の
# _DEFAULT_VARIANT_BY_SITE も揃える。
DEFAULT_SITE = 'campus-m8'
DEFAULT_VARIANT = 'v2'
DEMO_VARIANT_BY_SITE = {
    'campus-m8': 'candidate',
    'campus': 'v2',
}
# Pillow の版が違うと、BOX 縮小の丸めが1段ずれ得る。その差で正しい起動を止めないよう、
# 差が 2 を超える画素だけを数える。塗り直しは 0 と 254 の差なので、これでも検出できる。
PIXEL_TOLERANCE = 2


class InputError(Exception):
    """入力が読めない。食い違いとは分けて終了コード 2 にする。"""


def load_inputs(yaml_path, web_dir):
    try:
        nav = yaml.safe_load(yaml_path.read_text())
        meta = json.loads((web_dir / 'map_meta.json').read_text())
    except (OSError, ValueError, yaml.YAMLError) as e:
        raise InputError(f'読めません: {e}') from e
    if not isinstance(nav, dict):
        raise InputError(f'{yaml_path} が地図の YAML ではありません')
    if not isinstance(meta, dict):
        raise InputError(f'{web_dir / "map_meta.json"} が地図のメタ情報ではありません')
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


def resolve_source(source, maps_root):
    """map_meta.json の source_pgm を実際のファイルの場所にする。"""
    source = Path(source)
    # m7_make_web_map.py はリポジトリ相対で書く。docs/maps/... なら、WHILL_MAPS_ROOT で
    # 別の地図ディレクトリを使うときも比べられるよう、地図ディレクトリの中の相対として読む。
    try:
        return (maps_root / source.relative_to('docs/maps')).resolve()
    except ValueError:
        pass
    # 地図をリポジトリの外に置くと ../../data/maps/... や絶対パスになる。そのまま解決する。
    return (source if source.is_absolute() else REPO / source).resolve()


def find_problems(yaml_path, web_dir, maps_root):
    """食い違いの説明の一覧と、注意の一覧を返す。"""
    try:
        return _find_problems(yaml_path, web_dir, maps_root)
    except InputError:
        raise
    except (OSError, TypeError, ValueError, KeyError) as e:
        # 壊れた画像や数値でない origin を、食い違い（終了コード 1）と区別する。
        raise InputError(f'{type(e).__name__}: {e}') from e


def _find_problems(yaml_path, web_dir, maps_root):
    nav, meta = load_inputs(yaml_path, web_dir)
    problems, notes = [], []

    # map_server と同じ規約で、image は YAML のディレクトリからの相対。
    image = (yaml_path.parent / nav['image']).resolve()
    if not image.is_file():
        raise InputError(f'YAML の画像がありません: {image}')
    source = resolve_source(meta['source_pgm'], maps_root)
    if source != image:
        problems.append(f'Web の地図の出典が違います: map_meta.json は {meta["source_pgm"]}'
                        f'（{source}）、YAML の画像は {image}')

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
    png = Image.open(png_path)
    if png.size != expected_size:
        problems.append(f'{png_path.name} の寸法が違います: {png.size[0]}x{png.size[1]}、'
                        f'期待 {expected_size[0]}x{expected_size[1]}')
    else:
        # m7_make_web_map.py と同じ順で、元の画像のまま縮小してからグレーにする。
        expected = np.asarray(src.resize(expected_size, Image.BOX).convert('L'), dtype=np.int16)
        actual = np.asarray(png.convert('L'), dtype=np.int16)
        diff = np.abs(expected - actual)
        differ = int(np.count_nonzero(diff > PIXEL_TOLERANCE))
        if differ:
            problems.append(f'{png_path.name} の画素が元画像と違います: {differ} 画素'
                            f'（最大差 {int(diff.max())}）。地図を塗り直した後に作り直していない')

    # 割り切れないと、resize が縦横を factor ちょうどで縮めない。app.js は res_eff を
    # 縦横とも使うので、原点から遠い端ほどずれる。今の campus も縦が割り切れない。
    for name, n in (('幅', w), ('高さ', h)):
        if n % factor:
            err = (n % factor) * res
            notes.append(f'元画像の{name} {n} が factor {factor} で割り切れません。'
                         f'Web の座標は原点と反対の端で最大 {err:.2f} m ずれます')

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
        elif threshold + 1 <= nav_free_min:
            # app.js は 値 > FREE_THRESHOLD を FREE とする。Web が Nav2 より広く FREE と
            # 見ると、Nav2 が拒む地点を選べてしまう。
            problems.append(f'app.js の FREE_THRESHOLD {threshold} が、Nav2 の FREE の下限 '
                            f'{nav_free_min:.1f}（free_thresh {nav["free_thresh"]}）より緩い'
                            '（app.js は 値 > FREE_THRESHOLD を FREE とする）')
    return problems, notes


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', default=DEFAULT_SITE)
    ap.add_argument('--variant', default=None,
                    help='nav_launch の map_variant。空なら occupancy.yaml。省略すると '
                         'site ごとのデモの variant（campus-m8: candidate、それ以外は v2）')
    ap.add_argument('--maps-root', type=Path,
                    default=Path(os.environ.get('WHILL_MAPS_ROOT', REPO / 'docs' / 'maps')))
    ap.add_argument('--yaml', type=Path, help='YAML を直接指定する（--site / --variant より優先）')
    ap.add_argument('--web-dir', type=Path, default=REPO / 'src' / 'whill_dispatch' / 'web',
                    help='map.png と map_meta.json の置き場。実機では install 側も確かめられる')
    # 起動のたびに同じ注意が出ると読まれなくなる。起動前検査では出さない。
    ap.add_argument('--quiet-notes', action='store_true', help='注意（失敗ではないもの）を表示しない')
    args = ap.parse_args(argv)

    variant = (args.variant if args.variant is not None
               else DEMO_VARIANT_BY_SITE.get(args.site, DEFAULT_VARIANT))
    name = f'occupancy_{variant}.yaml' if variant else 'occupancy.yaml'
    yaml_path = args.yaml or args.maps_root / args.site / name
    try:
        problems, notes = find_problems(yaml_path, args.web_dir, args.maps_root)
    except InputError as e:
        print(f'地図の同期: 確かめられません: {e}', file=sys.stderr)
        return 2
    if not args.quiet_notes:
        for note in notes:
            print(f'注意: {note}')
    if problems:
        print(f'地図の同期: 食い違いがあります（{yaml_path} と {args.web_dir}）', file=sys.stderr)
        for p in problems:
            print(f'  - {p}', file=sys.stderr)
        # 作り直すのは常に src 側。install 側だけ直すと、copy の install では src が古いまま残る。
        print('  直す:\n'
              f'    cd {REPO}\n'
              f'    python3 scripts/m7_make_web_map.py --yaml {yaml_path.resolve()} '
              '--out-dir src/whill_dispatch/web\n'
              '    colcon build --symlink-install --packages-select whill_dispatch',
              file=sys.stderr)
        return 1
    print(f'地図の同期: ok（{yaml_path} と {args.web_dir}）')
    return 0


if __name__ == '__main__':
    sys.exit(main())

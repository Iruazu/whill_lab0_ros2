#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""docs/m*-bench-data/ の bag を ~/bags/ へ移す（#139）。方針は docs/bags.md。

引数なしは確認だけで、何も動かさない。--apply を付けたときだけ移す。
移すのは docs/bags-manifest.tsv に載った bag だけで、削除はしない。
"""
import argparse
import csv
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys

import yaml

MANIFEST = Path('docs/bags-manifest.tsv')
PATTERNS = ('*.db3', '*.mcap', '*.db3.zstd', '*.mcap.zstd')
FIELDS = ('source', 'bag', 'bytes', 'files', 'duration_s', 'messages',
          'started', 'storage')


def main_root(script):
    # worktree から実行しても main 側の docs を指す。bag は main 側にしか無い。
    common = subprocess.run(
        ['git', '-C', str(script.parent), 'rev-parse', '--path-format=absolute',
         '--git-common-dir'], check=True, capture_output=True, text=True)
    return Path(common.stdout.strip()).parent


def size_of(path):
    files = [p for p in path.rglob('*') if p.is_file()]
    return sum(p.stat().st_size for p in files), len(files)


def is_tracked(root, path):
    # git 管理下の bag は証跡として残したもの（07-12 の before）。動かすと差分になる。
    out = subprocess.run(['git', '-C', str(root), 'ls-files', '--', str(path)],
                         check=True, capture_output=True, text=True)
    return bool(out.stdout.strip())


def scan(root):
    """docs の bag を探して、一覧の行にする。"""
    rows = []
    errors = []
    bag_dirs = set()
    for pattern in PATTERNS:
        for bench in sorted((root / 'docs').glob('m*-bench-data')):
            bag_dirs.update(p.parent for p in bench.rglob(pattern))
    for path in sorted(bag_dirs):
        rel = path.relative_to(root)
        if is_tracked(root, path):
            continue
        meta_path = path / 'metadata.yaml'
        if not meta_path.is_file():
            errors.append(f'metadata.yaml が無い（記録が途中で切れた可能性）: {rel}')
            continue
        info = yaml.safe_load(meta_path.read_text())['rosbag2_bagfile_information']
        start = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
        size, count = size_of(path)
        rows.append({
            'source': str(rel),
            'bag': str(rel.relative_to('docs')),
            'bytes': size,
            'files': count,
            'duration_s': round(info['duration']['nanoseconds'] * 1e-9, 1),
            'messages': info['message_count'],
            'started': datetime.fromtimestamp(start, timezone.utc).astimezone()
                               .strftime('%Y-%m-%d %H:%M:%S %z'),
            'storage': info['storage_identifier'],
        })
    return rows, errors


def read_manifest(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f, delimiter='\t'))


def write_manifest(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, FIELDS, delimiter='\t', lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def state_of(row, root, bags):
    """一覧の1行を実物と比べ、(状態, 説明) を返す。状態が error なら移さない。"""
    src = root / row['source']
    dst = bags / row['bag']
    if src.exists() and dst.exists():
        return 'error', f'移動元と移動先の両方にある: {src} / {dst}'
    if not src.exists() and not dst.exists():
        return 'error', f'どちらにも無い: {src} / {dst}'
    where = src if src.exists() else dst
    expected = (int(row['bytes']), int(row['files']))
    actual = size_of(where)
    if actual != expected:
        return 'error', f'大きさが一覧と違う: {where}（一覧 {expected}、実物 {actual}）'
    return ('pending', '') if src.exists() else ('moved', '')


def nearest_existing(path):
    while not path.exists():
        path = path.parent
    return path


def prune_empty_parents(path, stop):
    # bag を出した後の空ディレクトリだけ片付ける。rmdir は中身があると失敗する。
    while path != stop and stop in path.parents:
        try:
            path.rmdir()
        except OSError:
            return
        path = path.parent


def human(size):
    return f'{size / 2**30:.1f} GiB'


def run(rows, root, bags, apply):
    states = [state_of(row, root, bags) for row in rows]
    problems = [message for state, message in states if state == 'error']
    pending = [row for row, (state, _) in zip(rows, states) if state == 'pending']
    moved = len(rows) - len(pending) - len(problems)
    total = sum(int(row['bytes']) for row in pending)
    print(f'一覧 {len(rows)} 本: 移動済み {moved}、未移動 {len(pending)}（{human(total)}）、'
          f'問題 {len(problems)}')
    print(f'移動先: {bags}')
    for message in problems:
        print(f'  ERROR {message}', file=sys.stderr)
    # 1本でも問題があれば1本も動かさない。途中まで動いた状態を作らない。
    if problems:
        return 1
    if not apply:
        for row in pending:
            print(f"  {row['source']} -> {bags / row['bag']}")
        if pending:
            print('確認だけです。移すときは --apply を付けます。')
        return 0
    if pending and nearest_existing(bags).stat().st_dev != (root / 'docs').stat().st_dev:
        # rename はコピーを伴わない。別ディスクへのコピーと削除をここで行うと、
        # 中断したときに bag を失う恐れがあるので、手順を分ける。
        print(f'ERROR {bags} は docs と別のファイルシステムです。'
              'このスクリプトでは移しません。', file=sys.stderr)
        return 1
    for row in pending:
        src = root / row['source']
        dst = bags / row['bag']
        dst.parent.mkdir(parents=True, exist_ok=True)
        os.rename(src, dst)
        state, message = state_of(row, root, bags)
        if state != 'moved':
            print(f'ERROR 移動後の確認で止めました: {message}', file=sys.stderr)
            return 1
        prune_empty_parents(src.parent, root / 'docs')
        print(f"  移動: {row['source']} -> {dst}")
    return 0


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--apply', action='store_true', help='実際に移す')
    ap.add_argument('--write-manifest', action='store_true',
                    help='docs の bag を調べて一覧を書く（移さない）')
    ap.add_argument('--manifest', type=Path, help='一覧のパス（既定: このリポジトリの一覧）')
    ap.add_argument('--repo-root', type=Path, help='bag のあるリポジトリ（既定: main 側）')
    ap.add_argument('--bags-root', type=Path,
                    default=Path(os.environ.get('WHILL_BAGS_ROOT', Path.home() / 'bags')))
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    script = Path(__file__).resolve()
    root = (args.repo_root or main_root(script)).resolve()
    manifest = args.manifest or script.parent.parent / MANIFEST
    if args.write_manifest:
        rows, errors = scan(root)
        for message in errors:
            print(f'ERROR {message}', file=sys.stderr)
        if errors:
            return 1
        write_manifest(manifest, rows)
        print(f'{manifest}: {len(rows)} 本、'
              f'{human(sum(row["bytes"] for row in rows))}')
        return 0
    return run(read_manifest(manifest), root, args.bags_root.expanduser(), args.apply)


if __name__ == '__main__':
    sys.exit(main())

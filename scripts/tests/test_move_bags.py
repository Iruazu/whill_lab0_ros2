# SPDX-License-Identifier: BSD-3-Clause
"""bag の移動（#139）の安全側の動きを確かめる。実データは使わない。"""
import importlib.util
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'move_bags.py'
spec = importlib.util.spec_from_file_location('move_bags', SCRIPT)
move_bags = importlib.util.module_from_spec(spec)
spec.loader.exec_module(move_bags)

METADATA = """rosbag2_bagfile_information:
  storage_identifier: sqlite3
  duration:
    nanoseconds: 2500000000
  starting_time:
    nanoseconds_since_epoch: 1785478491000000000
  message_count: 7
"""


def make_bag(path, payload=b'x' * 10, metadata=True):
    path.mkdir(parents=True)
    (path / 'bag_0.db3').write_bytes(payload)
    if metadata:
        (path / 'metadata.yaml').write_text(METADATA)


@pytest.fixture
def env(tmp_path):
    root = tmp_path / 'repo'
    root.mkdir()
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    make_bag(root / 'docs/m7-bench-data/run1/rosbag2_a')
    make_bag(root / 'docs/m5r-bench-data/run2/bag')
    # bag の隣の成果物は docs に残す。
    (root / 'docs/m5r-bench-data/run2/NOTES.md').write_text('note')
    return root, tmp_path / 'bags', tmp_path / 'manifest.tsv'


def cli(env, *extra):
    root, bags, manifest = env
    return move_bags.main(['--repo-root', str(root), '--bags-root', str(bags),
                           '--manifest', str(manifest), *extra])


def test_manifest_lists_bags_without_moving(env):
    root, _, manifest = env
    assert cli(env, '--write-manifest') == 0
    rows = move_bags.read_manifest(manifest)
    assert [r['bag'] for r in rows] == ['m5r-bench-data/run2/bag', 'm7-bench-data/run1/rosbag2_a']
    assert rows[0]['duration_s'] == '2.5'
    assert rows[0]['messages'] == '7'
    assert (root / 'docs/m7-bench-data/run1/rosbag2_a/bag_0.db3').exists()


def test_tracked_bag_stays_out_of_manifest(env):
    root, _, manifest = env
    tracked = root / 'docs/m6r-bench-data/evidence/bag'
    make_bag(tracked)
    subprocess.run(['git', '-C', str(root), 'add', '-f', str(tracked)], check=True)
    assert cli(env, '--write-manifest') == 0
    assert not [r for r in move_bags.read_manifest(manifest) if 'evidence' in r['bag']]


def test_bag_without_metadata_stops_manifest(env):
    root, _, manifest = env
    make_bag(root / 'docs/m7-bench-data/run3/broken', metadata=False)
    assert cli(env, '--write-manifest') == 1
    assert not manifest.exists()


def test_default_moves_nothing(env):
    root, bags, _ = env
    cli(env, '--write-manifest')
    assert cli(env) == 0
    assert (root / 'docs/m7-bench-data/run1/rosbag2_a').exists()
    assert not bags.exists()


def test_apply_moves_keeps_neighbours_and_prunes_empty(env):
    root, bags, _ = env
    cli(env, '--write-manifest')
    assert cli(env, '--apply') == 0
    assert (bags / 'm7-bench-data/run1/rosbag2_a/bag_0.db3').read_bytes() == b'x' * 10
    assert (bags / 'm5r-bench-data/run2/bag/metadata.yaml').exists()
    assert (root / 'docs/m5r-bench-data/run2/NOTES.md').exists()
    assert not (root / 'docs/m7-bench-data').exists()


def test_apply_again_is_no_op(env):
    _, bags, _ = env
    cli(env, '--write-manifest')
    assert cli(env, '--apply') == 0
    assert cli(env, '--apply') == 0
    assert (bags / 'm7-bench-data/run1/rosbag2_a/bag_0.db3').exists()


def test_existing_destination_blocks_every_move(env):
    root, bags, _ = env
    cli(env, '--write-manifest')
    make_bag(bags / 'm7-bench-data/run1/rosbag2_a', payload=b'other')
    assert cli(env, '--apply') == 1
    # 衝突していない方も動かない。
    assert (root / 'docs/m5r-bench-data/run2/bag').exists()
    assert (root / 'docs/m7-bench-data/run1/rosbag2_a').exists()
    assert (bags / 'm7-bench-data/run1/rosbag2_a/bag_0.db3').read_bytes() == b'other'


def test_size_mismatch_blocks_every_move(env):
    root, _, _ = env
    cli(env, '--write-manifest')
    (root / 'docs/m7-bench-data/run1/rosbag2_a/bag_0.db3').write_bytes(b'short')
    assert cli(env, '--apply') == 1
    assert (root / 'docs/m5r-bench-data/run2/bag').exists()


def test_missing_bag_is_an_error(env):
    root, _, _ = env
    cli(env, '--write-manifest')
    (root / 'docs/m7-bench-data/run1/rosbag2_a/bag_0.db3').unlink()
    (root / 'docs/m7-bench-data/run1/rosbag2_a/metadata.yaml').unlink()
    (root / 'docs/m7-bench-data/run1/rosbag2_a').rmdir()
    assert cli(env) == 1


def test_other_filesystem_is_refused(env, monkeypatch):
    root, _, _ = env
    cli(env, '--write-manifest')
    real_stat = Path.stat

    class Other:
        st_dev = -1

    def fake_stat(self, *a, **k):
        # 移動先の側（bags の親）だけ別デバイスに見せる。
        if self == root.parent:
            return Other()
        return real_stat(self, *a, **k)

    monkeypatch.setattr(Path, 'stat', fake_stat)
    assert cli(env, '--apply') == 1
    assert (root / 'docs/m7-bench-data/run1/rosbag2_a').exists()

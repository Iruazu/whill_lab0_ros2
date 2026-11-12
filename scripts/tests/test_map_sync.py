"""check_map_sync.py を、小さな合成地図と現用の campus 地図で検証する。実機は使わない。

合成地図は m7_make_web_map.py で背景を作り、そこから1箇所ずつ壊して検出を確かめる。
本物の地図と背景は書き換えない。
"""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import check_map_sync  # noqa: E402

FACTOR = 4


class SyntheticMapTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.maps = root / 'docs' / 'maps'
        site = self.maps / 'site'
        site.mkdir(parents=True)
        # 道（254）、未知（205）、壁（0）。高さ 50 は 4 で割り切れない（campus と同じ状況）。
        pixels = np.full((50, 64), 205, dtype=np.uint8)
        pixels[20:30, :] = 254
        pixels[:, 10] = 0
        self.pgm = site / 'map.pgm'
        Image.fromarray(pixels).save(self.pgm)
        self.yaml = site / 'occupancy_v2.yaml'
        self.nav = {'image': 'map.pgm', 'resolution': 0.05, 'origin': [-1.5, -2.25, 0.0],
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
        self.yaml.write_text(yaml.safe_dump(self.nav))
        self.web = root / 'web'
        self.web.mkdir()
        shutil.copy(REPO / 'src' / 'whill_dispatch' / 'web' / 'app.js', self.web / 'app.js')
        subprocess.run([sys.executable, str(SCRIPTS / 'm7_make_web_map.py'), '--yaml', str(self.yaml),
                        '--out-dir', str(self.web), '--factor', str(FACTOR)],
                       check=True, capture_output=True)
        # m7_make_web_map.py はリポジトリ相対で書く。一時ディレクトリは docs/maps/... 形式に
        # 直して、WHILL_MAPS_ROOT で別の地図ディレクトリを使う場合として確かめる。
        self.generated_source = json.loads((self.web / 'map_meta.json').read_text())['source_pgm']
        self.edit_meta(source_pgm='docs/maps/site/map.pgm')

    def edit_meta(self, **changes):
        path = self.web / 'map_meta.json'
        meta = json.loads(path.read_text())
        meta.update(changes)
        path.write_text(json.dumps(meta))

    def edit_yaml(self, **changes):
        self.yaml.write_text(yaml.safe_dump(dict(self.nav, **changes)))

    def problems(self):
        return check_map_sync.find_problems(self.yaml, self.web, self.maps)

    def assert_one_problem(self, text):
        problems, _ = self.problems()
        self.assertEqual(len(problems), 1, problems)
        self.assertIn(text, problems[0])

    def test_fresh_output_is_in_sync(self):
        problems, notes = self.problems()
        self.assertEqual(problems, [])
        # 50 = 4 × 12 + 2。原点と反対の端で 2 × 0.05 m ずれる。
        self.assertEqual(len(notes), 1, notes)
        self.assertIn('最大 0.10 m', notes[0])

    def test_origin_mismatch(self):
        self.edit_yaml(origin=[-1.5, -2.0, 0.0])
        self.assert_one_problem('origin が違います')

    def test_resolution_mismatch(self):
        self.edit_yaml(resolution=0.1)
        self.assert_one_problem('resolution が違います')

    def test_repainted_map_without_regenerating(self):
        # 寸法・原点を変えずに塗り直した。メタ情報だけでは気づけない食い違い。
        pixels = np.asarray(Image.open(self.pgm)).copy()
        pixels[0:8, 40:48] = 0
        Image.fromarray(pixels).save(self.pgm)
        self.assert_one_problem('画素が元画像と違います')

    def test_source_is_another_map(self):
        # 2026-07-22 の事故の形。メタ情報は別の画像から作られている。
        shutil.copy(self.pgm, self.pgm.with_name('old.pgm'))
        self.edit_meta(source_pgm='docs/maps/site/old.pgm')
        self.assert_one_problem('出典が違います')

    def test_size_mismatch(self):
        self.edit_meta(width=32)
        self.assert_one_problem('map_meta.json の寸法が違います')

    def test_negate_breaks_web_free_gate(self):
        self.edit_yaml(negate=1)
        self.assert_one_problem('negate: 1')

    def test_free_threshold_looser_than_nav2(self):
        app = self.web / 'app.js'
        app.write_text(app.read_text().replace('const FREE_THRESHOLD = 230;',
                                               'const FREE_THRESHOLD = 204;'))
        self.assert_one_problem('FREE_THRESHOLD 204')
        self.assertIn('より緩い', self.problems()[0][0])

    def test_free_threshold_equal_to_nav2_lower_bound_passes(self):
        # app.js は 値 > 205 を FREE とする。Nav2 の FREE（206 以上）と同じ範囲。
        app = self.web / 'app.js'
        app.write_text(app.read_text().replace('const FREE_THRESHOLD = 230;',
                                               'const FREE_THRESHOLD = 205;'))
        self.assertEqual(self.problems()[0], [])

    def test_map_outside_repo_uses_generated_source(self):
        # 地図をリポジトリの外に置くと、source_pgm は ../../... の相対になる。
        self.assertFalse(self.generated_source.startswith('docs/maps'), self.generated_source)
        self.edit_meta(source_pgm=self.generated_source)
        self.assertEqual(self.problems()[0], [])

    def test_small_rounding_difference_is_tolerated(self):
        # Pillow の版の違いで BOX 縮小の丸めが1段ずれる場合を模す。
        png = self.web / 'map.png'
        pixels = np.asarray(Image.open(png)).astype(np.int16)
        Image.fromarray(np.clip(pixels + 1, 0, 255).astype(np.uint8)).save(png)
        self.assertEqual(self.problems()[0], [])

    def test_broken_inputs_are_input_errors(self):
        for broken in ('', 'origin: [a, b, c]\nimage: map.pgm\nresolution: 0.05\n'):
            self.yaml.write_text(broken)
            with self.assertRaises(check_map_sync.InputError):
                self.problems()
        self.edit_yaml()
        self.pgm.write_bytes(b'not an image')
        with self.assertRaises(check_map_sync.InputError):
            self.problems()

    def test_cli_exit_codes(self):
        cmd = [sys.executable, str(SCRIPTS / 'check_map_sync.py'), '--maps-root', str(self.maps),
               '--site', 'site', '--web-dir', str(self.web)]
        ok = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.edit_yaml(origin=[0.0, 0.0, 0.0])
        bad = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(bad.returncode, 1, bad.stdout)
        self.assertIn('食い違いがあります', bad.stderr)
        self.assertIn('m7_make_web_map.py --yaml', bad.stderr)
        # install 側を検査しても、作り直す先は src 側。
        self.assertIn('--out-dir src/whill_dispatch/web', bad.stderr)
        self.assertIn('colcon build', bad.stderr)
        self.edit_yaml()
        self.assertIn('注意:', subprocess.run(cmd, capture_output=True, text=True).stdout)
        quiet = subprocess.run(cmd + ['--quiet-notes'], capture_output=True, text=True)
        self.assertEqual(quiet.returncode, 0, quiet.stderr)
        self.assertNotIn('注意:', quiet.stdout)
        self.yaml.write_text('')
        empty = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(empty.returncode, 2, empty.stderr)
        self.assertNotIn('Traceback', empty.stderr)
        missing = subprocess.run(cmd + ['--variant', 'nothing'], capture_output=True, text=True)
        self.assertEqual(missing.returncode, 2, missing.stderr)


class CurrentMapTests(unittest.TestCase):
    def test_campus_v2_and_web_background_are_in_sync(self):
        maps = REPO / 'docs' / 'maps'
        yaml_path = maps / 'campus' / 'occupancy_v2.yaml'
        if not (maps / 'campus' / 'v2' / 'final.pgm').is_file():
            self.skipTest('campus の地図画像が無い')
        problems, _ = check_map_sync.find_problems(
            yaml_path, REPO / 'src' / 'whill_dispatch' / 'web', maps)
        self.assertEqual(problems, [])


if __name__ == '__main__':
    unittest.main()

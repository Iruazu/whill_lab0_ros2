"""既定の site と variant が、3 つの launch・起動スクリプト・同期検査で揃っているかを確かめる。

ROS は使わない。launch は import せず ast で定数を読む（launch_ros が無い環境でも回すため）。
起動スクリプトは --dry-run の出力から、nav_launch に渡す map_variant を読む。

揃える範囲（2026-11-13、#190）:
  - 既定の site（campus-m8）は、どこでも同じでなければならない。1 か所でも違うと、
    localizer の PCD・Nav2 の格子・配車地点が別の地図を指し、自己位置と経路が食い違う。
  - campus-m8 の variant（candidate）も、どこでも同じでなければならない。campus-m8 には
    occupancy.yaml が無いので、nav_launch が別の variant を既定にすると起動に失敗し、
    スクリプトだけが違うと Web の背景の検査が Nav2 と別の地図を見る。
  - fallback の campus は、わざと揃えていない。nav_launch で variant を省くと従来どおり
    occupancy.yaml（M5-R の元の地図）を読み、スクリプトと同期検査は v2 を使う。
    nav_launch の空 = occupancy.yaml の意味を変えないため（_DEFAULT_VARIANT_BY_SITE の
    コメント）。デモの起動はいつもスクリプトか runbook で map_variant:=v2 を明示するので、
    この非対称は運用に出ない。ここでは今の非対称そのものを固定しておく。
"""
import ast
from pathlib import Path
import re
import subprocess
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import check_map_sync  # noqa: E402

LAUNCHES = {
    'bringup': REPO / 'src' / 'whill_safety' / 'launch' / 'm6r_bringup_launch.py',
    'nav': REPO / 'src' / 'whill_navigation' / 'launch' / 'nav_launch.py',
    'dispatch': REPO / 'src' / 'whill_dispatch' / 'launch' / 'dispatch_launch.py',
}


def module_constants(path):
    """モジュールの最上位にある定数の代入を、評価せずに読む。"""
    out = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass  # os.path.join(...) など定数でないものは読まない
    return out


def dry_run(*args):
    result = subprocess.run(['bash', str(SCRIPTS / 'start_dispatch_demo.sh'), '--dry-run', *args],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    out = {}
    for pkg in ('m6r_bringup_launch.py', 'nav_launch.py', 'dispatch_launch.py'):
        line = next(l for l in result.stdout.splitlines() if pkg in l)
        out[pkg] = dict(re.findall(r'(\w+):=(\S+)', line))
    return out


class DefaultSiteConsistencyTests(unittest.TestCase):
    def setUp(self):
        self.consts = {name: module_constants(path) for name, path in LAUNCHES.items()}

    def test_default_site_matches_everywhere(self):
        expected = check_map_sync.DEFAULT_SITE
        for name, consts in self.consts.items():
            with self.subTest(launch=name):
                self.assertEqual(consts.get('_DEFAULT_SITE'), expected)
        launched = dry_run()
        for pkg, args in launched.items():
            with self.subTest(script=pkg):
                self.assertEqual(args['site'], expected)

    def test_default_site_variant_matches_everywhere(self):
        site = check_map_sync.DEFAULT_SITE
        expected = check_map_sync.DEMO_VARIANT_BY_SITE[site]
        self.assertEqual(self.consts['nav']['_DEFAULT_VARIANT_BY_SITE'][site], expected)
        self.assertEqual(dry_run()['nav_launch.py']['map_variant'], expected)
        # その variant の地図が実際にある（無いと既定の起動が必ず失敗する）。
        self.assertTrue((REPO / 'docs' / 'maps' / site / f'occupancy_{expected}.yaml').is_file())

    def test_fallback_campus_asymmetry_is_intended(self):
        # 上の docstring のとおり、campus だけは nav_launch とスクリプトで既定が違う。
        self.assertNotIn('campus', self.consts['nav']['_DEFAULT_VARIANT_BY_SITE'])
        self.assertEqual(check_map_sync.DEMO_VARIANT_BY_SITE['campus'], 'v2')
        launched = dry_run('--site', 'campus')
        self.assertEqual(launched['nav_launch.py']['map_variant'], 'v2')
        for args in launched.values():
            self.assertEqual(args['site'], 'campus')


if __name__ == '__main__':
    unittest.main()

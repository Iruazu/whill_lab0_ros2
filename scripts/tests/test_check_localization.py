"""check_localization.py の判定と集計を、合成の受信列で確かめる。

ROS は使わない（rclpy は main の中でだけ import される）。failsafe の Layer B の定数の写しが
failsafe_node.py と一致すること、既定の site が bringup と一致することもここで確かめる。
写しがずれると、failsafe なら止まる状態を PASS にしてしまう。
"""
import ast
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import check_localization as cl  # noqa: E402


def module_constants(path):
    out = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return out


def sample(t, fitness=0.02, converged=True, initialpose=True, **extra):
    raw = {'initialpose_received': 'true' if initialpose else 'false',
           'map_received': 'true', 'consecutive_rejected_updates': '0',
           'effective_score_threshold': '6.000000',
           'recovery_state': 'tracking', 'failure_category': 'none'}
    if fitness is not None:
        raw['fitness_score'] = 'nan' if isinstance(fitness, float) and math.isnan(fitness) \
            else f'{fitness:.6f}'
    if converged is not None:
        raw['has_converged'] = 'true' if converged else 'false'
    raw.update(extra)
    return cl.Sample(t, raw)


def pose(t):
    return cl.Pose(t, t, 0.0, 0.0, 0.0, 0.0)


def steady(t0, t1, hz=10.0, **kw):
    """t0〜t1 の一定周期の受信。浮動小数の和の誤差を避けて、番号から時刻を作る。"""
    n = int(round((t1 - t0) * hz))
    return [sample(t0 + i / hz, **kw) for i in range(n)]


def poses_between(t0, t1, hz=10.0):
    n = int(round((t1 - t0) * hz))
    return [pose(t0 + i / hz) for i in range(n)]


class ConstantsTest(unittest.TestCase):
    def test_layer_b_constants_match_failsafe(self):
        c = module_constants(REPO / 'src/whill_safety/whill_safety/failsafe_node.py')
        for name in ('FITNESS_MAX', 'FITNESS_WINDOW_S', 'PCL_POSE_TIMEOUT_S'):
            self.assertEqual(getattr(cl, name), c[name], name)

    def test_default_site_matches_bringup(self):
        c = module_constants(REPO / 'src/whill_safety/launch/m6r_bringup_launch.py')
        self.assertEqual(cl.DEFAULT_SITE, c['_DEFAULT_SITE'])

    def test_reference_sha_matches_metadata(self):
        # 参考値は sha256 d244865… の地図の再生。metadata の正本が変わったら参考値も見直す。
        sha = cl.expected_pcd_sha256(REPO / 'docs/maps/campus-m8/metadata.yaml')
        self.assertEqual(sha, 'd244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5')

    def test_alignment_keys_exist_upstream(self):
        hpp = REPO / 'src/third_party/lidar_localization_ros2/include/lidar_localization/' \
                     'alignment_diagnostics_policy.hpp'
        if not hpp.exists():
            self.skipTest('src/third_party が無い（worktree で run_tests.sh を通さずに実行）')
        text = hpp.read_text()
        for key in cl.ALIGNMENT_KEYS:
            self.assertIn(f'"{key}"', text, key)


class ParseTest(unittest.TestCase):
    def test_bool_and_float_follow_failsafe(self):
        self.assertIs(cl.to_bool('true'), True)
        self.assertIs(cl.to_bool('OK'), True)
        self.assertIs(cl.to_bool('0'), False)
        self.assertIsNone(cl.to_bool('maybe'))
        self.assertTrue(math.isnan(cl.to_float('nan')))
        self.assertIsNone(cl.to_float('x'))

    def test_merge_later_status_wins(self):
        raw, level, msg = cl.merge_status_values([
            (0, 'ok', [('fitness_score', '0.1'), ('has_converged', 'true')]),
            (2, 'bad', [('fitness_score', '3.0')]),
        ])
        self.assertEqual(raw['fitness_score'], '3.0')
        self.assertEqual(raw['has_converged'], 'true')
        self.assertEqual((level, msg), (0, 'ok'))

    def test_yaw(self):
        s, c = math.sin(math.radians(45)), math.cos(math.radians(45))
        self.assertAlmostEqual(cl.quat_to_yaw_deg(0, 0, s, c), 90.0)

    def test_motion(self):
        self.assertEqual(cl.classify_motion(None, 0, 0), 'unknown')
        self.assertEqual(cl.classify_motion(1.5, 0, 0), 'unknown')
        self.assertEqual(cl.classify_motion(0.2, 0.01, 0.0), 'stationary')
        self.assertEqual(cl.classify_motion(0.2, 0.3, 0.0), 'moving')
        self.assertEqual(cl.classify_motion(0.2, 0.0, -0.2), 'moving')

    def test_percentile_matches_numpy_linear(self):
        xs = [0.5, 0.1, 0.3, 0.9, 0.7]
        self.assertAlmostEqual(cl.percentile(xs, 50), 0.5)
        self.assertAlmostEqual(cl.percentile(xs, 95), 0.86)
        self.assertIsNone(cl.percentile([], 50))


class JudgeTest(unittest.TestCase):
    def run_judge(self, samples, poses, end_t, site='campus-m8', env=()):
        s = cl.summarize(samples, poses, end_t)
        return cl.judge(s, site, list(env)), s

    def test_clean_run_passes(self):
        (v, reasons), s = self.run_judge(steady(0, 60), poses_between(0, 60), 60.0)
        self.assertEqual(v, 'PASS', reasons)
        self.assertEqual(cl.VERDICT_EXIT[v], 0)
        self.assertAlmostEqual(s['fitness']['p50'], 0.02)
        self.assertEqual(s['fitness_over_max_count'], 0)

    def test_no_data(self):
        (v, _), _ = self.run_judge([], [], 10.0)
        self.assertEqual(v, 'NO_DATA')
        self.assertEqual(cl.VERDICT_EXIT[v], 2)

    def test_not_initialized_is_not_fail(self):
        # 初期位置の前は has_converged=false が続いても FAIL にしない。
        samples = steady(0, 30, converged=False, fitness=None, initialpose=False)
        (v, _), s = self.run_judge(samples, [], 30.0)
        self.assertEqual(v, 'NOT_INITIALIZED')
        self.assertEqual(cl.VERDICT_EXIT[v], 2)
        self.assertFalse(s['initialpose_received'])

    def test_pre_init_samples_not_judged(self):
        pre = steady(0, 10, converged=False, fitness=5.0, initialpose=False)
        post = steady(10, 60)
        (v, reasons), s = self.run_judge(pre + post, poses_between(10, 60), 60.0)
        self.assertEqual(v, 'PASS', reasons)
        self.assertAlmostEqual(s['armed_at_s'], 10.0)
        self.assertEqual(s['alignment_judged_count'], len(post))
        self.assertEqual(s['fitness']['max'], 0.02)

    def test_fitness_over_max_for_window_fails(self):
        samples = steady(0, 20) + steady(20, 22.5, fitness=1.5) + steady(22.5, 40)
        (v, reasons), s = self.run_judge(samples, poses_between(0, 40), 40.0)
        self.assertEqual(v, 'FAIL')
        self.assertTrue(any(r.startswith('B:fitness') for r in reasons))
        self.assertEqual(cl.VERDICT_EXIT[v], 1)

    def test_fitness_spike_shorter_than_window_passes(self):
        # 1.9 s で戻れば failsafe は止めない。回数だけ数える。
        samples = steady(0, 20) + steady(20, 21.9, fitness=1.5) + steady(21.9, 40)
        (v, reasons), s = self.run_judge(samples, poses_between(0, 40), 40.0)
        self.assertEqual(v, 'PASS', reasons)
        self.assertEqual(s['fitness_over_max_count'], 19)

    def test_bad_until_end_counts(self):
        # 最後まで解けない悪い状態は、計測の終わりまで続いたとみなす。
        samples = steady(0, 20) + [sample(20.0, fitness=3.0)]
        (v, reasons), s = self.run_judge(samples, poses_between(0, 23), 23.0)
        self.assertEqual(v, 'FAIL')
        self.assertTrue(s['layer_b_fitness_runs'][-1]['ended_by_stop'])

    def test_missing_value_keeps_state(self):
        # fitness の無い受信は状態を変えない（failsafe と同じ）。
        samples = [sample(0.0, fitness=2.0), sample(1.0, fitness=None),
                   sample(2.5, fitness=None), sample(3.0, fitness=0.1)] + steady(3.1, 30)
        runs = cl.bad_runs(samples, 30.0, cl.fitness_bad)
        self.assertEqual(len(runs), 1)
        self.assertAlmostEqual(runs[0].duration, 3.0)

    def test_nan_fitness_follows_failsafe(self):
        # failsafe では nan > 1.0 が False なので fitness では止まらない。未収束で拾う。
        samples = steady(0, 10) + steady(10, 13, fitness=math.nan, converged=False) \
            + steady(13, 30)
        (v, reasons), s = self.run_judge(samples, poses_between(0, 30), 30.0)
        self.assertEqual(v, 'FAIL')
        self.assertTrue(any(r.startswith('B:converged') for r in reasons))
        self.assertFalse(any(r.startswith('B:fitness') for r in reasons))
        self.assertEqual(s['fitness_nan_count'], 30)

    def test_pose_gap_over_timeout_fails(self):
        poses = poses_between(0, 20) + poses_between(21.2, 40)
        (v, reasons), s = self.run_judge(steady(0, 40), poses, 40.0)
        self.assertEqual(v, 'FAIL')
        self.assertTrue(any(r.startswith('B:pcl_pose_silent') for r in reasons))
        self.assertGreater(s['pose_gap']['max'], 1.0)

    def test_pose_gap_at_end_fails(self):
        (v, reasons), _ = self.run_judge(steady(0, 40), poses_between(0, 30), 40.0)
        self.assertEqual(v, 'FAIL')

    def test_pose_gap_under_timeout_passes(self):
        # 参考値の最大 0.81〜0.91 s 程度の間隔は通る。
        poses = poses_between(0, 20) + poses_between(20.8, 40)
        (v, reasons), s = self.run_judge(steady(0, 40), poses, 40.0)
        self.assertEqual(v, 'PASS', reasons)
        self.assertAlmostEqual(s['pose_gap']['max'], 0.9, places=6)

    def test_initialized_without_pose_fails(self):
        (v, reasons), _ = self.run_judge(steady(0, 30), [], 30.0)
        self.assertEqual(v, 'FAIL')
        self.assertTrue(any('never_arrived' in r for r in reasons))

    def test_pose_without_alignment_is_incomplete(self):
        (v, _), _ = self.run_judge([], poses_between(0, 30), 30.0)
        self.assertEqual(v, 'INCOMPLETE')
        self.assertEqual(cl.VERDICT_EXIT[v], 2)

    def test_high_p95_warns(self):
        limit = cl.WARN_P95_FACTOR * cl.REFERENCE['campus-m8']['fitness_p95'][1]
        (v, reasons), _ = self.run_judge(steady(0, 60, fitness=limit * 1.2),
                                         poses_between(0, 60), 60.0)
        self.assertEqual(v, 'WARN')
        self.assertTrue(any('p95' in r for r in reasons))
        self.assertEqual(cl.VERDICT_EXIT[v], 0)

    def test_no_reference_site_does_not_warn_on_p95(self):
        (v, reasons), _ = self.run_judge(steady(0, 60, fitness=0.5),
                                         poses_between(0, 60), 60.0, site='campus')
        self.assertEqual(v, 'PASS', reasons)

    def test_short_window_warns(self):
        (v, reasons), _ = self.run_judge(steady(0, 5), poses_between(0, 5), 5.0)
        self.assertEqual(v, 'WARN')

    def test_env_warning_propagates(self):
        (v, reasons), _ = self.run_judge(steady(0, 60), poses_between(0, 60), 60.0,
                                         env=['RMW_IMPLEMENTATION=None'])
        self.assertEqual(v, 'WARN')

    def test_fail_beats_warn(self):
        samples = steady(0, 20) + steady(20, 23, fitness=2.0) + steady(23, 60)
        (v, _), _ = self.run_judge(samples, poses_between(0, 60), 60.0,
                                   env=['RMW_IMPLEMENTATION=None'])
        self.assertEqual(v, 'FAIL')

    def test_counts(self):
        samples = steady(0, 10) + [
            sample(10.0, fitness=7.0, consecutive_rejected_updates='3',
                   recovery_state='degraded', failure_category='bad_match'),
            sample(10.1, converged=False, reinitialization_requested='true'),
        ] + steady(10.2, 30)
        s = cl.summarize(samples, poses_between(0, 30), 30.0)
        self.assertEqual(s['fitness_over_effective_threshold_count'], 1)
        self.assertEqual(s['not_converged_count'], 1)
        self.assertEqual(s['consecutive_rejected_max'], 3)
        self.assertEqual(s['reinitialization_requested_count'], 1)
        self.assertEqual(s['recovery_state_counts']['degraded'], 1)
        self.assertEqual(s['failure_category_counts']['bad_match'], 1)

    def test_by_motion(self):
        samples = steady(0, 10, fitness=0.01)
        for x in samples[50:]:
            x.motion = 'moving'
        for x in samples[:50]:
            x.motion = 'stationary'
        s = cl.summarize(samples, poses_between(0, 10), 10.0)
        self.assertEqual(s['fitness_by_motion']['moving']['count'], 50)
        self.assertEqual(s['fitness_by_motion']['stationary']['count'], 50)
        self.assertEqual(s['fitness_by_motion']['unknown']['count'], 0)


class OutputTest(unittest.TestCase):
    def test_write_outputs_is_valid_json_and_csv(self):
        samples = steady(0, 20) + [sample(20.0, fitness=math.nan)]
        poses = poses_between(0, 20)
        s = cl.summarize(samples, poses, 20.5)
        v, reasons = cl.judge(s, 'campus-m8', [])
        s.update({'verdict': v, 'reasons': reasons})
        s = cl.json_safe(s)
        with tempfile.TemporaryDirectory() as d:
            cl.write_outputs(Path(d), samples, poses, s, 1.7e9)
            loaded = json.loads((Path(d) / 'summary.json').read_text())
            self.assertEqual(loaded['alignment_count'], len(samples))
            lines = (Path(d) / 'samples.csv').read_text().splitlines()
            self.assertEqual(len(lines), len(samples) + 1)
            self.assertIn('fitness_score', lines[0])
            self.assertIn('nan', lines[-1])
            plines = (Path(d) / 'poses.csv').read_text().splitlines()
            self.assertEqual(len(plines), len(poses) + 1)

    def test_map_provenance_mismatch_warns(self):
        with tempfile.TemporaryDirectory() as d:
            site = Path(d) / 'x'
            site.mkdir()
            (site / 'static.pcd').write_bytes(b'abc')
            (site / 'metadata.yaml').write_text('static_pcd_sha256: "' + '0' * 64 + '"\n')
            info, warns = cl.map_provenance(Path(d), 'x')
            self.assertFalse(info['sha256_match'])
            self.assertEqual(len(warns), 1)
            info, warns = cl.map_provenance(Path(d), 'missing')
            self.assertIsNone(info['static_pcd_sha256'])
            self.assertEqual(len(warns), 1)

    def test_help_runs_without_ros(self):
        r = subprocess.run([sys.executable, str(SCRIPTS / 'check_localization.py'), '--help'],
                           capture_output=True, text=True, timeout=10)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('--duration', r.stdout)

    def test_negative_duration_rejected(self):
        with self.assertRaises(SystemExit):
            cl.parse_args(['--duration', '-1'])


if __name__ == '__main__':
    unittest.main()

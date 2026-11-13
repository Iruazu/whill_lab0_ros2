"""ROS・車体を起動せず、引数と所有プロセスの終了を検証する。"""
import hashlib
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'start_dispatch_demo.sh'


class LauncherTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run(['bash', str(SCRIPT), *args], text=True,
                              capture_output=True, timeout=5)

    def test_default_commands(self):
        # 既定は campus-m8 の候補の地図（2026-11-13、#190）。3 つの launch に同じ site を渡す。
        result = self.run_cli('--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('m6r_bringup_launch.py site:=campus-m8\n', result.stdout)
        self.assertIn('nav_launch.py site:=campus-m8 map_variant:=candidate speed:=normal',
                      result.stdout)
        self.assertIn('dispatch_launch.py site:=campus-m8 use_mock:=false use_tls:=true',
                      result.stdout)
        self.assertIn('rviz2', result.stdout)

    def test_fallback_campus_uses_v2(self):
        result = self.run_cli('--dry-run', '--site', 'campus')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('m6r_bringup_launch.py site:=campus\n', result.stdout)
        self.assertIn('nav_launch.py site:=campus map_variant:=v2 speed:=normal', result.stdout)
        self.assertIn('dispatch_launch.py site:=campus use_mock:=false', result.stdout)
        self.assertNotIn('campus-m8', result.stdout)

    def test_variant_override(self):
        result = self.run_cli('--dry-run', '--variant', 'v2')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('nav_launch.py site:=campus-m8 map_variant:=v2', result.stdout)

    def test_custom_environment_and_preflight(self):
        result = self.run_cli('--dry-run', '--domain', '021', '--preflight')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('ROS_DOMAIN_ID=21', result.stdout)
        self.assertIn('timeout 180', result.stdout)
        self.assertNotIn('ros2 launch', result.stdout)

    def test_no_rviz_and_speed(self):
        result = self.run_cli('--dry-run', '--speed', 'slow', '--no-rviz')
        self.assertEqual(result.returncode, 0)
        self.assertIn('speed:=slow', result.stdout)
        self.assertNotIn('rviz2', result.stdout)

    def test_invalid_arguments(self):
        for args in [('--speed', 'invalid'), ('--site', '../campus'),
                     ('--variant', '../v2'), ('--variant',),
                     ('--domain', '233'), ('--domain', '-1'),
                     ('--domain', 'text'), ('--site',), ('--bad',)]:
            with self.subTest(args=args):
                self.assertNotEqual(self.run_cli('--dry-run', *args).returncode, 0)

    def check_static_pcd(self, maps_root, site):
        # check_static_pcd だけを呼ぶ。ROS・機器の検査（check_requirements の他の部分）は通らない。
        code = f'''source {shlex.quote(str(SCRIPT))}
check_static_pcd {shlex.quote(str(maps_root))} {shlex.quote(site)}
'''
        return subprocess.run(['bash', '-c', code], text=True, capture_output=True, timeout=10)

    def make_site(self, root, pcd=None, sha=None, source=None):
        site = Path(root) / 'site'
        site.mkdir()
        meta = {}
        if sha is not None:
            meta['static_pcd_sha256'] = sha
        if source is not None:
            meta['dufomap_output'] = source
        (site / 'metadata.yaml').write_text(''.join(f'{k}: "{v}"\n' for k, v in meta.items()))
        if pcd is not None:
            (site / 'static.pcd').write_bytes(pcd)
        return site

    def test_static_pcd_missing_prints_copy_command(self):
        with tempfile.TemporaryDirectory() as directory:
            site = self.make_site(directory, sha='ab' * 32, source='/master/static.pcd')
            result = self.check_static_pcd(directory, 'site')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(f'cp /master/static.pcd {site / "static.pcd"}', result.stderr)
            self.assertIn('sha256sum -c', result.stderr)
            self.assertIn('README.md', result.stderr)

    def test_static_pcd_sha256_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            self.make_site(directory, pcd=b'other map', sha=hashlib.sha256(b'map').hexdigest(),
                           source='/master/static.pcd')
            result = self.check_static_pcd(directory, 'site')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('sha256 が metadata.yaml と違います', result.stderr)
            self.assertIn(hashlib.sha256(b'other map').hexdigest(), result.stderr)

    def test_static_pcd_sha256_match_and_unrecorded_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            self.make_site(directory, pcd=b'map', sha=hashlib.sha256(b'map').hexdigest())
            result = self.check_static_pcd(directory, 'site')
            self.assertEqual(result.returncode, 0, result.stderr)
        # 旧 campus のように sha256 の記録が無い site は、ファイルの有無だけを見る。
        with tempfile.TemporaryDirectory() as directory:
            self.make_site(directory, pcd=b'map')
            result = self.check_static_pcd(directory, 'site')
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_child_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            code = f'''source {shlex.quote(str(SCRIPT))}
LOG_DIR={shlex.quote(directory)}
exec 9>/dev/null
start_process failed bash -c 'echo expected-failure; exit 7'
sleep 0.2
check_children
'''
            result = subprocess.run(['bash', '-c', code], text=True,
                                    capture_output=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('expected-failure', result.stderr)

    def test_initial_pose_precedes_nav2_activation_wait(self):
        code = f'''source {shlex.quote(str(SCRIPT))}
start_process() {{ printf 'start %s\\n' "$1"; }}
wait_active() {{ printf 'wait %s\\n' "$1"; }}
confirm_initial_pose() {{ printf 'initial-pose\\n'; }}
initialize_navigation
'''
        result = subprocess.run(['bash', '-c', code], text=True,
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.splitlines(), [
            'start rviz', 'wait /lidar_localization', 'wait /map_server',
            'initial-pose', 'wait /planner_server', 'wait /controller_server',
            'wait /bt_navigator', 'wait /velocity_smoother'])

    def test_process_group_shutdown_includes_descendant(self):
        with tempfile.TemporaryDirectory() as directory:
            child = Path(directory) / 'process.py'
            marker = Path(directory) / 'stopped'
            child.write_text('''import signal, subprocess, sys, time
p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
def stop(*_):
    p.terminate()
    p.wait()
    open(sys.argv[1], 'w').write('stopped')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
while True: time.sleep(.1)
''')
            code = f'''source {shlex.quote(str(SCRIPT))}
LOG_DIR={shlex.quote(directory)}
exec 9>/dev/null
start_process worker python3 {shlex.quote(str(child))} {shlex.quote(str(marker))}
for attempt in {{1..30}}; do
    if grep -q ready "$LOG_DIR/worker.log"; then break; fi
    sleep .1
done
stop_group "${{PIDS[0]}}"
if kill -0 -- "-${{PIDS[0]}}" 2>/dev/null; then exit 1; fi
'''
            result = subprocess.run(['bash', '-c', code], text=True,
                                    capture_output=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(marker.exists())


if __name__ == '__main__':
    unittest.main()

"""ROS・車体を起動せず、引数と所有プロセスの終了を検証する。"""
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
        result = self.run_cli('--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('map_variant:=v2 speed:=normal', result.stdout)
        self.assertIn('use_mock:=false use_tls:=true', result.stdout)
        self.assertIn('rviz2', result.stdout)

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
                     ('--domain', '233'), ('--domain', '-1'),
                     ('--domain', 'text'), ('--site',), ('--bad',)]:
            with self.subTest(args=args):
                self.assertNotEqual(self.run_cli('--dry-run', *args).returncode, 0)

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

"""ROS・車体を起動せず、引数と所有プロセスの終了を検証する。"""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
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


# 終了経路の試験（#194）。ros2 launch の代役として、子を持つ無害な Python を使う。
# 代役は ros2 launch と同じく「SIGINT を受けたら子へ転送し、子の終了後に抜ける」。
# SIGINT を無視した状態で起動されると何もしない点も実物と同じ（Python は継承した
# SIG_IGN を保つ）。
WORKER = '''import os, signal, subprocess, sys, time
child = subprocess.Popen([sys.executable, '-c', """
import signal, time
signal.signal(signal.SIGINT, lambda *_: (time.sleep(%(delay)s), exit(0)))
signal.signal(signal.SIGTERM, lambda *_: exit(0))
while True: time.sleep(.1)
"""])
if signal.getsignal(signal.SIGINT) is signal.SIG_IGN:
    print('sigint-ignored', flush=True)
else:
    def stop(*_):
        child.send_signal(signal.SIGINT)
        child.wait()
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
print('ready', child.pid, flush=True)
while True: time.sleep(.1)
'''


class TeardownTests(unittest.TestCase):
    """main と同じ install_traps / start_process / 監視ループで、残りが出ないことを確かめる。"""

    def start(self, directory, delay=0, stdout=subprocess.PIPE):
        worker = Path(directory) / 'worker.py'
        worker.write_text(WORKER % {'delay': delay})
        pids = Path(directory) / 'pids'
        code = f'''set -eo pipefail
source {shlex.quote(str(SCRIPT))}
LOG_DIR={shlex.quote(directory)}
exec 9>/dev/null
install_traps
start_process bringup python3 {shlex.quote(str(worker))}
start_process viz python3 {shlex.quote(str(worker))}
for attempt in {{1..50}}; do
    if grep -q ready "$LOG_DIR/bringup.log" && grep -q ready "$LOG_DIR/viz.log"; then break; fi
    sleep .1
done
echo "${{PIDS[*]}}" > {shlex.quote(str(pids))}
while :; do check_children; sleep 1; done
'''
        # 実運用の対話シェルと同じく、スクリプト自身を process group の先頭にする。
        proc = subprocess.Popen(['bash', '-c', code], text=True, stdout=stdout,
                                stderr=subprocess.PIPE, start_new_session=True)
        for _ in range(100):
            if pids.exists() and pids.read_text().strip():
                break
            time.sleep(.1)
        if not pids.exists():
            proc.kill()
            _, err = proc.communicate()
            self.fail(f'起動中にスクリプトが終了した: {err}')
        sids = pids.read_text().split()
        self.addCleanup(self.kill_sessions, sids)
        self.assertEqual(len(sids), 2, sids)
        return proc, sids

    @staticmethod
    def ctrl_c(proc):
        # 端末の Ctrl-C と同じく、スクリプトの process group 全体（前景の sleep を含む）へ送る。
        os.killpg(proc.pid, signal.SIGINT)

    @staticmethod
    def session_members(sids):
        result = subprocess.run(['pgrep', '-s', ','.join(sids)], text=True,
                                capture_output=True)
        # 0=有り・1=無し。それ以外は数えられていないので、空を「残り無し」と読まない。
        assert result.returncode in (0, 1), result.stderr
        return result.stdout.split()

    def kill_sessions(self, sids):
        for pid in self.session_members(sids):
            try:
                os.kill(int(pid), signal.SIGKILL)
            except OSError:
                pass

    def assert_clean_exit(self, proc, sids, code):
        _, err = proc.communicate(timeout=40)
        self.assertEqual(proc.returncode, code, err)
        self.assertEqual(self.session_members(sids), [], err)

    def test_launch_does_not_inherit_ignored_sigint(self):
        with tempfile.TemporaryDirectory() as directory:
            proc, sids = self.start(directory)
            log = (Path(directory) / 'bringup.log').read_text()
            self.assertNotIn('sigint-ignored', log)
            self.assertIn('ready', log)
            self.ctrl_c(proc)
            self.assert_clean_exit(proc, sids, 130)

    def test_sigterm_stops_all_sessions(self):
        with tempfile.TemporaryDirectory() as directory:
            proc, sids = self.start(directory)
            proc.terminate()
            self.assert_clean_exit(proc, sids, 143)

    def test_hangup_with_unwritable_terminal_stops_all_sessions(self):
        # 端末が閉じた後は表示が失敗する。/dev/full への書き込み失敗で同じ状況を作る。
        with tempfile.TemporaryDirectory() as directory, open('/dev/full', 'w') as full:
            proc, sids = self.start(directory, stdout=full)
            proc.send_signal(signal.SIGHUP)
            self.assert_clean_exit(proc, sids, 129)

    def test_closed_pipe_stdout_does_not_abort_cleanup(self):
        # `| tee demo.log` の tee が Ctrl-C で先に終わった状況。最初の表示が SIGPIPE になる。
        with tempfile.TemporaryDirectory() as directory:
            proc, sids = self.start(directory)
            proc.stdout.close()
            proc.stdout = None
            self.ctrl_c(proc)
            self.assert_clean_exit(proc, sids, 130)

    def test_second_interrupt_does_not_abort_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            proc, sids = self.start(directory, delay=2)
            self.ctrl_c(proc)
            time.sleep(1.5)  # 1本目の停止待ちの最中にもう一度 Ctrl-C
            self.ctrl_c(proc)
            self.assert_clean_exit(proc, sids, 130)

    def test_stop_group_reaches_process_that_left_group(self):
        # process group を抜けて SIGINT も無視する孫は、group 宛ての kill では届かず、
        # 親が先に終わると旧実装は「group が空」と判断して残した。
        with tempfile.TemporaryDirectory() as directory:
            stray = Path(directory) / 'stray.py'
            stray.write_text('''import subprocess, sys, time
subprocess.Popen([sys.executable, '-c', """
import os, signal, time
os.setpgid(0, 0)
signal.signal(signal.SIGINT, signal.SIG_IGN)
while True: time.sleep(.1)
"""])
time.sleep(.5)
print('ready', flush=True)
while True: time.sleep(.1)
''')
            code = f'''source {shlex.quote(str(SCRIPT))}
LOG_DIR={shlex.quote(directory)}
exec 9>/dev/null
start_process stray python3 {shlex.quote(str(stray))}
for attempt in {{1..50}}; do
    if grep -q ready "$LOG_DIR/stray.log"; then break; fi
    sleep .1
done
echo "sid ${{PIDS[0]}}"
cleanup
'''
            result = subprocess.run(['bash', '-c', code], text=True,
                                    capture_output=True, timeout=30)
            sid = result.stdout.split('sid ', 1)[1].split()[0]
            self.addCleanup(self.kill_sessions, [sid])
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.session_members([sid]), [])

    def run_with_worker(self, directory, body):
        worker = Path(directory) / 'worker.py'
        worker.write_text(WORKER % {'delay': 0})
        code = f'''source {shlex.quote(str(SCRIPT))}
LOG_DIR={shlex.quote(directory)}
exec 9>/dev/null
start_process worker python3 {shlex.quote(str(worker))}
for attempt in {{1..50}}; do
    if grep -q ready "$LOG_DIR/worker.log"; then break; fi
    sleep .1
done
echo "sid ${{PIDS[0]}}"
{body}
'''
        result = subprocess.run(['bash', '-c', code], text=True,
                                capture_output=True, timeout=30)
        sid = result.stdout.split('sid ', 1)[1].split()[0]
        self.addCleanup(self.kill_sessions, [sid])
        return result, sid

    def test_sweep_reports_and_stops_leftover(self):
        # stop_group を通らずに残ったものを、名前を出して止める。
        with tempfile.TemporaryDirectory() as directory:
            result, sid = self.run_with_worker(directory, 'sweep_leftovers; echo "rc=$?"')
            self.assertIn('残っているプロセスを停止します', result.stdout)
            self.assertIn('worker.py', result.stdout)
            self.assertIn('rc=0', result.stdout)
            self.assertEqual(self.session_members([sid]), [])

    def test_cleanup_fails_when_leftover_cannot_be_stopped(self):
        # 止められなかったときは非0で終わり、「終了しました」を出さない。
        with tempfile.TemporaryDirectory() as directory:
            result, sid = self.run_with_worker(directory, 'stop_group() { :; }\ncleanup')
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn('停止できないプロセスがあります', result.stderr)
            self.assertIn(f'pgrep -a -s {sid}', result.stderr)
            self.assertNotIn('終了しました', result.stdout)
            self.assertNotEqual(self.session_members([sid]), [])


if __name__ == '__main__':
    unittest.main()

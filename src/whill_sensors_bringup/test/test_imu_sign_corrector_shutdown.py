# Copyright (c) 2026 Iruazu / Utsunomiya University Systems Lab
# SPDX-License-Identifier: BSD-3-Clause
"""imu_sign_corrector の終了処理 (Issue #169)。

ros2 launch は Ctrl-C で各プロセスに SIGINT を送り、残れば SIGTERM、
最後に SIGKILL を送る。どちらのシグナルでも、トレースバック無しに
exit 0 で、すぐ終わることを確かめる。

修正前の失敗は競合なので、1回では出ないことがある (2026-11-12 の実測で
SIGINT 100 回中 36 回にトレースバック)。各条件を REPEAT 回繰り返す。
シグナルは準備完了ログの直後に送る。起動直後の窓がいちばん競合しやすい。
IMU の入力は流さない。入力が無いと spin が C の wait から戻らず、
SIGINT で終われなくなる失敗も、終了待ちのタイムアウトで捕まえられる。

実機 PC で colcon test を直接回しても、走行中のスタックに混ざらないように
する。子プロセスは専用の ROS_DOMAIN_ID と ROS_LOCALHOST_ONLY=1 で起動し、
さらに topic を /test_169/ 以下に remap する (どちらか一方が漏れても、
本物の /imu/data_raw を購読して /imu/data_rep145 を EKF に流さない)。
"""

import contextlib
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

NODE = (Path(__file__).resolve().parents[1]
        / 'whill_sensors_bringup' / 'imu_sign_corrector.py')
READY = 'republishing /imu/data_raw'
# run_tests.sh の 137 や回帰チェックの 142 とも分ける。
TEST_DOMAIN_ID = '169'
REMAPS = ['--ros-args',
          '-r', '/imu/data_raw:=/test_169/imu/data_raw',
          '-r', '/imu/data_rep145:=/test_169/imu/data_rep145']
# 起動確認の上限。DDS の初期化を含めても通常は 1 秒未満。
READY_TIMEOUT_S = 15.0
# launch の既定は SIGINT から SIGTERM まで 5 秒待つ。それより十分短く終わる
# ことを求める。
EXIT_TIMEOUT_S = 3.0
# 1回 0.3 秒程度。3条件 x 15 回で 15 秒前後に収まる。
REPEAT = 15


def _ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


class _Node:
    """起動したノード1つ。出力は別スレッドで読む。

    readline() を主スレッドで呼ぶと、ノードが何も出さないときに永久に
    止まり、起動確認の期限が効かない (ctest の 120 秒で pytest だけが殺され、
    ノードが残る)。
    """

    def __init__(self, sigint_ignored):
        env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN_ID,
                   ROS_LOCALHOST_ONLY='1')
        self.proc = subprocess.Popen(
            [sys.executable, str(NODE)] + REMAPS,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            env=env, start_new_session=True,
            preexec_fn=_ignore_sigint if sigint_ignored else None)
        self.lines = []
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.append(line)

    def output(self):
        return ''.join(self.lines)

    def wait_ready(self):
        deadline = time.monotonic() + READY_TIMEOUT_S
        while time.monotonic() < deadline:
            if READY in self.output():
                return
            if self.proc.poll() is not None:
                break
            time.sleep(0.01)
        pytest.fail('node did not become ready:\n' + self.output())

    def wait_exit(self, sig):
        """終了を待ち、出力全体を返す。期限切れなら失敗にする。"""
        try:
            self.proc.wait(timeout=EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            pytest.fail(f'still running {EXIT_TIMEOUT_S}s after {sig.name}:\n'
                        + self.output())
        self._reader.join(timeout=EXIT_TIMEOUT_S)
        return self.output()

    def kill(self):
        # セッションごと消す。テストが途中で失敗しても子を残さない。
        if self.proc.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait()


@contextlib.contextmanager
def _started(sigint_ignored=False):
    node = _Node(sigint_ignored)
    try:
        node.wait_ready()
        yield node
    finally:
        node.kill()


# sigint_ignored: 親から SIGINT 無視を継いだ場合 (非対話 bash の & 起動など)。
# 修正前は、この継いだハンドラの違いで spin から上がる例外が変わった
# (KeyboardInterrupt / ExternalShutdownException)。修正後はノードが自分の
# ハンドラを入れるので、どちらでも同じ経路で終わるはず。
@pytest.mark.parametrize('sig,sigint_ignored', [
    (signal.SIGINT, False),
    (signal.SIGINT, True),
    (signal.SIGTERM, False),
])
def test_exits_cleanly_on_signal(sig, sigint_ignored):
    for i in range(REPEAT):
        with _started(sigint_ignored) as node:
            node.proc.send_signal(sig)
            out = node.wait_exit(sig)
            assert 'Traceback' not in out, f'run {i}:\n{out}'
            # 負の値はシグナルで死んだこと (SIGSEGV なら -11)。
            assert node.proc.returncode == 0, f'run {i}:\n{out}'


def test_restarts_after_clean_exit():
    # 同じノード名で続けて起動できること。前のプロセスの後始末が残ると
    # ここで起動確認か終了が失敗する。
    for _ in range(2):
        with _started() as node:
            node.proc.send_signal(signal.SIGINT)
            out = node.wait_exit(signal.SIGINT)
            assert node.proc.returncode == 0, out

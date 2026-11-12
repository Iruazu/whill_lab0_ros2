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

ノードを別プロセスで起動するので ROS の環境が要る。scripts/run_tests.sh は
ROS_DOMAIN_ID と ROS_LOCALHOST_ONLY を分けてから colcon test を回す。
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

NODE = (Path(__file__).resolve().parents[1]
        / 'whill_sensors_bringup' / 'imu_sign_corrector.py')
READY = 'republishing /imu/data_raw'
# 起動確認の上限。DDS の初期化を含めても通常は 1 秒未満。
READY_TIMEOUT_S = 15.0
# launch の既定は SIGINT から SIGTERM まで 5 秒待つ。それより十分短く終わる
# ことを求める。
EXIT_TIMEOUT_S = 3.0
# 1回 0.3 秒程度。3条件 x 15 回で 15 秒前後に収まる。
REPEAT = 15


def _ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def _start(sigint_ignored=False):
    proc = subprocess.Popen(
        [sys.executable, str(NODE)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=dict(os.environ),
        preexec_fn=_ignore_sigint if sigint_ignored else None)
    deadline = time.monotonic() + READY_TIMEOUT_S
    lines = []
    while time.monotonic() < deadline:
        line = proc.stdout.readline()
        if not line:
            break
        lines.append(line)
        if READY in line:
            return proc, lines
    proc.kill()
    proc.wait()
    pytest.fail('node did not become ready:\n' + ''.join(lines))


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
        proc, lines = _start(sigint_ignored)
        proc.send_signal(sig)
        try:
            rest, _ = proc.communicate(timeout=EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            rest, _ = proc.communicate()
            pytest.fail(f'run {i}: still running {EXIT_TIMEOUT_S}s '
                        f'after {sig.name}')
        out = ''.join(lines) + rest
        assert 'Traceback' not in out, f'run {i}:\n{out}'
        # 負の値はシグナルで死んだこと (SIGSEGV なら -11)。
        assert proc.returncode == 0, f'run {i}:\n{out}'


def test_restarts_after_clean_exit():
    # 同じノード名で続けて起動できること。前のプロセスの後始末が残ると
    # ここで起動確認か終了が失敗する。
    for _ in range(2):
        proc, _ = _start()
        proc.send_signal(signal.SIGINT)
        proc.communicate(timeout=EXIT_TIMEOUT_S)
        assert proc.returncode == 0

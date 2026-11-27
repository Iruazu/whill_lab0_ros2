#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""check_localization.py — 実機の NDT localizer の状態を見続け、記録して合否を出す（#156）。

runbook 8 節の `/alignment_status` の `--once` は 1 回の値しか見ない。屋外の受入では、初期位置を
与えてから止まっている間・人が手で走らせている間ずっと、failsafe の Layer B に掛かる状態が
1 度も出なかったかを確かめたい。このスクリプトは `/alignment_status`・`/pcl_pose`・`/whill/odom`
を受けるだけで、何も publish しない（/initialpose も /cmd_vel も出さない）。走っている
bringup に別ターミナルから足しても、車体の動きにも localizer にも影響しない。

使い方（runbook 3 節の共通環境を設定したターミナルで。bringup は別ターミナルで動いている前提）:
    cd ~/whill_lab0_ros2
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${PWD}/configs/cyclonedds-runtime-min.xml"
    export ROS_DOMAIN_ID=0
    python3 scripts/check_localization.py --duration 600
    # Ctrl-C まで続ける。出力先を指定する
    python3 scripts/check_localization.py --duration 0 --out log/loc-seimon

判定（summary.json の verdict と終了コード）:
  FAIL（1）: 初期位置の後に、failsafe の Layer B が止める状態が 1 度でもあった。
             fitness > FITNESS_MAX が FITNESS_WINDOW_S 続いた・has_converged=false が
             FITNESS_WINDOW_S 続いた・/pcl_pose の間隔が PCL_POSE_TIMEOUT_S を超えた、のどれか。
             初期位置の後に /pcl_pose が 1 度も来なかった場合も含む。
  WARN（0）: FAIL ではないが、fitness の p95 が 07-24 の bag の NDT 再生（参考値）より大きく
             悪い、static.pcd が metadata.yaml の sha256 と違う、RMW が CycloneDDS でない、
             判定した区間が短い、のどれか。
  PASS（0）: 上のどれでもない。
  NOT_INITIALIZED（2）: 終わるまで初期位置が与えられなかった（initialpose_received が true に
             ならず、/pcl_pose も来なかった）。失敗ではないので FAIL と分ける。
  NO_DATA（2）: /alignment_status も /pcl_pose も 1 件も来なかった。DDS の設定・domain・
             bringup の起動を疑う。
  INCOMPLETE（2）: 初期位置の後の /alignment_status が 1 件も無く、fitness を判定できない。

出力（--out。既定 log/localization-check-<日時>/）:
  samples.csv   /alignment_status の受信ごとの値（キーは文字列のまま。judged は判定に使った行）
  poses.csv     /pcl_pose の受信ごとの x y z yaw と、前の姿勢からの受信の間隔
  summary.json  件数・fitness の分位・Layer B に掛かった区間・pose の間隔・版・site・sha256・判定

前提と限界:
  - 時刻は受信した時刻（このプロセスの monotonic）で数える。failsafe も受信時刻で判定する
    （_now_s）ので同じ物差しになる。DDS の遅れで、localizer の stamp の間隔とは少し違う。
  - failsafe は 20 Hz の tick で判定するので、間隔が 1.0 s をわずかに超えただけなら止まらない
    こともある。ここでは tick を待たず、超えたら FAIL とする（安全側）。
  - 実際に localizer が読んだ地図は bringup を起動した checkout の docs/maps/<site>/static.pcd
    （または WHILL_MAPS_ROOT）。このスクリプトが sha256 を取るのは --maps-root の側なので、
    別の checkout で bringup を起動していると一致しないことがある。
  - fitness が小さいことは点群が地図に重なることを示すだけで、位置が正しいことの証明ではない
    （docs/maps/campus-m8/README.md 4 節の最後の項）。地点ごとの位置の照合は別に行う。

--help だけなら ROS は要らない。rclpy は main の中で import する（単体テストが ROS なしで
判定と集計の関数を import できるようにするため）。
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass

REPO = pathlib.Path(__file__).resolve().parents[1]

# failsafe の Layer B の定数の写し。出典は src/whill_safety/whill_safety/failsafe_node.py の
# モジュール先頭（FITNESS_MAX / FITNESS_WINDOW_S / PCL_POSE_TIMEOUT_S）。import しないのは、
# whill_safety を import すると rclpy と install の source が要り、ROS なしの単体テストと
# --help が回らなくなるため。値がずれると「failsafe なら止まる状態を PASS にする」ことに
# なるので、scripts/tests/test_check_localization.py が failsafe_node.py を ast で読んで
# 一致を確かめる。
FITNESS_MAX = 1.0
FITNESS_WINDOW_S = 2.0
PCL_POSE_TIMEOUT_S = 1.0

# 既定の site は m6r_bringup_launch.py の _DEFAULT_SITE と同じ（#190。テストで一致を確かめる）。
DEFAULT_SITE = 'campus-m8'

# 参考値は docs/maps/campus-m8/README.md 4 節の NDT 再生（07-24 の bag の Hi-Res 補正版、
# #145 の TF、campus-m8）。2026-11-12 と 2026-11-13 の 240 秒・741 秒の 4 本（11-12 の表の
# 2 行と、11-13 の b・c・d）の幅をそのまま写した。中央値 0.0154〜0.0163、p95 0.038〜0.065、
# 最大 0.62〜0.66、姿勢の間隔の最大 0.81〜0.91 s、閾値超え・収束しなかった回はどれも 0。
# 再生は走行中だけで、7 月の記録（駐車の車や植栽は今と違う）。各条件 1 回ずつで、実行ごとの
# ばらつきは評価していない（同 README）。ここでは合否には使わず、WARN と並べて見るためだけ。
REFERENCE = {
    'campus-m8': {
        'source': 'docs/maps/campus-m8/README.md 4 節（NDT 再生、07-24 bag Hi-Res、#145 TF、'
                  '2026-11-12 / 2026-11-13 の 240 s・741 s）',
        'fitness_p50': [0.0154, 0.0163],
        'fitness_p95': [0.038, 0.065],
        'fitness_max': [0.62, 0.66],
        'pose_gap_max_s': [0.81, 0.91],
        'fitness_over_max_count': 0,
        'not_converged_count': 0,
    },
}
# p95 が参考値の上端の 3 倍（campus-m8 で 0.195）を超えたら WARN。3 倍は決め打ち。目安にした
# のは、同じ再生で旧 campus・#145 の TF・Hi-Res の 240 秒の p95 が 0.195 だったこと
# （同 README の 11-12 の表）。そこまで悪いと、作り直した地図の利点（p95 0.3 倍）が消えて
# いる。実行ごとのばらつきは測っていないので、2 倍程度の差はばらつきと区別できないと見て、
# それより広く取った。屋外 #156 の記録が溜まったら決め直す。
WARN_P95_FACTOR = 3.0
# 判定した区間（初期位置の後）がこれより短いと、PASS でも WARN にする。FITNESS_WINDOW_S の
# 5 倍で、Layer B の窓を何度か見たと言える最低限として決め打ちした。
MIN_JUDGED_S = 5.0 * FITNESS_WINDOW_S

# /whill/odom で静止と走行を分ける。WHILL の odom は 2.5 Hz（ekf_odom.yaml のコメント）なので、
# 1.0 s（2.5 周期）来なければ状態不明にする。速さの閾値は決め打ち: nav の最低速 slow が
# 0.30 m/s（nav_launch.py の speed_caps）なので、その 1/6 の 0.05 m/s・0.05 rad/s 未満を静止と
# 見る。静止時の odom の雑音は測っていない。
ODOM_STALE_S = 1.0
STATIONARY_V_MPS = 0.05
STATIONARY_W_RADPS = 0.05

# samples.csv に残す KeyValue。キーの正本は src/third_party/lidar_localization_ros2/include/
# lidar_localization/alignment_diagnostics_policy.hpp の buildAlignmentDiagnosticValues。
# 全キー（約 60）を毎回書くと 10 Hz で重いので、判定・切り分けに使うものに絞った。
ALIGNMENT_KEYS = (
    'fitness_score', 'has_converged', 'score_threshold', 'effective_score_threshold',
    'consecutive_rejected_updates', 'recovery_state', 'recovery_action', 'failure_category',
    'initialpose_received', 'map_received', 'correction_translation_m', 'correction_yaw_deg',
    'accepted_gap_sec', 'alignment_time_sec', 'filtered_point_count',
    'reinitialization_requested', 'reinitialization_request_reason', 'registration_seed_source',
)

VERDICT_EXIT = {'PASS': 0, 'WARN': 0, 'FAIL': 1,
                'NOT_INITIALIZED': 2, 'NO_DATA': 2, 'INCOMPLETE': 2}


def to_float(s: str | None) -> float | None:
    """failsafe_node.py の _to_float と同じ。'nan' は nan になる（上流は失敗時に NaN を書く）。"""
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def to_bool(s: str | None) -> bool | None:
    """failsafe_node.py の _to_bool と同じ解釈。違うと、failsafe と判定が食い違う。"""
    if s is None:
        return None
    lo = str(s).strip().lower()
    if lo in ('true', '1', 'ok'):
        return True
    if lo in ('false', '0'):
        return False
    return None


def to_int(s: str | None) -> int | None:
    v = to_float(s)
    if v is None or not math.isfinite(v):
        return None
    return int(v)


@dataclass
class Sample:
    """/alignment_status の 1 受信。t は計測開始からの受信時刻 [s]。"""
    t: float
    raw: dict[str, str]
    level: int | None = None
    message: str = ''
    motion: str = 'unknown'
    judged: bool = False

    @property
    def fitness(self) -> float | None:
        return to_float(self.raw.get('fitness_score'))

    @property
    def converged(self) -> bool | None:
        return to_bool(self.raw.get('has_converged'))

    @property
    def initialpose_received(self) -> bool | None:
        return to_bool(self.raw.get('initialpose_received'))


@dataclass
class Pose:
    t: float
    stamp: float
    x: float
    y: float
    z: float
    yaw_deg: float
    cov_xx: float = math.nan
    cov_yy: float = math.nan
    cov_yawyaw: float = math.nan
    motion: str = 'unknown'


@dataclass
class Run:
    """Layer B に掛かる状態が続いた区間。end は解けた受信の時刻か、計測の終わり。"""
    start: float
    end: float
    ended_by_stop: bool = False

    @property
    def duration(self) -> float:
        return self.end - self.start


def merge_status_values(statuses: list[tuple[int, str, list[tuple[str, str]]]]) \
        -> tuple[dict[str, str], int | None, str]:
    """DiagnosticArray の全 status の KeyValue を 1 つにまとめる。

    failsafe の _on_alignment と同じく全 status を舐め、同じキーは後の値が勝つ。上流は今
    status を 1 つしか出さないが、並びが変わっても failsafe と同じ値を見るため。
    level と message は最初の status のもの。
    """
    raw: dict[str, str] = {}
    level = None
    message = ''
    for i, (lv, msg, values) in enumerate(statuses):
        if i == 0:
            level, message = lv, msg
        for k, v in values:
            raw[k] = v
    return raw, level, message


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def classify_motion(odom_age_s: float | None, vx: float, wz: float) -> str:
    if odom_age_s is None or odom_age_s > ODOM_STALE_S:
        return 'unknown'
    if abs(vx) < STATIONARY_V_MPS and abs(wz) < STATIONARY_W_RADPS:
        return 'stationary'
    return 'moving'


def percentile(values: list[float], q: float) -> float | None:
    """numpy.percentile の既定（linear）と同じ。空なら None。"""
    xs = sorted(values)
    if not xs:
        return None
    pos = (len(xs) - 1) * q / 100.0
    lo = math.floor(pos)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def armed_time(samples: list[Sample], poses: list[Pose]) -> float | None:
    """初期位置が与えられたとみなす時刻。これより前は判定しない。

    localizer は初期位置の前は scan を処理しない（decideScanAdmission）ので、その間に届く
    状態は「未指定」であって失敗ではない。initialpose_received=true の最初の受信か、最初の
    /pcl_pose の早い方を使う。/pcl_pose を足すのは、キーが無い版の localizer でも始められる
    ようにするため。
    """
    ts = [s.t for s in samples if s.initialpose_received is True]
    if poses:
        ts.append(poses[0].t)
    return min(ts) if ts else None


def bad_runs(samples: list[Sample], end_t: float, is_bad) -> list[Run]:
    """is_bad(sample) が True / False / None（値なし）を返す列から、悪い状態が続いた区間を作る。

    failsafe の _on_alignment と同じ状態遷移: 悪い受信で開始（既に悪ければそのまま）、良い受信
    で解除、値の無い受信では何も変えない。最後まで解けなければ end_t で切る（failsafe は次の
    受信が来ない間も止め続けるので、受信が途絶えても区間は続く扱い）。
    """
    runs: list[Run] = []
    since = None
    for s in samples:
        b = is_bad(s)
        if b is None:
            continue
        if b:
            if since is None:
                since = s.t
        elif since is not None:
            runs.append(Run(since, s.t))
            since = None
    if since is not None:
        runs.append(Run(since, end_t, ended_by_stop=True))
    return runs


def fitness_bad(s: Sample) -> bool | None:
    # NaN は failsafe では `nan > FITNESS_MAX` が False で「良い」側に倒れる。ここも同じに
    # する（Layer B の再現が目的）。NaN の回数は別に数え、has_converged=false で拾う。
    f = s.fitness
    if f is None:
        return None
    return f > FITNESS_MAX


def converged_bad(s: Sample) -> bool | None:
    c = s.converged
    if c is None:
        return None
    return not c


def pose_gaps(poses: list[Pose], end_t: float) -> list[tuple[float, float]]:
    """(前の受信の時刻, 間隔) の列。最後の姿勢から計測の終わりまでも 1 つの間隔に数える。

    failsafe は最初の /pcl_pose から監視を始め、途絶えたら止める。終わりの直前に途絶えた
    場合も止まるはずなので、最後の間隔を落とさない。
    """
    out = []
    for a, b in zip(poses, poses[1:]):
        out.append((a.t, b.t - a.t))
    if poses:
        out.append((poses[-1].t, end_t - poses[-1].t))
    return out


def stats(values: list[float]) -> dict:
    finite = [v for v in values if math.isfinite(v)]
    return {
        'count': len(finite),
        'p50': percentile(finite, 50),
        'p95': percentile(finite, 95),
        'max': max(finite) if finite else None,
    }


def summarize(samples: list[Sample], poses: list[Pose], end_t: float) -> dict:
    """判定の前の集計。samples の judged もここで決める。"""
    t0 = armed_time(samples, poses)
    for s in samples:
        s.judged = t0 is not None and s.t >= t0
    judged = [s for s in samples if s.judged]

    fit_all = [s.fitness for s in judged if s.fitness is not None]
    fit_runs = bad_runs(judged, end_t, fitness_bad)
    conv_runs = bad_runs(judged, end_t, converged_bad)
    gaps = pose_gaps(poses, end_t)
    gap_values = [g for _, g in gaps]
    # 最後の間隔は計測の終わりで切ったもので、受信の間隔ではない。p95 には入れず、判定と
    # 最大にだけ使う。
    inner_gaps = gap_values[:-1]

    rejected = [to_int(s.raw.get('consecutive_rejected_updates')) for s in judged]
    rejected = [r for r in rejected if r is not None]
    eff = [(s.fitness, to_float(s.raw.get('effective_score_threshold'))) for s in judged]

    def count_by(key: str) -> dict[str, int]:
        out: dict[str, int] = {}
        for s in judged:
            v = s.raw.get(key, '')
            out[v] = out.get(v, 0) + 1
        return out

    by_motion = {}
    for m in ('stationary', 'moving', 'unknown'):
        by_motion[m] = stats([s.fitness for s in judged
                              if s.motion == m and s.fitness is not None])

    initialpose_seen = any(s.initialpose_received is True for s in samples)
    return {
        'alignment_count': len(samples),
        'alignment_judged_count': len(judged),
        'pose_count': len(poses),
        'armed_at_s': t0,
        'judged_duration_s': (end_t - t0) if t0 is not None else 0.0,
        'end_s': end_t,
        'initialpose_received': initialpose_seen,
        'map_received_last': samples[-1].raw.get('map_received') if samples else None,
        'fitness': stats(fit_all),
        'fitness_by_motion': by_motion,
        'fitness_nan_count': sum(1 for f in fit_all if math.isnan(f)),
        'fitness_over_max_count': sum(1 for f in fit_all if f > FITNESS_MAX),
        'fitness_over_effective_threshold_count': sum(
            1 for f, th in eff if f is not None and th is not None and f > th),
        'not_converged_count': sum(1 for s in judged if s.converged is False),
        'consecutive_rejected_max': max(rejected) if rejected else None,
        'reinitialization_requested_count': sum(
            1 for s in judged if to_bool(s.raw.get('reinitialization_requested')) is True),
        'recovery_state_counts': count_by('recovery_state'),
        'failure_category_counts': count_by('failure_category'),
        'layer_b_fitness_runs': [vars(r) | {'duration': r.duration} for r in fit_runs],
        'layer_b_converged_runs': [vars(r) | {'duration': r.duration} for r in conv_runs],
        'pose_gap': {
            'count': len(inner_gaps),
            'p95': percentile(inner_gaps, 95),
            'max': max(gap_values) if gap_values else None,
            'tail_s': gap_values[-1] if gap_values else None,
        },
        'pose_gaps_over_timeout': [
            {'after_s': a, 'gap_s': g} for a, g in gaps if g > PCL_POSE_TIMEOUT_S],
    }


def judge(summary: dict, site: str, env_warnings: list[str]) -> tuple[str, list[str]]:
    """summary から判定と理由を返す。理由は表示と summary.json にそのまま出す。"""
    if summary['alignment_count'] == 0 and summary['pose_count'] == 0:
        return 'NO_DATA', ['/alignment_status も /pcl_pose も受信していない']
    if summary['armed_at_s'] is None:
        return 'NOT_INITIALIZED', ['初期位置が未指定（initialpose_received が true にならず、'
                                   '/pcl_pose も来なかった）']

    fails = []
    for r in summary['layer_b_fitness_runs']:
        if r['duration'] >= FITNESS_WINDOW_S:
            fails.append(f'B:fitness  fitness > {FITNESS_MAX} が {r["duration"]:.2f} s 続いた'
                         f'（{r["start"]:.1f} s から）')
    for r in summary['layer_b_converged_runs']:
        if r['duration'] >= FITNESS_WINDOW_S:
            fails.append(f'B:converged  has_converged=false が {r["duration"]:.2f} s 続いた'
                         f'（{r["start"]:.1f} s から）')
    for g in summary['pose_gaps_over_timeout']:
        fails.append(f'B:pcl_pose_silent  /pcl_pose の間隔 {g["gap_s"]:.2f} s > '
                     f'{PCL_POSE_TIMEOUT_S} s（{g["after_s"]:.1f} s の後）')
    if summary['pose_count'] == 0 and summary['judged_duration_s'] > PCL_POSE_TIMEOUT_S:
        # failsafe の startup_grace による never_arrived と同じ状態。初期位置は受けたのに
        # 姿勢を出していない。
        fails.append('B:pcl_pose_never_arrived  初期位置の後に /pcl_pose が 1 件も来ない')
    if fails:
        return 'FAIL', fails

    if summary['alignment_judged_count'] == 0:
        return 'INCOMPLETE', ['初期位置の後の /alignment_status が無く、fitness を判定できない']

    warns = list(env_warnings)
    ref = REFERENCE.get(site)
    p95 = summary['fitness']['p95']
    if ref is not None and p95 is not None:
        limit = WARN_P95_FACTOR * ref['fitness_p95'][1]
        if p95 > limit:
            warns.append(f'fitness p95 {p95:.3f} が参考値の上端 {ref["fitness_p95"][1]} の '
                         f'{WARN_P95_FACTOR:g} 倍（{limit:.3f}）を超えた')
    if summary['judged_duration_s'] < MIN_JUDGED_S:
        warns.append(f'判定した区間が {summary["judged_duration_s"]:.1f} s しかない'
                     f'（{MIN_JUDGED_S:g} s 未満）')
    if warns:
        return 'WARN', warns
    return 'PASS', []


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def expected_pcd_sha256(metadata: pathlib.Path) -> str | None:
    # yaml を読まずに 1 行だけ拾う。metadata.yaml のスキーマ（docs/maps/README.md）で
    # static_pcd_sha256 は最上位の 1 行の文字列。
    try:
        text = metadata.read_text()
    except OSError:
        return None
    m = re.search(r'^static_pcd_sha256:\s*"?([0-9a-f]{64})"?', text, re.M)
    return m.group(1) if m else None


def map_provenance(maps_root: pathlib.Path, site: str) -> tuple[dict, list[str]]:
    pcd = maps_root / site / 'static.pcd'
    expected = expected_pcd_sha256(maps_root / site / 'metadata.yaml')
    info = {'static_pcd': str(pcd), 'static_pcd_sha256': None,
            'expected_sha256': expected, 'sha256_match': None}
    warns = []
    if not pcd.is_file():
        warns.append(f'{pcd} が無い。localizer が読んだ地図を確かめられない')
        return info, warns
    info['static_pcd_sha256'] = sha256_file(pcd)
    if expected is not None:
        info['sha256_match'] = info['static_pcd_sha256'] == expected
        if not info['sha256_match']:
            warns.append('static.pcd の sha256 が metadata.yaml と違う（参考値と比べられない）')
    return info, warns


def git_version() -> dict:
    def run(*args: str) -> str | None:
        try:
            return subprocess.run(['git', '-C', str(REPO), *args], capture_output=True,
                                  text=True, timeout=5, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    return {'commit': run('rev-parse', 'HEAD'),
            'describe': run('describe', '--always', '--dirty', '--abbrev=12'),
            'branch': run('rev-parse', '--abbrev-ref', 'HEAD')}


def ros_env() -> tuple[dict, list[str]]:
    env = {k: os.environ.get(k) for k in
           ('RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')}
    warns = []
    # runbook 3 節の前提。FastDDS 既定では /velodyne_points が詰まった記録があり、bringup と
    # RMW が違うと発見できないこともある。止めはしない（受信できれば値は見られる）。
    if env['RMW_IMPLEMENTATION'] != 'rmw_cyclonedds_cpp':
        warns.append(f'RMW_IMPLEMENTATION={env["RMW_IMPLEMENTATION"]!r}（runbook 3 節は '
                     f'rmw_cyclonedds_cpp）')
    return env, warns


def write_outputs(out: pathlib.Path, samples: list[Sample], poses: list[Pose],
                  summary: dict, wall_t0: float) -> None:
    out.mkdir(parents=True, exist_ok=True)
    with open(out / 'samples.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'unix_time', 'level', 'message', 'motion', 'judged', *ALIGNMENT_KEYS])
        for s in samples:
            w.writerow([f'{s.t:.3f}', f'{wall_t0 + s.t:.3f}', s.level, s.message, s.motion,
                        int(s.judged), *(s.raw.get(k, '') for k in ALIGNMENT_KEYS)])
    gaps = {id(b): b.t - a.t for a, b in zip(poses, poses[1:])}
    with open(out / 'poses.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'unix_time', 'stamp_s', 'x', 'y', 'z', 'yaw_deg',
                    'cov_xx', 'cov_yy', 'cov_yawyaw', 'gap_s', 'motion'])
        for p in poses:
            gap = gaps.get(id(p))
            w.writerow([f'{p.t:.3f}', f'{wall_t0 + p.t:.3f}', f'{p.stamp:.6f}',
                        f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}', f'{p.yaw_deg:.3f}',
                        f'{p.cov_xx:.6g}', f'{p.cov_yy:.6g}', f'{p.cov_yawyaw:.6g}',
                        '' if gap is None else f'{gap:.3f}', p.motion])
    with open(out / 'summary.json', 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2, allow_nan=False,
                  default=_json_default)
        f.write('\n')


def _json_default(o):
    raise TypeError(f'not serializable: {o!r}')


def json_safe(o):
    """NaN / inf を null にする（JSON の仕様外で、読む側の json.load が通らないことがある）。"""
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, dict):
        return {k: json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [json_safe(v) for v in o]
    return o


def fmt(v, spec: str = '.3f', none: str = '-') -> str:
    if v is None:
        return none
    if isinstance(v, float) and math.isnan(v):
        return 'nan'
    return format(v, spec)


def print_summary(summary: dict) -> None:
    f = summary['fitness']
    pg = summary['pose_gap']
    print()
    print('==> 要約')
    print(f'  判定        {summary["verdict"]}')
    for r in summary['reasons']:
        print(f'    - {r}')
    print(f'  受信        alignment {summary["alignment_count"]}（判定 '
          f'{summary["alignment_judged_count"]}）、pose {summary["pose_count"]}、'
          f'判定した区間 {summary["judged_duration_s"]:.1f} s')
    print(f'  初期位置    {"受信済み" if summary["initialpose_received"] else "未指定"}')
    print(f'  fitness     p50 {fmt(f["p50"], ".4f")} / p95 {fmt(f["p95"], ".4f")} / '
          f'max {fmt(f["max"], ".3f")}（{f["count"]} 件）')
    for m, st in summary['fitness_by_motion'].items():
        if st['count']:
            print(f'    {m:<10}  p50 {fmt(st["p50"], ".4f")} / p95 {fmt(st["p95"], ".4f")} / '
                  f'max {fmt(st["max"], ".3f")}（{st["count"]} 件）')
    print(f'  > {FITNESS_MAX}       {summary["fitness_over_max_count"]} 回、'
          f'> effective_score_threshold {summary["fitness_over_effective_threshold_count"]} 回、'
          f'nan {summary["fitness_nan_count"]} 回')
    print(f'  未収束      {summary["not_converged_count"]} 回、reject の最大連続 '
          f'{fmt(summary["consecutive_rejected_max"], "d")}')
    print(f'  pose 間隔   p95 {fmt(pg["p95"])} s / max {fmt(pg["max"])} s')
    ref = summary.get('reference')
    if ref:
        print(f'  参考値      p50 {ref["fitness_p50"]} / p95 {ref["fitness_p95"]} / '
              f'max {ref["fitness_max"]} / pose 間隔の最大 {ref["pose_gap_max_s"]} s')
        print(f'              出典 {ref["source"]}')
    m = summary['map']
    print(f'  static.pcd  {m["static_pcd"]}  sha256 {fmt(m["static_pcd_sha256"], "s")}'
          f'（metadata と一致: {m["sha256_match"]}）')
    print(f'  出力        {summary["out_dir"]}')


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description='NDT localizer の /alignment_status と /pcl_pose を受けて記録し、failsafe '
                    'の Layer B に掛かる状態が無かったかを判定する。何も publish しない。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--duration', type=float, default=0.0,
                   help='計測する秒数。0 は Ctrl-C まで')
    p.add_argument('--out', type=pathlib.Path, default=None,
                   help='出力先。既定は <repo>/log/localization-check-<日時>/')
    p.add_argument('--site', default=DEFAULT_SITE,
                   help='sha256 と参考値を引く site（bringup の site:= と同じにする）')
    p.add_argument('--maps-root', type=pathlib.Path,
                   default=pathlib.Path(os.environ.get('WHILL_MAPS_ROOT') or REPO / 'docs/maps'),
                   help='地図の置き場。bringup と同じく WHILL_MAPS_ROOT を優先する')
    p.add_argument('--odom-topic', default='/whill/odom',
                   help='静止・走行の区別に使う Odometry。空文字で受けない')
    args = p.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        p.error('--duration は 0 以上の有限値')
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    # WARN は stderr に出す。tee やパイプで stdout がブロックバッファになると、表示の順が
    # 前後して、どの確認の WARN か分からなくなるので行ごとに出す。
    sys.stdout.reconfigure(line_buffering=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out = args.out or REPO / 'log' / f'localization-check-{stamp}'

    env, env_warns = ros_env()
    print('==> ROS 環境')
    for k, v in env.items():
        print(f'  {k}={v if v is not None else "(未設定)"}')
    for w in env_warns:
        print(f'WARN: {w}', file=sys.stderr)
    print(f'==> 地図 {args.site} の static.pcd の sha256 を計算する')
    map_info, map_warns = map_provenance(args.maps_root, args.site)
    print(f'  {map_info["static_pcd"]}  {map_info["static_pcd_sha256"]}'
          f'（metadata と一致: {map_info["sha256_match"]}）')
    for w in map_warns:
        print(f'WARN: {w}', file=sys.stderr)

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    # rclpy の SIGINT ハンドラは spin の中で例外にして context を落とすので、落ちた後に
    # 書き出しの前でノードを触れなくなることがある。自前の旗だけ立て、ループを抜けてから
    # 書く（failsafe_node.py と同じく SignalHandlerOptions.NO）。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = {'flag': False}

    def on_signal(signum, _frame):
        stop['flag'] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    # pid を付けるのは、同じ計測を 2 つ並べたときにノード名の重複の警告を出さないため。
    node = rclpy.create_node(f'check_localization_{os.getpid()}')
    # QoS は failsafe_node.py の create_subscription（depth 10 の既定 = RELIABLE / VOLATILE）と、
    # scripts/check_failsafe_inputs.sh の --qos-reliability reliable --qos-durability volatile
    # に揃える。publisher は KeepLast(1)・transient_local・reliable（lidar_localization_component.cpp
    # の pose_pub_ / status_pub_）で、volatile の購読は互換。transient_local で受けると、起動
    # 前の最後の 1 件が今の受信時刻で入り、pose の間隔と未指定の判断を誤るので受けない。
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)

    t_mono0 = time.monotonic()
    wall_t0 = time.time()
    samples: list[Sample] = []
    poses: list[Pose] = []
    odom = {'t': None, 'vx': 0.0, 'wz': 0.0}

    def now() -> float:
        return time.monotonic() - t_mono0

    def motion_now(t: float) -> str:
        age = None if odom['t'] is None else t - odom['t']
        return classify_motion(age, odom['vx'], odom['wz'])

    def on_alignment(msg) -> None:
        t = now()
        raw, level, message = merge_status_values(
            [(st.level if isinstance(st.level, int) else int.from_bytes(st.level, 'little'),
              st.message, [(kv.key, kv.value) for kv in st.values]) for st in msg.status])
        samples.append(Sample(t, raw, level, message, motion_now(t)))

    def on_pose(msg) -> None:
        t = now()
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        poses.append(Pose(t, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                          p.x, p.y, p.z, quat_to_yaw_deg(q.x, q.y, q.z, q.w),
                          c[0], c[7], c[35], motion_now(t)))

    def on_odom(msg) -> None:
        odom['t'] = now()
        odom['vx'] = msg.twist.twist.linear.x
        odom['wz'] = msg.twist.twist.angular.z

    node.create_subscription(DiagnosticArray, '/alignment_status', on_alignment, qos)
    node.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', on_pose, qos)
    if args.odom_topic:
        # whill_node.cpp の odom_pub_ は depth 10 の既定（RELIABLE / VOLATILE）。
        node.create_subscription(Odometry, args.odom_topic, on_odom, qos)

    dur = f'{args.duration:g} s' if args.duration > 0 else 'Ctrl-C まで'
    print(f'==> 計測（{dur}）。何も publish しない。初期位置は RViz の 2D Pose Estimate で与える')
    print('  経過     fitness  conv  rej  recovery/failure            pose 経過(1s 最大)  '
          'x        y        yaw     motion  B')

    next_print = 1.0
    fit_since = None
    conv_since = None
    n_seen = 0
    try:
        while not stop['flag'] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            t = now()
            # 表示用の Layer B の状態。判定は終わりに summarize が全件から作り直す。
            for s in samples[n_seen:]:
                # 初期位置の前の未収束は失敗ではない。表示に持ち越さない。
                if s.initialpose_received is False:
                    fit_since = conv_since = None
                    continue
                fb, cb = fitness_bad(s), converged_bad(s)
                if fb is not None:
                    fit_since = (fit_since if fit_since is not None else s.t) if fb else None
                if cb is not None:
                    conv_since = (conv_since if conv_since is not None else s.t) if cb else None
            n_seen = len(samples)
            if t >= next_print:
                next_print += 1.0
                print(status_line(t, samples, poses, fit_since, conv_since), flush=True)
            if args.duration > 0 and t >= args.duration:
                break
    finally:
        end_t = now()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    summary = summarize(samples, poses, end_t)
    warn_inputs = env_warns + map_warns
    verdict, reasons = judge(summary, args.site, warn_inputs)
    summary.update({
        'verdict': verdict,
        'exit_code': VERDICT_EXIT[verdict],
        'reasons': reasons,
        # NO_DATA などで reasons に入らなかった環境・地図の WARN も残す。
        'warnings': warn_inputs,
        'thresholds': {
            'FITNESS_MAX': FITNESS_MAX, 'FITNESS_WINDOW_S': FITNESS_WINDOW_S,
            'PCL_POSE_TIMEOUT_S': PCL_POSE_TIMEOUT_S,
            'source': 'src/whill_safety/whill_safety/failsafe_node.py（値の写し）',
            'WARN_P95_FACTOR': WARN_P95_FACTOR, 'MIN_JUDGED_S': MIN_JUDGED_S,
        },
        'reference': REFERENCE.get(args.site),
        'site': args.site,
        'map': map_info,
        'git': git_version(),
        'env': env,
        'started_at': datetime.datetime.fromtimestamp(wall_t0).astimezone().isoformat(),
        'duration_arg_s': args.duration,
        'odom_topic': args.odom_topic or None,
        'out_dir': str(out),
    })
    summary = json_safe(summary)
    write_outputs(out, samples, poses, summary, wall_t0)
    print_summary(summary)
    return VERDICT_EXIT[verdict]


def status_line(t: float, samples: list[Sample], poses: list[Pose],
                fit_since: float | None, conv_since: float | None) -> str:
    if not samples and not poses:
        return f'  {t:7.1f}s  未受信（/alignment_status・/pcl_pose がまだ来ない）'
    s = samples[-1] if samples else None
    if armed_time(samples, poses) is None:
        return (f'  {t:7.1f}s  初期位置が未指定。RViz の 2D Pose Estimate を待つ'
                f'（alignment {len(samples)} 件）')
    fit = fmt(s.fitness, '.4f') if s else '-'
    conv = {True: 'true', False: 'FALSE', None: '-'}[s.converged] if s else '-'
    rej = s.raw.get('consecutive_rejected_updates', '-') if s else '-'
    rf = f'{s.raw.get("recovery_state", "-")}/{s.raw.get("failure_category", "-")}' if s else '-'
    if poses:
        p = poses[-1]
        # 10 Hz なら 1 秒は 10 件。全件を毎秒舐めないよう、末尾の 30 件だけ見る。
        tail = poses[-30:]
        recent = [b.t - a.t for a, b in zip(tail, tail[1:]) if b.t > t - 1.0]
        age = t - p.t
        gap = f'{age:5.2f}({fmt(max(recent) if recent else None, "5.2f", "  -  ")})'
        xyz = f'{p.x:8.2f} {p.y:8.2f} {p.yaw_deg:7.1f}'
        motion = p.motion
    else:
        age = None
        gap, xyz, motion = '   -       ', f'{"-":>8} {"-":>8} {"-":>7}', '-'
    layers = []
    if fit_since is not None and t - fit_since >= FITNESS_WINDOW_S:
        layers.append('fitness')
    if conv_since is not None and t - conv_since >= FITNESS_WINDOW_S:
        layers.append('converged')
    if age is not None and age > PCL_POSE_TIMEOUT_S:
        layers.append('pose_silent')
    return (f'  {t:7.1f}s  {fit:>7}  {conv:<5} {rej:>3}  {rf:<26}  {gap:<18}  {xyz}  '
            f'{motion:<7} {",".join(layers) or "-"}')


if __name__ == '__main__':
    sys.exit(main())

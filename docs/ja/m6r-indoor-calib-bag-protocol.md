# Indoor calibration bag 収録プロトコル (M6-R / Issue #64 解消用)

- 日付: 2026-07-08
- 目的: **T_lidar_imu の GRIL-Calib 再校正**を成立させる motion excitation を含む
  bag を取得する
- 呼出元: `docs/ja/plans/2026-06-24-m6r-localization.md` R5 緩和 (drift 主因は
  Issue #64 の T_lidar_imu 未計測回転)
- 根拠: [`docs/ja/m5r-imu-diagnostic.md`](m5r-imu-diagnostic.md) §2 を運用可能な
  形に落とし込んだもの
- 対象 script: [`scripts/m6r_record_calib_bag.sh`](../../scripts/m6r_record_calib_bag.sh) (録画),
  [`scripts/m5r4_run_gril_calib.sh`](../../scripts/m5r4_run_gril_calib.sh) (校正実行)

## 0. なぜ必要か

campus-half-v3 走行 bag (2026-07-07) を GLIM にかけた結果、drift 66 m (14.5%)
+ Z warp 45 m という大 drift が観測された。10 秒平均でも roll / pitch が ±20°
振れており、**estimator が架空姿勢を捏造して LiDAR-IMU 不一致を吸収している**
ことが判明。根本原因は Issue #64 で診断済の T_lidar_imu ~1-2° 回転
miscalibration。

GRIL-Calib は motion-based に T_lidar_imu を推定するが、収束には **全 3 軸の
回転励起 (roll / pitch / yaw) と加減速** が必要。屋外一周では pitch/roll
observability = 0 % で `insufficient_motion` 停止する
(実測: `docs/m5r-bench-data/2026-06-24-loop-outdoor-ext/gril-calib-out/SUMMARY.md`)。
本プロトコルはその observability を満たす形。

## 1. 事前確認 (毎回、外せない)

```bash
# 1. RMW (CycloneDDS)
echo $RMW_IMPLEMENTATION
# → rmw_cyclonedds_cpp でなければ ~/.bashrc の設定を確認

# 2. CPU governor (performance)
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
# → powersave なら:
sudo cpupower frequency-set -g performance
```

RMW と governor は録画品質に直結。過去に `/velodyne_points` 1 Hz 詰まりを
起こした事実あり ([`docs/ja/m5r-rmw-cyclonedds.md`](m5r-rmw-cyclonedds.md))。

## 2. 場所と WHILL 設定

| 項目 | 値 |
|---|---|
| 場所 | 屋内平坦な床、**最少 5 m × 5 m**、障害物なし |
| 環境 | 静的物体のみ (歩行者・他の車両を避ける) |
| WHILL 電源 | フル充電 or 十分残量 |
| WHILL 速度 | **mode 1 (~0.28 m/s) を推奨**。加減速の分解能が上がり GRIL-Calib の観測性が良い |
| ドライバー | 有人ジョイスティック (実機経験者) |
| 録画 duration | **5-10 分** (300-600 秒)。長すぎると batter y と操作者の集中が切れる |

## 3. Motion プロトコル (順番厳守、1 本の bag に連続収録)

以下を順にこなす。**全て済ませて 1 本の bag**。GRIL-Calib は時系列的な
一貫性を見るため、途中で bag を分けない。**セクション間は 2-3 秒のブランク
なら OK** (WHILL 位置調整、心構えの時間); 10 秒以上の長停止は不要。

| # | 動き | 目安時間 | 目的 | 動作の質 |
|---|---|---|---|---|
| P1 | **完全静止** | 20-30 秒 | IMU bias 初期化 | 動かない |
| P2 | **停まらず 8 の字を 3 周以上、直径 2-3 m** | 60-90 秒 | yaw + 遠心力 (横加速度) の同時励起 → 6-DoF 全観測性 | **連続的な曲線、絶対に停止しない** |
| P3 | 直線 **急加減速** の往復 (加速 → 減速 → 反転) を 3-5 セット | 60 秒 | 並進加速度の観測性 | 反転点で明確な停止、直線区間は真っすぐ |
| P4 | **その場 360° 旋回** を右→左→右→左 (各方向 2 回) | 30-60 秒 | yaw 単独の観測性、方向切替 | **各回転の間に 1-2 秒停止** |
| P5 | **急停止** (フル加速からブレーキ) を 2-3 回 | 30 秒 | ジャーク観測 | 各セット間に停止 |
| P6 | **完全静止** で終了 | 10-20 秒 | 最終 bias 確認 | 動かない |

**P2 (8 の字) の詳細** — ここがプロトコルの肝:

- WHILL ジョイスティックで **前進入力 + 旋回入力を同時に出しっぱなし**
- 旋回入力を「右 → 左 → 右 → 左」と 3-5 秒周期で緩やかに切り替える
- **速度を落とさない、旋回だけにしない**
- 1 曲線 = 3-5 秒程度、直径 2-3 m 程度が目安 (急旋回は逆効果)

なぜ P2 を停めないか: GRIL-Calib は「連続的な前進 + 旋回」の同時観測で
LiDAR-IMU 相対姿勢を推定する。停止 + 旋回だけだと遠心力 (横加速度) がゼロで
**並進 3 軸の観測性が消える** → 6-DoF のうち translation 3 成分が推定不能。
車のスラロームを一定速度でこなすイメージ。

**追加テクニック** (可能なら):
- P2 の 8 の字を **速度モードを頻繁に切り替え**ながら (mode 1 ↔ mode 3 交互)
- P3 の直線を意図的に **斜めに** (床面と非平行) — WHILL は自動転回するが、
  操作者の手加減で pitch が僅かに立つ

段差や小さな障害物 (延長コード等) は **避ける** — GLIM 診断で分かった通り、
段差は間接的に de-skew を狂わせる。今回は「純粋な motion 励起」を狙う。

**タイムライン例** (mode 1 の場合、合計 4-5 分):

```
時刻     動作                    備考
─────────────────────────────────────────────
0-25s    P1 静止                初期 bias
25-30s   位置調整               ブランク OK
30-90s   P2 8 の字 3-4 周       停まらずカーブ
90-95s   直線区間へ             ブランク
95-155s  P3 加減速往復 3-5 セット 反転点で停止
155-160s 位置調整               ブランク
160-200s P4 360° x 4            右→止→左→止→右→止→左
200-205s 直線区間へ             ブランク
205-235s P5 急停止 x 2-3         フル加速→ブレーキ
235-255s P6 最終静止             終了確認
```

## 4. 録画

Terminal A (bringup):
```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch whill_localization odom_bringup_launch.py
```

Terminal B (録画):
```bash
cd ~/whill_lab0_ros2
./scripts/m6r_record_calib_bag.sh docs/m5r-bench-data/2026-07-08-indoor-calib
# 録画開始 → §3 のプロトコルを実施 → Ctrl+C で終了
```

スクリプトは自動で:
- RMW / governor 確認
- 出力 dir 作成
- `ros2 bag record` を規約通り (圧縮なし、必要 topic のみ) で起動
- 終了時に `ros2 bag info` で健全性チェック
- 次工程コマンド (GRIL-Calib) を提示

## 5. 収録後の健全性

期待値 (5 分収録なら):
- `/velodyne_points`: 3000 前後 (300s × 10 Hz、95% 以上)
- `/imu/data_rep145`: 30000 前後 (300s × 100 Hz、95% 以上)
- `/tf_static`: 3-4

半分以下なら破棄して再録。原因調査は `docs/ja/m5r-rmw-cyclonedds.md`。

## 6. GRIL-Calib 実行

```bash
./scripts/m5r4_run_gril_calib.sh \
    docs/m5r-bench-data/2026-07-08-indoor-calib/bag
```

出力: `docs/m5r-bench-data/2026-07-08-indoor-calib/gril-calib-out/`

- 収束時: `GRIL_Calib_result.txt` に 4×4 変換行列。`new_tli.txt` に GLIM
  format 貼付用が生成される
- 収束せず watchdog fire: SUMMARY.md に `insufficient_motion` と書かれる →
  §3 の欠けた動きを補って再収録

## 7. GLIM config への反映

```bash
# 1. new_tli.txt を確認
cat docs/m5r-bench-data/2026-07-08-indoor-calib/gril-calib-out/new_tli.txt

# 2. scripts/m5r3_run_glim.sh line ~275 の T_lidar_imu を差し替え

# 3. campus-half-v3 bag で GLIM 再実行 (校正効果測定)
./scripts/m5r3_run_glim.sh \
    docs/m5r-bench-data/2026-07-07-campus-half-v3/bag \
    docs/m5r-bench-data/2026-07-07-campus-half-v3/glim-out-calibrated
```

期待効果:
- drift 66 m → 5-10 m 級 (Issue #64 分析の予測)
- Z warp 45 m → 1 m 未満
- roll/pitch 10s span → 数度程度

## 8. 差分検証

```bash
python3 - << 'EOF'
import numpy as np
for label, path in [
    ('before', 'docs/m5r-bench-data/2026-07-07-campus-half-v3/glim-out/traj_lidar.txt'),
    ('after',  'docs/m5r-bench-data/2026-07-07-campus-half-v3/glim-out-calibrated/traj_lidar.txt')]:
    d = np.loadtxt(path)
    xyz = d[:, 1:4]
    diffs = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    print(f'{label}: path={diffs.sum():.1f}m, end-start={np.linalg.norm(xyz[-1]-xyz[0]):.2f}m, '
          f'Z range={xyz[:,2].ptp():.2f}m')
EOF
```

「after」の Z range と end-start が「before」の 1/5 以下になっていれば校正
成功。DUFOMap → 占有格子まで再実行して M6R-1 smoke test の入力を更新。

## 9. 失敗時の分岐

- **GRIL-Calib が insufficient_motion で watchdog fire**: §3 の欠けた動きを
  補って収録し直し。特に P4 (その場 360° 旋回) を **各方向 3 回ずつ** に増量
- **収束したが GLIM 再実行しても drift 改善しない**: T_lidar_imu 以外の要因
  (imu_acc_noise、IMU bias initial 分散等) を疑い、`config_sensors.json` の
  他フィールドを Issue #64 診断で試した組み合わせに変える
- **時間切れ (7/19 まで)**: 現地図で M6R-1 に進む方針に切替 (計画書 §5 の
  G1-G3 で判定)。校正は demo 後の品質改善に降格

## 関連文書

- 親: [`docs/ja/plans/2026-06-24-m6r-localization.md`](plans/2026-06-24-m6r-localization.md) §9.1 R5
- Issue #64 診断: [`docs/ja/m5r-imu-diagnostic.md`](m5r-imu-diagnostic.md)
- 前回 GRIL-Calib dry-run: [`docs/m5r-bench-data/2026-06-24-loop-outdoor-ext/gril-calib-out/SUMMARY.md`](../../m5r-bench-data/2026-06-24-loop-outdoor-ext/gril-calib-out/SUMMARY.md)
- CycloneDDS 環境: [`docs/ja/m5r-rmw-cyclonedds.md`](m5r-rmw-cyclonedds.md)

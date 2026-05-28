# ADR-0001: 車輪オドメトリと LIO の融合に robot_localization 二段 EKF を採用する

- 日付: 2026-05-28
- 状態: accepted (2026-05-28, ユーザー承認)

## 文脈

本リポは現在、`whill_navigation/launch/tf_bridge_launch.py` で
`map -> camera_init` を identity として吐き、その下に FAST-LIO の
`camera_init -> body` を繋いで Nav2 に渡している。これは M5-a〜c を立ち上げる
ための短期対処であり、以下の問題を抱えている。

- Nav2 標準の `map -> odom -> base_link` 三層 TF tree になっていない。
  `odom` フレームが事実上存在せず、Nav2 controller が想定する instantaneous
  motion 用フレームが欠落
- M2 で稼働中の `/whill/odom` (WHILL 内部の車輪オドメトリ) が Nav2 に全く流れて
  いない。FAST-LIO が一瞬 jump した時の controller への影響を緩衝する層が無い
- FAST-LIO 単体ドリフトが 60 秒で 18%。ループクロージャ (ADR-0002) を入れても
  瞬時値は LIO 出力にしか頼れないため、車輪オドメと相補的に使う構成が必要

旧 noetic スタックには車輪オドメ + LIO 融合層は存在せず、`slam_localization`
の PCL NDT が `/integrated_to_init` を直接補正していた。本リポはこれを Nav2
標準 TF tree に整理し直す方針。

## 検討した選択肢

### 選択肢 A: robot_localization 単一 EKF (`/whill/odom` + `/imu` + FAST-LIO odom)
- 全ソースを 1 つの EKF に入れる
- メリット: 構成が単純、ノード 1 個
- デメリット:
  - `map` と `odom` の役割分離が崩れる。FAST-LIO の絶対位置補正と車輪オドメの
    瞬時速度を同じ filter で扱うと、LIO の jump が `odom -> base_link` に
    伝播 (Nav2 local costmap がガクつく)
  - Nav2 公式 setup guide が明確に否定しているパターン

### 選択肢 B: robot_localization 二段 EKF (本リポ採用案)
- `ekf_odom`: `/whill/odom` + `/imu` (yaw rate) → `odom -> base_link` を publish
- `ekf_map`: `ekf_odom` 出力 + FAST-LIO `/Odometry` → `map -> odom` を publish
- Nav2 setup_guides robot_localization で公式に推奨される構成
- メリット:
  - `odom -> base_link` は車輪オドメ + IMU yaw だけで連続。LIO jump の影響を
    遮断
  - `map -> odom` は LIO のドリフト補正だけを反映する純粋な correction
- デメリット:
  - EKF プロセスが 2 個になり、param ファイル 2 個保守

### 選択肢 C: 自前で `tf_bridge` ノードを拡張して TF を分解
- 旧 noetic 風に LIO の出力を `map -> odom` と `odom -> base_link` に分解
- メリット: 依存パッケージが増えない
- デメリット:
  - 車輪オドメと LIO の時刻同期・共分散統合を自作することになり、
    robot_localization が既に解いている問題の再発明
  - 保守責任を本リポが負う

## 決定

**選択肢 B (robot_localization 二段 EKF)** を採用する。

理由:
- Nav2 公式 setup guide のリファレンス構成と一致し、Nav2 内部の暗黙の前提
  (例: `odom -> base_link` の連続性、`map -> odom` の jump 許容) と完全に合う
- 車輪オドメと LIO のジャンプ伝播を構造的に分離できる
- 保守対象は YAML 2 つで、EKF のロジック自体は upstream が面倒を見る

実装場所は `whill_localization/config/` に `ekf_odom.yaml`, `ekf_map.yaml` を
追加し、`whill_localization/launch/localization_launch.py` から両 EKF を起動
する。新パッケージは作らない (パッケージ境界を増やすほどの責務分離は不要)。

## 帰結

良い側面:
- Nav2 標準 TF tree への完全準拠。M5-b 以降のすべての phase が Nav2 公式
  ドキュメント通りに進められる
- LIO jump (リローカライゼーション直後を含む) が controller に直接波及しない
- `/whill/odom` が初めて Nav2 stack に統合され、車輪駆動の瞬時応答が
  controller に届く

悪い側面:
- EKF param tuning が 2 つ必要。特に `ekf_map` の covariance 設計を間違えると
  `map -> odom` が振動する
- ノード数が 2 つ増える (計算負荷は無視できる範囲)

将来見直すべき条件:
- `ekf_map` の共分散調整が現地で破綻し、`map -> odom` が落ち着かない場合は
  ADR-0002 の段階 B (ndt_omp pure localization) で `map -> odom` 直接生成への
  切替を検討
- WHILL 側 `/whill/odom` の covariance が未設定で経験値上書きが恒久化する場合、
  `ros2_whill` 側の修正 ADR を別途立てる

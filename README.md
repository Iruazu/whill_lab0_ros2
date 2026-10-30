# whill_lab0_ros2

WHILL Model CR2 の実機スタック（ROS 2 Humble）。車体・センサ、自己位置推定、Nav2、フェイルセーフ、タブレットからの配車を含む。地図はオフラインで作り、実機では保存済みの地図を読む。

統合起動と実機配車は 2026-10-14 に成功した（[検証記録](docs/reports/2026-10-14-dispatch-demo.md)）。1 回の成功を全経路・全停止機能の受入とは扱わない。

## 文書

- [実機起動・停止手順](docs/runbook-live-demo.md)
- [構成とデータの流れ](docs/architecture-overview.md)
- [スクリプト一覧](docs/scripts.md)
- [開発環境とビルド](docs/development.md)
- [未解決事項と制約](docs/repository-maintenance.md)
- [文書一覧](docs/README.md)

文書は日本語のみ。過去の計画・検証記録は当時の記録で、現在の手順には使わない。

## 現在の実装

| 構成 | 実装内容 | 確認の範囲 |
| --- | --- | --- |
| 車体・センサ | WHILL USB、VLP-16、RT 9軸 IMU、静的 TF。RealSense は任意 | launch と設定 |
| オドメトリ | 車輪速度と補正済み IMU（`/imu/data_rep145`）の EKF | 実機で使用 |
| 地図上の自己位置 | NDT_OMP の scan-to-map。初期位置は RViz | 実機で使用 |
| ナビゲーション | NavFn、RPP、Patchwork++、LaserScan、速度平滑化 | 動的障害物層は local costmap のみ |
| 安全層 | failsafe A〜D と twist_mux | 停止指令も velocity_smoother を通る |
| 配車・Web | 地点・地図タップ、キュー、取消、手動救出、状態表示 | 認証・永続キュー・複数車両は未実装 |
| 地図作成 | GLIM、DUFOMap、占有格子 v2、手動マスク合成 | 車載 runtime とは別処理 |

Web は HTTPS 8000、rosbridge は WSS 9090。`whill_console`（gateway 8765）とは別構成。

## 起動

```bash
cd ~/whill_lab0_ros2
scripts/start_dispatch_demo.sh
```

bringup・Nav2・配車 Web・RViz を起動する。初期位置と preflight は手動。終了は Ctrl-C。詳細は [runbook](docs/runbook-live-demo.md)。

## ビルド

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  whill_sensors_bringup whill_localization whill_navigation \
  whill_perception whill_safety whill_dispatch
source install/setup.bash
```

上流ソースの取得と依存導入は [開発環境](docs/development.md)。

## ディレクトリ

| 場所 | 内容 |
| --- | --- |
| `src/whill_*/` | 自作パッケージ。責務と設定は各 README |
| `src/third_party/` | `whill_lab.repos` から取得する上流ソース（Git 管理外） |
| `scripts/` | セットアップ、起動、地図生成、校正、検証 |
| `configs/` | DDS・オフライン処理の設定 |
| `docs/` | 現用手順、設計判断、過去の記録 |
| `docs/maps/` | 地図と地点。`static.pcd` は別途配置 |
| `docs/m*-bench-data/` | 実測記録・録画 |
| `network/`, `udev/` | 有線接続とデバイス名の設定 |

## 変更のルール

コードと設定を根拠に文書を更新し、実機受入には日時・条件・証跡を残す。launch を変えたら `--show-args` で確認する。作業はブランチと PR で出し、マージはユーザーが行う。

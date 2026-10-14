# whill_lab0_ros2

WHILL Model CR2 の実機スタックを ROS 2 Humble で動かすためのリポジトリ。車体・センサ、自己位置推定、Nav2、フェイルセーフ、タブレットからの配車を含む。地図作成はオフラインで行い、実機運用では保存済みの地図を読み込む。

本 README は 2026-10-14 にローカルのソース・設定を照合して更新した。実装の存在と実機での検証完了は区別する。現在の checkout の最新コミットは `022987e`（2026-07-24）。8月デモ全体の受入結果を今回再確認したものではない。

## 最初に読む文書

- [実機起動・停止手順](docs/ja/runbook-live-demo.md)
- [構成とデータの流れ](docs/ja/architecture-overview.md)
- [スクリプト一覧](docs/ja/scripts.md)
- [開発環境とビルド](docs/ja/development.md)
- [現在の制約と整備課題](docs/ja/repository-maintenance.md)
- [文書一覧](docs/ja/README.md)

文書は日本語を正本とする。旧言語別入口は互換リンクとして残す。過去の計画・検証記録は当時の記録であり、現在の起動手順には使わない。

## 現在の実装

| 構成 | 実装内容 | 確認の範囲 |
| --- | --- | --- |
| 車体・センサ | WHILL USB、VLP-16、RT 9軸 IMU、静的 TF。RealSense は任意起動 | launch と設定を確認 |
| オドメトリ | 車輪速度と補正済み IMU を EKF で融合 | `/imu/data_rep145` が現用入力 |
| 地図上の自己位置 | NDT_OMP による scan-to-map、RViz から初期位置入力 | FAST-LIO の旧 runtime launch は削除済み |
| ナビゲーション | NavFn、RPP、Patchwork++、LaserScan、速度平滑化 | 衝突検知は有効。動的障害物層は local costmap のみ |
| 安全層 | failsafe A〜D と twist_mux | 停止指令も後段の velocity_smoother を通る |
| 配車・Web | 地点・地図タップ、キュー、取消、手動救出、状態表示 | 認証・永続キュー・複数車両管理は未実装 |
| 地図作成 | GLIM、DUFOMap、占有格子 v2、手動マスク合成 | 車載 runtime と別の処理 |

実機運用は bringup / Nav2 / dispatch の3つの launch を併用する。詳細は runbook に集約する。`whill_platform` / `whill_console` の gateway 8765 とは別構成で、このリポジトリの Web は HTTPS 8000、rosbridge は WSS 9090。

## ディレクトリ

| 場所 | 内容 |
| --- | --- |
| `src/whill_*/` | 自作 ROS 2 パッケージ。各 README に責務と設定を記載 |
| `src/third_party/` | `whill_lab.repos` から取得する上流ソース。Git 管理対象外 |
| `scripts/` | セットアップ、地図生成、校正、検証 |
| `configs/` | DDS・オフライン処理などの設定 |
| `docs/ja/` | 現用手順、設計判断、過去の計画・調査 |
| `docs/maps/` | 地図・地点・生成経緯。`static.pcd` は別途配置が必要 |
| `docs/m*-bench-data/` | 実測記録・録画。録画本体はコードと分けて扱う |
| `network/`, `udev/` | 有線接続とデバイス名の設定 |
| `build/`, `install/`, `log/` | colcon の生成物 |

## ビルド

Ubuntu 22.04 / ROS 2 Humble を前提とする。上流取得・依存導入の準備は [開発環境](docs/ja/development.md) を参照。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  whill_sensors_bringup whill_localization whill_navigation \
  whill_perception whill_safety whill_dispatch
source install/setup.bash
```

`--packages-up-to whill` だけではこのリポジトリの自作6パッケージは対象にならない。GLIM は地図作成時に別途準備する。

## 更新のルール

コード・設定を根拠に文書を更新し、実機受入には日時・条件・証跡を残す。launch の変更は `--show-args` で確認する。作業はブランチと PR で提示し、マージはユーザーが行う。

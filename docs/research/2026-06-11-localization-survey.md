# Research: キャンパス自己位置推定 / SLAM 手法調査 (要約版)

## 調査日
2026-06-11

## この文書の位置づけ

`docs/plans/2026-06-11-platform-pivot.md` (開発方針) の技術的根拠。
方針文書が「決定」を載せるのに対し、本文書は候補比較と現実装診断の詳細を残す。
人間向けの図表付き完全版 (HTML 2 本) はリポジトリ外で保管し、
リポジトリ管理対象は本要約のみとする。

## TL;DR

- FAST-LIO の 18%/60s ドリフトは、ループクロージャを持たない LIO の構造的限界。
  対処は手法の差し替えではなく「マップ作成 (オフライン・ループ付き) と
  運用 (事前地図への scan-to-map localization) の二相分離」
- つくばチャレンジ完走チームの主流は、3D LiDAR で事前点群地図を作り、
  運用は NDT / MCL 系で localization する構成。宇都宮大 REAL_C も 2024 完走
- 推奨: マップ作成 = GLIM (第一候補) または FAST-LIO SAM / li_slam_ros2、
  動的除去 = ERASOR、運用 = lidar_localization_ros2 (NDT_OMP)。
  TF は REP-105 (map -> odom: localizer / odom -> base_link: 車輪 + IMU の EKF)

## 候補比較: マップ作成 SLAM / LIO

| 手法 | humble 対応 | ライセンス | ループ | VLP-16 実績 | 備考 |
|------|------------|-----------|--------|------------|------|
| GLIM | 公式 (PPA/Docker, Jetson Orin 検証) | MIT | あり (大域最適化) | 回転式対応 | GPU 推奨・CPU モード可。第一候補 |
| FAST-LIO SAM | ROS1 中心 + ROS2 派生 | GPL 系 | あり | velodyne プリセット | 既存 FAST-LIO 資産からの最小移行先 |
| li_slam_ros2 / lidarslam_ros2 | 公式 | Apache-2.0 | あり | あり (casual_walk.bag) | 軽量。permissive |
| LIO-SAM | コミュニティ移植 | BSD-3 | あり | 強い (公式データ提供) | 9 軸 IMU 必須。ビルド時メモリ注意 |
| LeGO-LOAM | 移植あり | BSD-3 | あり | あり | つくば Aqua の地図作成実績 |
| FAST-LIO2 (現行) | コミュニティ | GPL-2.0 | なし | あり | マップ作成補助・比較用に格下げ |
| DLIO / KISS-ICP / Point-LIO / Faster-LIO | 公式〜コミュニティ | MIT / GPL 混在 | なし | あり | オドメトリ用途。参考 |

## 候補比較: 運用 localization

| 手法 | humble 対応 | ライセンス | 特徴 |
|------|------------|-----------|------|
| lidar_localization_ros2 | 公式 | BSD 系 | NDT/GICP/NDT_OMP。つくば 2024 で安定動作。odometry 拘束併用が前提。第一候補 |
| hdl_localization | コミュニティ | BSD-2 | NDT + UKF (IMU 融合)。VLP16 テスト済 |
| Autoware ndt_scan_matcher | 公式 (autoware_core) | Apache-2.0 | 動的マップ読込・正則化。大規模向け |
| FAST_LIO_LOCALIZATION 系 | ROS2 版あり | GPL 系 | FAST-LIO + 低頻度 scan-to-map 補正。事前マップ補正で ATE が桁違いに改善 (arXiv:2402.05540) |
| mcl_3dl | 対応 | BSD-3 | 3D パーティクルフィルタ。リセット耐性。つくば完走実績 (FAST-LIO odom 併用構成) |
| emcl2 | 対応 (2D) | — | 膨張リセットで誘拐回復。設計思想を failsafe 設計に流用する |

## 動的環境 (歩行者) 対応

- マップ作成後に ERASOR (RA-L 2021。静的点を保ちつつ高速) または Removert で
  動的トレースを除去し、静的地図化する
- 運用時の scan-to-map は動的点が少数派のため比較的頑健。ただし本リポの run3
  (歩行者横断で FAST-LIO 発散) が示す通り「補正なしオドメトリ」は人に弱い。
  これが二相分離の直接の動機
- 運用中の回避は Nav2 obstacle layer の復活 (QoS 橋) が前提

## つくばチャレンジ知見 (要点)

- 約 2 km・歩行者環境・屋根付き GNSS 不良区間。2024 は本走行出走 78 台中、完走 14 台
- 完走構成の典型は「事前 3D 点群地図 + NDT または MCL」。車輪オドメトリ拘束が
  段差・LiDAR 縮退時の破綻を防いだとの報告あり (AbudoriLab 2024)
- 失敗原因の最多は自己位置推定 (fuRo 原 2018 調査)

## 現実装の診断 (P1-P5 の詳細)

方針文書 2 章の根拠。

- P1 補正経路なし: `tf_bridge_launch.py` が `map -> camera_init` を identity 固定。
  FAST-LIO のドリフトがそのまま map 座標誤差になり、static map は
  Nav2 にとって「ズレていく壁」になる
- P2 初期位置: camera_init = 起動位置。地図原点はマッピング走行の起動位置
  (`pcd_to_occupancy_grid.py` の origin 前提)。同地点・同方位での起動が暗黙の前提で、
  リローカライズ手段がない
- P3 発散の未検知: run3 で実証。発散しても TF は出続け Nav2 は走行継続。
  マッチングスコア / 共分散のゲート、リセット、E-stop が存在しない
- P4 odom 不在: `/whill/odom` (M2 で動作済) が未配線。補正導入時のジャンプ緩衝材がなく、
  LiDAR 縮退時のバックアップもなく、rolling local costmap が map 基準で回っている
- P5 地図品質と安全機能の連鎖停止: 歪み + ゴーストにより
  `use_collision_detection: false`、QoS 不一致により obstacle layer なし。
  結果として運用中の歩行者が costmap に一切映らない

## ライセンス要点

- 義務のトリガーは「配布」。研究室内の実行・解析・論文執筆は対象外
- プロセス分離 + トピック通信なら GPL ノードは自作ノードを派生物にしない
  というのが ROS の通例解釈。ソース改変・リンク・コピペは派生物になる
- 運用スタックは permissive (MIT/BSD/Apache) 構成可能に保つ (方針 3.4)。
  `src/third_party/` 非同梱と GPL コピペ禁止を維持

## 計算資源の要点

- VLP-16 は点数が少なく、運用 (localization + Nav2) はミドル CPU・GPU 不要
- 重いのはマッピングの大域最適化 (GPU 推奨) とオフライン後処理。bag を母艦で処理する
- 開発機の確認結果と機材判断 (Alienware x15 R2 で母艦兼用、Jetson TX2 除外) は
  方針文書 9 章を参照

## 出典 (主要)

- GLIM: https://github.com/koide3/glim
- lidar_localization_ros2: https://github.com/rsasaki0109/lidar_localization_ros2
- li_slam_ros2: https://github.com/rsasaki0109/li_slam_ros2
- FAST-LIO SAM: https://github.com/engcang/FAST-LIO-SAM
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM
- hdl_localization: https://github.com/koide3/hdl_localization
- mcl_3dl: https://github.com/at-wat/mcl_3dl
- emcl2: https://github.com/ryuichiueda/emcl2
- ERASOR: https://github.com/LimHyungTae/ERASOR (arXiv:2103.04316)
- Removert: https://github.com/gisbi-kim/removert
- 事前マップ補正の定量効果: arXiv:2402.05540
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- つくばチャレンジ公式記録: https://tsukubachallenge.jp/

導入前に各リポジトリの最新コミット・ライセンスを再確認すること。

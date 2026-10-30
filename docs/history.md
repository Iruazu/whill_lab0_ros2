# 過去の計画・調査・検証

当時の判断と実測を残す索引。本文の予定・未着手・旧設定は執筆時点のもので、現在の手順には使わない。現在の起動は [runbook](runbook-live-demo.md)、設計判断は [ADR 一覧](decisions/README.md)。

- [M5-e/Phase A〜C 凍結記録 (2026-06-13)](legacy-findings/2026-06-13-m5e-frozen.md)
- [旧 M5-b マップディレクトリ リネーム記録 (2026-06-21)](legacy-findings/2026-06-21-m5b-maps-renamed.md)
- [旧リポインデックス (`~/whill_lab0/`)](legacy-index.md)
- [M2 — ROS 2 humble での WHILL コアドライバ](m2-whill-core.md)
- [noetic スタックから引き継ぐ LiDAR ↔ IMU 外部パラメータ](m3-extrinsics-from-noetic.md)
- [M3 — ROS 2 humble でのセンサスタック](m3-sensors.md)
- [M4-R 前置: ModelCr2State の単位曖昧性を実機で確定する](m4r-whill-units.md)
- [M5-R 前置: CUDA Toolkit 12.4 と cuDNN 8 のセットアップ](m5r-cuda-setup.md)
- [M5-R 前置: FAST-LIO SAM 評価候補化 (clone-on-demand 運用)](m5r-fastlio-sam-eval.md)
- [M5-R 前置: GLIM のソースビルドによるセットアップ](m5r-glim-setup.md)
- [M5-R: GLIM IMU integration 警告の原因切り分けと対処方針](m5r-imu-diagnostic.md)
- [M5-R マップ作成パイプライン](m5r-pipeline.md)
- [M5-R: `/velodyne_points` 1 Hz 病の原因切り分けと RMW 選定](m5r-rmw-cyclonedds.md)
- [M5R-3: GLIM vs FAST-LIO SAM 実 bag 比較プロトコル](m5r3-comparison-protocol.md)
- [M6-R デモ準備チェックリスト](m6r-demo-prep-checklist.md)
- [Indoor calibration bag 収録プロトコル (M6-R / Issue #64 解消用)](m6r-indoor-calib-bag-protocol.md)
- [開発方針: 配車プラットフォーム化に向けたアーキテクチャ再編](plans/2026-06-11-platform-pivot.md)
- [M4-R 実行計画: odom 基盤の再構築](plans/2026-06-13-m4r-execution.md)
- [M5-R 実行計画: マップ作成パイプライン](plans/2026-06-21-m5r-execution.md)
- [M6-R 実行計画: 運用 localization + Nav2 安全機能復帰 (2026-08-01 屋外デモに向けて)](plans/2026-06-24-m6r-localization.md)
- [M6R-4 実行計画: Nav2 obstacle layer 復活 + collision detection 復帰](plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
- [Plan: M7 配車 API 層 (whill_dispatch) + 最小 Web UI](plans/2026-07-19-m7-dispatch.md)
- [v2 map 品質改善パイプライン 実装レポート](reports/2026-07-18-pipeline-v2-report.md)
- [Research: キャンパス自己位置推定 / SLAM 手法調査 (要約版)](research/2026-06-11-localization-survey.md)

## 削除した文書

2026-10-30 に削除。本文は `git show 83e5abf:docs/ja/<ファイル名>` で読める（経緯は [整理の棚卸し](repository-refresh-2026-10-30.md)）。

- `m1-environment-setup.md` — M1 環境構築。現行は [開発環境](development.md)
- `m4-localization.md` — FAST-LIO を runtime とした旧 M4。現行は NDT localizer（ADR-0006）
- `m5-navigation.md` — 旧 M5 計画。現行は `src/whill_navigation/README.md`
- `migration-plan.md` — noetic → humble 初期移植計画。旧リポの対応は [旧リポインデックス](legacy-index.md)
- `session-2026-05-06.md` / `-07.md` / `-08.md` — M2〜M4 のセッションログ

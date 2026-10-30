# リポジトリ整理の棚卸し (2026-10-30)

ブランチ `chore/repo-refresh`（基点 `docs/concise-cleanup` 83e5abf）で行った整理の記録。根拠はコード・設定とし、文書をコードに合わせる。実行時の挙動（launch・YAML・failsafe・twist_mux・velocity_smoother・速度プリセット・地図・localizer・dispatch・静的 TF の値）は変えていない。

削除した文書の本文は `git show 83e5abf:docs/ja/<ファイル名>` で読める。

## 1. 構造

| 対象 | 処置 | 理由 |
| --- | --- | --- |
| `docs/ja/*` → `docs/*` | move（`git mv`） | ADR-0001 追記で日本語のみが正本になり、言語別ディレクトリは日英並列時代の名残。`docs/en/` は 2026-10-15 に削除済み |
| `docs/ja/.gitkeep` | delete | 空ディレクトリ保持用。不要 |
| `docs/m*-bench-data/` | 据え置き | 実測記録。置き場の変更は別途提案（本文中の `../ja/` リンクだけ直す） |
| `docs/maps/` | 据え置き | `m6r_bringup` が `docs/maps/<site>/static.pcd` を起動時に解決する |
| `configs/cyclonedds-runtime.xml` | move → `configs/legacy/` | 旧 NIC `enx00e04c6808dc`（ドック交換前）を列挙。リポ内のスクリプト・launch は参照していない（現用は `cyclonedds-runtime-min.xml`）。`~/.bashrc:140` だけが参照する（ユーザー作業） |
| `configs/cyclonedds-runtime.xml.bak` | report-only | Git 管理外で、メインのチェックアウトにだけある。ブランチ作業では触れない |
| `configs/cyclonedds-bag-record.xml` | 据え置き・コメント修正 | 録画専用。コメントが運用 XML を旧 `runtime.xml` と案内していた |
| `scratchpad/*.py` | 据え置き | 校正台帳が `imu_live_check.py` / `frame_audit.py` を手順として参照 |

## 2. 旧文書

| 文書 | 処置 | 理由・残す情報の所在 |
| --- | --- | --- |
| `session-2026-05-06.md` / `-07.md` / `-08.md` | delete | M2〜M4 当時のセッションログ。結論は `m2-whill-core.md`（コールドブート対策）、`m3-bench-data/`、`m4r-whill-units.md` に残る。`history.md` に削除コミットを記載 |
| `m1-environment-setup.md` | delete | 現行の手順は `development.md` と `scripts/install_ros2_humble.sh` / `setup_workspace.sh` / `configure_proxy.sh` |
| `m4-localization.md` | delete | FAST-LIO を runtime localizer とした旧 M4。FAST-LIO runtime は撤去済み（CLAUDE.md、`whill_lab.repos` のコメント）。現行は NDT（ADR-0006、`whill_safety/config/m6r_lidar_localization.yaml`） |
| `m5-navigation.md` | delete | 旧 M5 計画。現行は `src/whill_navigation/README.md` と ADR-0010 |
| `migration-plan.md` | delete | noetic→humble 初期の移植計画。旧リポの対応表は `legacy-index.md`、方針は `plans/2026-06-11-platform-pivot.md` |
| `m2-whill-core.md` | 保持 | `Iruazu/ros2_whill` fork のコールドブート対策など、他に無い情報がある |
| `m3-sensors.md` | 保持・fix | Velodyne ネットワーク設定の手順が他に無い。NIC 例を旧名からプレースホルダへ |
| `m3-extrinsics-from-noetic.md` | 保持 | `static_tf_launch.py` のコメントが LiDAR TF の分解根拠として参照 |
| `m4r-*` / `m5r-*` / `m5r3-*` / `m6r-indoor-*` / `imu-coordinate-audit.md` | 保持 | スクリプト（`install_glim.sh`、`install_gril_calib.sh`、`m5r3_*`）、`configs/cyclonedds-bag-record.xml`、地図 metadata が参照 |
| `m5r-fastlio-sam-eval.md` | 保持 | FAST-LIO SAM 不採用の根拠（ライセンス）として ADR-0003 が参照。runtime の手順ではない |
| `m6r-demo-prep-checklist.md` | 保持・fix | 経路整備の記録を ADR-0007 / 0009 が参照。起動手順は runbook と重複（下記 4 節） |
| `plans/`、`reports/`、`research/`、`legacy-findings/`、`legacy-index.md` | 保持 | 過去の計画・証跡。`.github` テンプレートと `.claude/commands` が platform-pivot 計画を参照 |
| `decisions/`、`calibration-ledger.md` | 保持 | ADR と校正台帳 |

## 3. コード・設定と食い違う記述

| 文書側 | コード・設定側 | 処置 |
| --- | --- | --- |
| `m6r-demo-prep-checklist.md:107` Layer D 起動ログ「> 5 pts … @ 0.5-2.0 m」 | `failsafe_node.py:73-74,82`（1.0〜2.0 m、`FORWARD_POINT_COUNT_MIN = 1`）、ログ書式 `:167-175` | fix |
| `m6r-demo-prep-checklist.md:109` 手を前方 1 m に翳す | 検知帯は 1.0 m から（`failsafe_node.py:73`）。preflight は約 1.5 m（`m6r_preflight.sh:105`） | fix（1.5 m） |
| `m3-sensors.md:120` NIC 例 `enx00e04c6808dc` | 現物と `cyclonedds-runtime-min.xml:26` は `enx00e04c680ec9`（2026-07-31 ドック交換） | fix（プレースホルダ） |
| `network/01-velodyne-static.yaml.template:11` 開発機の NIC を `enx00e04c6808dc` と記載 | 同上 | fix（コメントのみ） |
| `configs/cyclonedds-bag-record.xml:6,23,26` 運用は `cyclonedds-runtime.xml` | `start_dispatch_demo.sh:67,202`、runbook は `cyclonedds-runtime-min.xml` | fix（コメントのみ） |
| `repository-maintenance.md:8` DDS XML 2 本が併存 | 旧 XML は `configs/legacy/` へ移動 | fix |
| `nav_launch.py:164` コメントが `docs/session-2026-05-08.md` を参照（存在しない） | — | fix（コメントのみ。git 履歴の位置を記載） |
| `m5r3_run_glim.sh:357` コメントが `docs/session-2026-07-08` を参照（存在しない） | 該当の議論は `imu-coordinate-audit.md` §7 | fix（コメントのみ） |
| `pcd_to_occupancy_grid.py:22-23` 使用例が `docs/m5-maps/`（2026-06-21 に改名済み） | `docs/maps/<site>/` | fix（docstring のみ） |
| `.github/ISSUE_TEMPLATE/bug.md:10`、`.claude/agents/debugger.md:3,8,25` の例が `fastlio_mapping` / `/Odometry` | FAST-LIO runtime は撤去済み | fix（例を現行ノードへ） |
| `.claude/agents/ros2-implementer.md:40` が `fast_lio_launch.py` の冒頭コメントを参照（存在しない） | 同じ絶対パス解決は `nav_launch.py` の BT XML 処理にある | fix |
| `whill_lab.repos:35` IMU を「FAST-LIO input topic」と説明 | EKF は `/imu/data_rep145` を使用（`ekf_odom.yaml`） | fix（コメントのみ） |
| `scripts.md` に `tests/test_dispatch_demo_launcher.py` が無い | `scripts/tests/` に存在 | fix |
| `m5r-bench-data/README.md:9` リンク文字列が `../en/m5r-pipeline.md` | リンク先は日本語版 | fix |
| `decisions/0001-docs-i18n.md` 「詳細は `docs/ja/`」 | 本整理で `docs/` 直下へ | fix（追記） |

### report-only（承認が必要な設定値）

| 項目 | 場所 | 内容 |
| --- | --- | --- |
| LiDAR 静的 TF | `src/whill_sensors_bringup/launch/static_tf_launch.py:124-127` | コード (0.484, 0.382, 0.794, pitch +9.0°) / 実測 (0.42, 0.36, 0.777, pitch 約 +1°)。校正台帳 `calibration-ledger.md:158` も pitch +9° を誤りと記載。地図・localizer に波及するため別タスク |
| カメラ静的 TF | 同 `:143-145` | (0.54, 0.382, 0.79)。実測は約 (0.47, 0.378, 0.73)。`feat/avoidance-baseline` が更新済み（未マージ） |
| Velodyne 校正ファイル | `src/whill_sensors_bringup/launch/sensors_launch.py:93` → 上流 `velodyne-all-nodes-VLP16-launch.py:54` | `VLP16db.yaml`（±15°）を読む。実機は Puck Hi-Res（±10°）の疑い |
| `~/.bashrc:140` | ホスト設定 | `CYCLONEDDS_URI` が旧 `configs/cyclonedds-runtime.xml` を指す。本ブランチのマージ後はファイルが無くなる |

## 4. 重複

| 内容 | 箇所 | 処置 |
| --- | --- | --- |
| 実機起動手順 | `runbook-live-demo.md` と `m6r-demo-prep-checklist.md` の起動節 | report-only。checklist は過去記録として保持し、現用は runbook |
| 文書索引 | ルート `README.md` と `docs/README.md` | report-only。役割（概要と全索引）が違うため据え置き |
| M5-R パイプライン | `m5r-pipeline.md` と `m5r-bench-data/README.md` | report-only |

## 5. リンク

整理前の Markdown 相対リンク切れは 0 件。移動後もリポ全体で `docs/ja` と旧ファイル名を grep し、相対リンクを検査した。

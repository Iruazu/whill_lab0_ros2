# M6-R 実行計画: 運用 localization + Nav2 安全機能復帰 (2026-08-01 屋外デモに向けて)

Language: [日本語](2026-06-24-m6r-localization.md) | [English](../../en/plans/2026-06-24-m6r-localization.md)

- 日付: 2026-06-24 起草 / 2026-07-07 実データ収録・戦略確定・全面改訂 (accepted)
- 状態: **accepted (2026-07-07)**
  - 判断 §3.B: 案 A (新規 `whill_safety` パッケージ) 継続採用
  - 判断 §3.C: (i) `twist_mux` 採用継続
  - 判断 §3.E: M6R-6 (camera_link 再校正) は **本フェーズから除外、デモ後の品質改善タスクへ**
  - 判断 §5: **合否は G1-G3 で行う**。<1% 大域ループ閉合は M6-R ゲートに含めない
- 親方針: [`docs/ja/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md)
  §2 (診断 P1-P5), §3.2 (TF 責務), §3.3 (採用候補), §4 (M6-R), §5 (行動規範)
- 前段:
  - [`docs/ja/plans/2026-06-13-m4r-execution.md`](2026-06-13-m4r-execution.md) (M4-R 完了 = EKF)
  - [`docs/ja/plans/2026-06-21-m5r-execution.md`](2026-06-21-m5r-execution.md) (M5-R 完了 = GLIM + DUFOMap パイプライン)

## 0. ユーザー要件の理解

**最終目標**: 2026-08-01 オープンキャンパスでの屋外・実機・搭乗者ありの
配車試験運用。

**M6-R の位置付け**: この目標に向けた「運用 localization + Nav2 安全機能復帰」
フェーズ。M7 (配車 API) の直前に置き、以下の予算で回す:

- **ハード期限**: 2026-07-25 完成目標 (8/1 の 1 週間前、追加チューニング余地)
- **Go/No-Go 判定**: 2026-07-19、下記 §5 の **G1-G3 全て pass** で GO
- **環境スコープ**: 屋外キャンパス内道路のみ。**フォールバックなしのフルベット**
- **本フェーズで確定した戦略判断 (再検討しない)**:
  1. 大域ループ閉合精度 (<1%) を M6-R ゲートに **含めない**。運用 localizer
     (lidar_localization_ros2, NDT scan-to-map, 既定 `use_imu: false`) は
     保存地図に連続再アンカーするため、地図が大域的に多少歪んでいても
     局所整合していれば走行できる。根拠: [`docs/ja/m5r-imu-diagnostic.md`](../m5r-imu-diagnostic.md)
     の IMU ON/OFF 比較 (loop error 3.99 → 3.97 m でほぼ不変、translation は
     LiDAR ICP 支配) と [`docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml`](../../m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml)
     の loop closure 未発火 (直進往復ジオメトリが原因)
  2. IMU / extrinsic 再校正 (GRIL-Calib, Issue #64) と GLIM loop closure
     改善は **demo 後の品質改善** に回す。M6-R のブロッカーではない
  3. 環境スコープを屋外キャンパスに限定、屋内・フォールバックを持たない

**M4-R / M5-R との性格の違い**:
- M4-R = 実装フェーズ (4 Issue 直列)
- M5-R = 選定 + パイプライン整備 (7 Issue + ADR 3 件)
- **M6-R = 統合 + 安全機能復帰 + デモ準備**。localizer 候補
  (lidar_localization_ros2) と入力地図 (2026-07-07 収録 bag による GLIM 出力) が
  揃っており、作業は「組み込み + 安全網 + Nav2 復旧 + 実機検証」が中心

## 1. 背景

### 1.1 解消する既知課題

親方針 §2 診断のうち本フェーズが解消する範囲:

| ID | 内容 | M6-R での解消経路 |
|----|------|----------------|
| P1 | 運用時の自己位置に補正経路がない (`map -> camera_init` identity 廃止後、`map -> odom` publisher 不在) | lidar_localization_ros2 が `enable_map_odom_tf: true` で `map -> odom` を publish |
| P2 (残り) | 初期位置合わせ機構がない | RViz 2D Pose Estimate → `/initialpose` 受信 (localizer ネイティブ) |
| P3 | 発散を検知も回復もしない | `whill_safety` ノード (新規) が `/alignment_status` + `/pcl_pose` 監視で `twist_mux` 経由 `/cmd_vel` 遮断 |
| P5 (残り) | Nav2 obstacle layer 不在、`use_collision_detection: false` | pointcloud_to_laserscan で `/velodyne_points` → `/scan` (QoS bridge)、obstacle layer 復活、collision detection ON |

### 1.2 M4-R / M5-R 成果物の前提

- **M4-R 出力** (accepted, 2026-06-20):
  - TF: `odom -> base_link -> {imu_link, velodyne, camera_link}` 一本鎖
  - `/odometry/filtered` 30 Hz
  - launch: `whill_localization/launch/odom_bringup_launch.py`
- **M5-R 出力** (accepted, 2026-06-22):
  - GLIM ADR-0003 + DUFOMap ADR-0004 + occupancy 規約 ADR-0005
  - パイプライン: bag → GLIM → DUFOMap → occupancy grid → `docs/maps/<site>/`
- **今日 (2026-07-07) 収録** (M6-R 入力):
  - `docs/m5r-bench-data/2026-07-07-campus-half-v3/bag` (7分46秒、
    速度モード 3 固定、停止なし、健全性 velodyne 9.85 Hz / imu 100 Hz)。
    **GLIM 適用済** (real 208.5 s, playback 2.2x)、4579 キーフレーム。
    loop closure 未発火 (単一ループ + 直進主体、想定内)
  - `docs/m5r-bench-data/2026-07-07-campus-loop/bag` (13.7 分、外乱多め、参考)

REP-105 の TF 構造完成形:

```
map (lidar_localization_ros2, M6-R)
└── odom (ekf_filter_node, M4-R 継続)
    └── base_link (EKF が author、連続・滑らか)
        ├── imu_link, velodyne, camera_link (static, M4-R 継続)
```

`map -> odom` の補正ジャンプは `odom -> base_link` の連続性が吸収する。

### 1.3 M6-R で「触らないもの」

- **FAST-LIO のランタイム強化**: 親方針 §5 禁止 #2 維持
- **`tf_bridge_launch.py` の復活**: 親方針 §5 禁止 #1 維持
- **IMU / extrinsic 再校正 (GRIL-Calib)**: Issue #64 で診断済、
  demo 後の品質改善タスク
- **GLIM loop closure 改善**: 直進往復ジオメトリに起因、demo 後
- **配車 API (whill_dispatch)**: M7 担当
- **camera_link target-based 再校正**: **旧 M6R-6 を本フェーズから除外**、
  demo 後。理由: (a) lidar_localization_ros2 は camera を使わない、
  (b) Nav2 obstacle layer は `/velodyne_points` のみ消費、
  (c) M7 dispatch も camera 非依存、(d) 7/25 期限を守るための scope 削減

## 2. 作業範囲

### 2.1 扱うもの

1. **lidar_localization_ros2 vcs import + 単独動作確認** (M6R-1)
2. **M4-R EKF との統合 + `/initialpose` 運用** (M6R-2)
3. **フェイルセーフノード新設 (`whill_safety` パッケージ)** (M6R-3)
4. **Nav2 obstacle layer 復活 + `use_collision_detection: true`** (M6R-4)
5. **G1-G3 統合受入テスト + M7 引き渡し文書** (M6R-5)
6. **ADR-0006 (localizer 選定) と ADR-0007 (failsafe 設計) 起草・承認**

### 2.2 扱わないもの (明示、旧計画から降格したものを含む)

- 旧 M6R-6 camera_link 再校正 → **本フェーズから除外**、demo 後
- Issue #64 (GLIM IMU 警告根本原因) の解消 → demo 後
- 配車 API (whill_dispatch) → M7
- Web / タブレット UI → M8
- 完全なキャンパス本番試験 → M9
- <1% 大域ループ閉合の確認 → G1-G3 に含めない
- 屋内経路の localization 検証 → 屋外スコープ外
- GNSS / RTK 統合 → 親方針 §7 ADR 候補、非対象
- 新規 SLAM の選定 → M5-R で GLIM 確定 (ADR-0003)

## 3. 既存 WIP コードと残骸の扱い

### 3.A 旧版 M6-R 計画書 (2026-06-24 起草の 867 行版) の廃止

本改訂で **上書き**。旧版は C1-C8 の 8 項目受入基準・旧 M6R-6 を critical
path と扱う設計で、7/25 期限には過剰。本版が権威。

### 3.B `whill_safety` 新設 vs 既存パッケージ内包

継続採用: **案 A (新規 `whill_safety` パッケージ)**。理由: (1) 責務分離が
明確、(2) M9 の物理 E-stop / 遠隔停止を後付けする余地、(3) 親方針 §3.5 の
パッケージ境界設計と平仄。

### 3.C `/cmd_vel` 遮断方式

継続採用: **(i) `twist_mux`**。優先度切替の設定 yaml がそのまま「優先度の
文書」になる利点は変わらず。apt: `ros-humble-twist-mux`。

### 3.D `lidar_localization_ros2` の IMU 配線

本フェーズ既定は **`use_imu: false`** (v1.1.0 の
`lidar_localization_component.cpp` 直読で確認済)。EKF が `/imu/data_rep145`
を消費しており、localizer と経路分離できる。将来 scan undistortion が
必要になったら `use_imu: true` に切替 + `imu` topic に
`/imu/data_rep145` を remap する。

### 3.E `nav_launch.py`

M4-R 完了時点で「起動するが localise しない」意図的な半壊状態。M6R-4 で
default map yaml を新地図に向け直し、`pointcloud_to_laserscan` 追加、
`use_collision_detection: true` 復帰、obstacle layer 有効化。

### 3.F 新地図の site 名

**確定 (M6R-1 着手前提)**: `campus-half-v3`。bag ディレクトリ名を踏襲、
既存 `lab-legacy-m5b` と衝突しない。最終保存先 `docs/maps/campus-half-v3/`。

## 4. 前提条件

- **M4-R merged**: `odom_bringup_launch.py` で `odom -> base_link` EKF が
  30 Hz publish 済
- **M5-R merged**: GLIM + DUFOMap パイプライン、occupancy 規約 (ADR-0005)
- **PR #61 (b2cb7df) merged**: `base_link -> {imu_link, velodyne, camera_link}`
  static TF が実測値化
- **PR #56 (ecadced) merged**: `imu_sign_corrector` で `/imu/data_rep145`
  が REP-145 化
- **PR #73 (5f5b181) merged**: 実行環境の RMW を CycloneDDS に固定
- **今日 (2026-07-07) 収録済 bag** 2 本 + GLIM 出力 (campus-half-v3)
- WHILL Model CR2 / Velodyne VLP-16 / RT 9 軸 IMU 利用可能
- 実機検証・実走行 bag 取得はユーザー実施 (CLAUDE.md 規約)
- 各セッション開始時に CycloneDDS 環境と CPU governor `performance` を確認

## 5. 受け入れ基準 (G1-G3、7/19 go/no-go)

**旧 C1-C8 は廃止**。7/25 期限を守るため、以下 3 ゲートに単純化する。
G4-G7 は G1-G3 を成立させる前提条件として付随する。

### G1: 経路 1 周を通じて自己位置を失探しない

- **観測コマンド**:
  ```bash
  ros2 launch whill_safety m6r_bringup_launch.py site:=campus-half-v3
  # RViz 2D Pose Estimate → /initialpose publish
  ros2 topic hz /pcl_pose                    # 収束後、10 Hz 前後で継続
  ros2 topic echo /alignment_status --once   # score フィールド確認
  ros2 bag record /pcl_pose /alignment_status /tf -o eval-G1
  # 実機で経路 1 周
  ```
- **合格条件**:
  - `/initialpose` publish から **10 秒以内に `/pcl_pose` が指定座標近傍
    (< 1.0 m) に収束**
  - 経路 1 周中に `/pcl_pose` の publish が **停止しない** (途切れ 10 s 未満)
  - NDT alignment score が localizer 定義の failure threshold を維持
    (閾値は M6R-1 実測で確定、ADR-0006 に記載)

### G2: 走行中の `map -> odom` TF 飛びが 0.5 m 未満

- **観測コマンド**:
  ```bash
  ros2 bag record /tf -o eval-G2
  # 経路 1 周走行
  ./scripts/m6r_tf_jump_check.py eval-G2 --parent map --child odom
  ```
- **合格条件**: 連続フレーム間の並進差分 `> 0.5 m` が **ゼロ件**
- **旧計画からの差分**: 旧 0.3 m 閾値だったが、屋外 NDT の粗い補正を許容し
  0.5 m に緩和 (親方針の要件レベル)

### G3: 既知ランドマークの位置再現性が ±0.5 m 以内

- **観測コマンド**: 経路上の 3 点以上に地上マーカ (チョーク・ペイント) を
  置き、実走行時に WHILL がマーカ真上を通過した瞬間の `/pcl_pose` を記録。
  マーカ地上位置 (メジャー実測) と比較
- **合格条件**: 全マーカ通過時の位置誤差 `< 0.5 m`

### 補強基準 (code-reviewer 確認、G1-G3 の前提条件)

- **G4 (安全)**: `whill_safety` の failsafe が `/alignment_status` 劣化 or
  `/reinitialization_requested` 受信で `/cmd_vel` を **< 200 ms** で zero
  Twist 化。手動 publish 試験で確認
- **G5 (障害物)**: 経路上の歩行者・自転車が Nav2 local_costmap に反映され、
  `FollowPath.use_collision_detection: true` で停止 or 回避
- **G6 (ADR)**: ADR-0006 と ADR-0007 が `accepted`
- **G7 (排他性)**: `m6r_bringup_launch.py` と `odom_bringup_launch.py` の
  同時起動を防ぐ排他性を README に明記

## 6. Issue 分割案

以下 5 件に分割。旧 Issue #65-#70 のうち **#65-#69 を再利用**、
**#70 (camera 再校正) は本フェーズで close (demo 後の別 Issue に降格)**。

### M6R-1: lidar_localization_ros2 vcs import + smoke test (Issue #65)

- **目的**: rsasaki0109 fork v1.1.0 (BSD-2) を `src/third_party/` に取り込み、
  今日の GLIM 出力地図 (`campus-half-v3`) を渡して `map -> odom` TF と
  `/pcl_pose` が出ることを bag replay で確認
- **受け入れ基準**:
  - [ ] `whill_lab.repos` に `third_party/lidar_localization_ros2` 追加、
    `version: <commit-sha>` で pin (ADR-0006 で SHA 決定)
  - [ ] `vcs import` + `colcon build --packages-up-to lidar_localization_ros2` 成功
  - [ ] `scripts/m6r_smoke_test.sh` (新規) で今日の GLIM 出力 PCD を
    `map_path` に渡し、`docs/m5r-bench-data/2026-07-07-campus-half-v3/bag` を
    `ros2 bag play --clock` で流して `tf2_echo map odom` が連続出力
  - [ ] `/pcl_pose` `/alignment_status` フィールド構成を
    `docs/ja/m6r-localizer-eval.md` に記録 (M6R-3 の failsafe 閾値設計の入力)
  - [ ] ADR-0006 を **proposed** で起案
- **依存**: 前段 = M5-R + PR #61 + 今日の GLIM 完了
- **担当**: `ros2-implementer` → `code-reviewer`
- **ブランチ**: `m6r/1-localizer-smoke-test`

### M6R-2: EKF 統合 + `/initialpose` 運用 (Issue #66)

- **目的**: `odom_bringup_launch.py` に localizer を組み込んだ
  `m6r_bringup_launch.py` (`whill_safety` パッケージ内) を作成。TF 衝突が
  ないことを確認、`/initialpose` を RViz 2D Pose Estimate で受ける運用
- **受け入れ基準**:
  - [ ] `m6r_bringup_launch.py` で sensors + driver + EKF + localizer が
    1 コマンド起動
  - [ ] `ros2 run tf2_tools view_frames` で `map -> odom -> base_link ->
    sensors` の 4 段一本鎖、publisher の唯一性成立
  - [ ] RViz 2D Pose Estimate → `/pcl_pose` が 10 秒以内、`< 1.0 m` で収束
  - [ ] 旧 `odom_bringup_launch.py` との排他性を README に明記 (G7)
- **依存**: M6R-1
- **担当**: `ros2-implementer` → `debugger` → `code-reviewer`
- **ブランチ**: `m6r/2-localizer-ekf-integration`

### M6R-3: `whill_safety` パッケージ (failsafe + twist_mux) (Issue #67)

- **目的**: 新規 ament_python パッケージ `whill_safety` を作成。
  `failsafe_node` (3 層購読 → 判定) + `twist_mux` (優先度切替) を実装
- **受け入れ基準**:
  - [ ] `src/whill_safety/` パッケージ新規作成 (package.xml, setup.py)
  - [ ] `failsafe_node` 実装 (購読: `/reinitialization_requested`,
    `/alignment_status`, `/pcl_pose`; publish: `/cmd_vel_safety`)
  - [ ] `config/twist_mux.yaml` で優先度: `cmd_vel_safety` (100) >
    `cmd_vel_nav` (10)
  - [ ] `launch/safety_launch.py` で `failsafe_node` + `twist_mux` を起動、
    `m6r_bringup_launch.py` から include
  - [ ] G4 手動試験 (`/reinitialization_requested` 手動 publish → cmd_vel
    zero 化 `< 200 ms`)
  - [ ] `/alignment_status` フィールド構成を `docs/ja/m6r-failsafe-design.md`
    に記録
  - [ ] ADR-0007 を **proposed** で起案
- **依存**: M6R-2 (`/pcl_pose` / `/alignment_status` 実流入手可能)
- **担当**: `ros2-implementer` → `debugger` → `code-reviewer`
- **ブランチ**: `m6r/3-failsafe-node`

### M6R-4: Nav2 obstacle layer 復活 + collision detection ON (Issue #68)

- **目的**: `pointcloud_to_laserscan` で QoS bridge、`nav2_params.yaml` に
  obstacle layer 追加、`use_collision_detection: true` 復帰、default map
  yaml を新地図に向け直し
- **受け入れ基準**:
  - [ ] `whill_navigation/launch/nav_launch.py` の default map yaml を
    `docs/maps/campus-half-v3/occupancy.yaml` に変更、`site` launch
    argument 追加
  - [ ] `pointcloud_to_laserscan` node を `nav_launch.py` に追加、
    `/velodyne_points` → `/scan`
  - [ ] `nav2_params.yaml` の local/global costmap `plugins` に
    `obstacle_layer` 追加、`observation_sources: scan`
  - [ ] `FollowPath.use_collision_detection: true` 復帰、旧コメント更新
  - [ ] G5 手動試験 (RViz local_costmap に歩行者が現れる)
- **依存**: M6R-2
- **担当**: `ros2-implementer` → `debugger` → `code-reviewer`
- **ブランチ**: `m6r/4-nav2-obstacle-layer`

### M6R-5: G1-G3 統合受入 + M7 引き渡し文書 (Issue #69)

- **目的**: M6R-1〜4 統合状態で G1-G3 (+ G4-G7) を全て pass。7/19 実機
  テストで判定
- **受け入れ基準**:
  - [ ] G1-G7 全て pass、エビデンス (bag, ヒストグラム PDF, スクショ) を
    `docs/m6r-bench-data/2026-07-19-integration/` に保存
  - [ ] `docs/ja/m6r-pipeline.md` (新規) に運用手順記載:
    - 起動コマンド (2 段: `m6r_bringup_launch.py` + `nav_launch.py`)
    - initial pose 指定方法
    - 異常時復旧手順
    - CLAUDE.md P1-P3 / P5-nav 解消経路の最終形
  - [ ] ADR-0006 と ADR-0007 が `accepted` 化
  - [ ] CLAUDE.md 「進行中の既知課題」P1 / P2 / P3 / P5 を解消マーク
- **依存**: M6R-1〜M6R-4 全完了
- **担当**: `ros2-implementer` (文書化) → `code-reviewer`
- **ブランチ**: `m6r/5-integration-test`

### 旧 M6R-6 (camera_link 再校正、Issue #70)

**本フェーズから除外**。Issue #70 に「demo 後に別 Issue で対応」コメントを
追加して close 予定。理由は §2.2 参照。

## 7. 実行順序と依存

```
M6R-1 (localizer smoke test)          ← 今夜〜7/9 目安
   │  ADR-0006 proposed 起案
   ▼
M6R-2 (EKF integration)               ← 7/10-12 目安
   │
   ├──▶ M6R-3 (failsafe)  ┐          ← 7/12-16 目安 (並列可)
   │       ADR-0007 proposed
   │                       │
   └──▶ M6R-4 (Nav2)      ┘          ← 7/12-16 目安
                          ▼
                M6R-5 (G1-G3 integration test)   ← 7/17-19 目安
                          │
                          ADR-0006 accepted
                          ADR-0007 accepted
                          CLAUDE.md P1-P3/P5 解消反映

7/19: Go/No-Go 判定 (G1-G3 全て pass → GO)
7/20-25: 追加チューニング + M7 (dispatch) 実装ウィンドウ
7/25: 完成目標
8/1:  オープンキャンパスデモ
```

単一開発者 + 実機共有なので M6R-3 / M6R-4 の並列化は判断次第 (直列が
安全)。M6R-3 のデバッグが長引く場合は M6R-4 を別ブランチで先行 merge も可。

### ADR 起草タイミング

- **ADR-0006 (localizer 選定)**: M6R-1 着手時に **proposed** 起案、M6R-5
  完了時 accepted 化。IMU 配線既定 `use_imu: false` を Decision に明記
- **ADR-0007 (failsafe 設計)**: M6R-3 完了直前に **proposed** 起案、
  M6R-5 完了時 accepted 化

## 8. 検証戦略

### 8.1 各 Issue の実機検証

| Issue | ユーザー側で実施する検証 |
|-------|--------------------|
| M6R-1 | (1) vcs 展開 + colcon build 成功、(2) 今日の bag replay で `tf2_echo map odom` 継続出力 |
| M6R-2 | (1) `m6r_bringup_launch.py` で 4 段 TF、(2) RViz `/initialpose` → `/pcl_pose` 収束 |
| M6R-3 | (1) `/reinitialization_requested` 手動 publish で cmd_vel `< 200 ms` zero、(2) センサ視野遮蔽で failsafe 発火 |
| M6R-4 | (1) Nav2 起動、local_costmap に歩行者反映、(2) 経路上の障害物で停止 or 回避 |
| M6R-5 | 7/19 実機テストで G1-G3 + G4-G7 全項目、エビデンス保存 |

### 8.2 ベンチデータ規約

`docs/m6r-bench-data/<YYYY-MM-DD>-<run-id>/` に bag (gitignored)、README、
派生中間ファイル (TF ヒストグラム PDF、score 推移 PNG は tracked) を置く。
M4-R / M5-R 規約を踏襲。

### 8.3 G1-G3 合格根拠

- **G1 (収束時間 < 10 s)**: NDT_OMP の典型初期収束時間 (rsasaki0109
  README) が 3-5 s。屋外ノイズ環境で 2 倍のマージン
- **G2 (TF 飛び < 0.5 m)**: WHILL 最大速度 0.3 m/s × 30 Hz TF period =
  10 mm/frame。0.5 m は 50 倍で NDT 補正の正常範囲外を検出する保守閾値
- **G3 (ランドマーク再現性 < 0.5 m)**: Nav2 の goal_tolerance 相当。
  配車デモとして許容できる終端誤差の上限

## 9. リスクと不確実性

### 9.1 リスク

- **R1: 新地図で lidar_localization_ros2 の NDT が発散**
  - 原因候補: PCD voxel size と NDT resolution の不整合
  - 緩和: M6R-1 で複数 voxel で smoke test、ADR-0006 に確定値記載。時間
    不足時は campus-loop bag (1 本目) で代替 GLIM も試す
- **R2: 屋外歩行者ジャンプで NDT スコア劣化 → localizer 失探**
  - 緩和: DUFOMap 動的除去済 static map に対する scan-to-map なので原理
    的に影響小、加えて failsafe (G4) で cmd_vel 遮断
- **R3: 7/19 で G3 (ランドマーク再現性) 不合格**
  - 緩和策 A: 経路を短縮する
  - 緩和策 B: 7/20-22 に追加チューニング (NDT resolution, EKF Q/R, TF
    latency)
  - 緩和策 C: M7 実装ウィンドウを圧縮する
- **R4: 屋外 WiFi 瞬断で live monitoring 途切れ**
  - 今日実測 (2026-07-07): CycloneDDS が本機自身の WiFi 経路を「peer」
    と誤認識、UDP write 失敗が log 汚染。**録画には無影響**
  - 緩和: 一次観測は bag record、live topic echo は補助扱い
- **R5: PCMK-G3X (RT IMU) のバイアス誤差が failsafe 閾値を跨いで頻発**
  - 既知 (Issue #64)。M6-R では EKF Q 分散を上げて許容 (localizer は
    `use_imu: false` なので独立影響なし)、恒久対応は demo 後

### 9.2 不確実性

- **U1: `/alignment_status` の実フィールド構成**
  - M6R-1 で実測して `docs/ja/m6r-localizer-eval.md` に記録
- **U2: BBS_2D global relocalization の収束時間**
  - M6R-2 で実測。走行中の re-seed 用途は demo スコープ外 (initial pose は
    静止時のみ RViz で投入する運用)
- **U3: 屋外街灯下 (夜間走行) の LiDAR ノイズ挙動**
  - 今日の bag は薄暮〜夜間取得。反射光の false return が NDT スコアに
    与える影響を M6R-1 smoke test で観測

## 10. 後続フェーズへの引き渡し

- **M7 (dispatch)**: 7/20 頃に `m6r_bringup_launch.py + nav_launch.py`
  2 段起動で「initial pose → goal 送信 → 走行 → 到着」の最低構成が動く
  状態を引き渡す
- **M8 (Web / タブレット UI)**: M7 完了後、rosbridge 経由の接続。demo
  では最小限の UI (地点選択 + 呼び出し 1 ボタン)
- **8/1 オープンキャンパスデモ**: 7/25 完成目標、7/25-31 が予備日
  + 現地下見 + リハーサル
- **Demo 後 (品質改善タスク)**:
  - Issue #64 GLIM IMU 警告根本原因 (GRIL-Calib で allan variance +
    T_lidar_imu 再校正)
  - GLIM loop closure 改善 (motion-rich bag 収録)
  - camera_link target-based 再校正 (旧 M6R-6)

## 11. ADR の候補

- **ADR-0006**: localizer 選定 (lidar_localization_ros2 rsasaki0109 v1.1.0
  BSD-2)。M6R-1 で proposed、M6R-5 で accepted
- **ADR-0007**: failsafe ノード設計 (3 層購読 + twist_mux 優先度)。
  M6R-3 で proposed、M6R-5 で accepted
- **ADR 候補 (demo 後)**: camera_link 再校正、IMU/T_lidar_imu 再校正、
  GLIM loop closure 改善 (旧 M6R-6 と Issue #64 を吸収)

## 12. 次のアクション

1. **今日 (2026-07-07)**:
   - [x] 屋外 bag 収録 2 本 (campus-loop, campus-half-v3)
   - [x] bench data metadata + NOTES 書き起こし
   - [x] campus-half-v3 に GLIM 適用 (real 208.5 s, 4579 keyframes)
   - [x] loop closure 状況確認 (未発火、想定内)
   - [ ] GLIM 出力 → DUFOMap → occupancy grid → `docs/maps/campus-half-v3/`
     作成 (M6R-1 着手前に完了させる)
   - [ ] 本計画書 + bench data を commit → `claude/localization-imu-bottleneck-9awrnm`
     ブランチ push
2. **7/8-9**: M6R-1 (lidar_localization_ros2 vcs import + smoke test) 着手
3. **7/10-16**: M6R-2 → M6R-3 + M6R-4 (並列)
4. **7/17-19**: M6R-5 統合テスト、7/19 G1-G3 判定
5. **7/20-25**: M7 (dispatch) 実装 + チューニング
6. **7/25-31**: 現地リハーサル
7. **8/1**: オープンキャンパスデモ本番

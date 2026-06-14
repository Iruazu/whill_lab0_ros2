# M4-R 実行計画: odom 基盤の再構築

Language: [日本語](2026-06-13-m4r-execution.md) | [English](../../en/plans/2026-06-13-m4r-execution.md)

- 日付: 2026-06-13
- 状態: accepted (Iruazu、2026-06-14)
- 親方針: [`docs/ja/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md) §4 (M4-R), §6 (受け入れ基準)
- 想定配置: `docs/ja/plans/2026-06-13-m4r-execution.md`
- 読者: 本フェーズに着手する `ros2-implementer` / `debugger` / `code-reviewer` および
  実機検証を実施するユーザー

## 0. ユーザー要件の理解

親方針 §4 で「M4-R = odom 基盤再構築」と定められた本フェーズを、
Issue 単位の実行可能な粒度に分解する。受け入れ基準は親方針 §6 で
3 項目固定 (TF 一本鎖 / `/odometry/filtered` 公開 / `tf_bridge_launch.py` 廃止)。
本書はその 3 項目を満たすための作業順序・依存・実機検証手順を確定させる。

## 1. 背景

### 1.1 なぜ M4-R を最初に置くか (親方針 §4 末尾の再掲)

M6-R (scan-to-map localizer) を先に入れると `map -> base_link` に補正ジャンプが
直接コントローラへ届き、車椅子が急 jerk を生む。REP-105 では `map -> odom` の
不連続を `odom -> base_link` の連続フレームが吸収する設計になっており、
M6-R を安全に導入するには先に `odom -> base_link` を成立させる必要がある。
これが M4-R を最初に持ってくる唯一の理由。

### 1.2 解消する既知課題

親方針 §2 の診断 P4 と P2 の一部:

| ID | 内容 | M4-R での解消経路 |
|----|------|----------------|
| P4 | odom フレーム不在・車輪オドメトリ未使用。補正導入時のジャンプ緩衝材がなく LiDAR 縮退時のバックアップもない | wheel + IMU の EKF で `odom -> base_link` を構築 |
| P2 (一部) | base_link → 各センサが identity placeholder のまま | LiDAR↔IMU 実測値 (`docs/ja/m3-extrinsics-from-noetic.md`) を `base_link -> imu_link` / `base_link -> velodyne` に展開し、現状の identity を置換する。P2 全体 (初期位置合わせ機構) の解消は M6-R に持ち越し |

### 1.3 残置する課題 (M4-R では触らない)

- P1 (map 補正経路の不在) → M6-R
- P2 残り (initial pose UI / 任意地点起動) → M6-R
- P3 (発散検知・フェイルセーフ) → M6-R
- P5 (地図品質・obstacle layer) → M5-R / M6-R

## 2. 作業範囲

### 2.1 扱うもの

1. **/whill/odom の供給**: 上流 `whill_driver` は `/whill/states/model_cr2`
   (`whill_msgs/ModelCr2State`) しか publish していない (確認: `src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:41-44`)。
   `Iruazu/ros2_whill` fork (M2 で cold-boot init パッチ実績あり、`whill_lab.repos` で指定) に
   publisher を追加して、`whill_driver` 自身が左右モータ角度・速度から `nav_msgs/Odometry`
   を計算し `/whill/odom` に publish する (案 1。詳細は §6 M4R-1)
2. **robot_localization EKF**: `/whill/odom` + `/imu/data_raw` を入力に
   `odom -> base_link` の TF と `/odometry/filtered` topic を出す
3. **base_link 中心の static TF 実測値反映**: 現状 identity の
   `base_link -> imu_link` / `base_link -> velodyne` / `base_link -> camera_link` を、
   noetic から引き継いだ LiDAR↔IMU 実測値および新規測定の base_link 基準値に置換する
4. **`tf_bridge_launch.py` の削除**: 親方針 §5 禁止事項 1 と §6 受け入れ基準 3
5. **新 bringup launch**: sensors + whill_driver (fork パッチで `/whill/odom` も publish) + EKF を一括起動
6. **検証文書**: `docs/m4r-bench-data/<run>/` 形式の実機ログ置き場と README 雛形

### 2.2 扱わないもの (明示)

- **FAST-LIO に触らない**: 親方針 §5 禁止事項 2。`src/whill_localization/launch/fast_lio_launch.py` および `velodyne_whill.yaml` は M5-R までフリーズ
- **GPS / navsat_transform**: WIP の `navsat_transform_launch.py` / `navsat_transform.yaml` は M4-R には含めない (理由は §3 で詳述)
- **Nav2 再統合**: `src/whill_navigation/launch/nav_launch.py` は `tf_bridge_launch.py` 削除に伴い壊れた状態で M4-R 完了とする。Nav2 への TF 再配線・obstacle layer 復活は M6-R 担当
- **`use_collision_detection: false` の見直し**: §5 禁止事項 3 と M6-R の担当範囲
- **本リポ内 `src/third_party/ros2_whill/` ローカル展開ディレクトリの直接編集**: 親方針
  §5 禁止事項 5。`/whill/odom` 供給は `Iruazu/ros2_whill` fork に publisher を追加する形で
  実施する (案 1。fork URL は `whill_lab.repos` で既に指定済、M2 で cold-boot init パッチ実績あり)。
  ローカルディレクトリへの直接編集禁止と fork パッチは別軸の規約 (詳細は §6 M4R-1 の規約解釈訂正)

## 3. 既存 WIP コードの扱い

Phase A〜C (M5-e サブフェーズ) は Issue #28 (2026-06-13 merge) で正式に凍結された。
アーカイブタグ `legacy/m5e-phase-abc-2026-06-13` で参照可能 (origin/m5e/velodyne-self-filter HEAD = 9b5be71)。

凍結対象に含まれていた `src/whill_odometry/` パッケージ (Phase A コミット 204989e で作成、
276 行の C++ ノード) は **main には取り込まない** (ユーザー判断 α、2026-06-13)。
ただし設計判断の参考材料としては archive を **読む** ことは許容される (α 判断と整合)。

詳細は `docs/ja/legacy-findings/2026-06-13-m5e-frozen.md` を参照。

### §3.A navsat_transform WIP の温存方針

リポ untracked の以下 2 ファイルも M4-R には組み込まない:

- `src/whill_localization/config/navsat_transform.yaml`
- `src/whill_localization/launch/navsat_transform_launch.py`

判断根拠:

1. 親方針 §3.2 では `map -> odom` は scan-to-map localizer の担当
   (M6-R)、GPS は §7 で「屋外拡張時の GNSS/RTK 統合」として ADR 候補 (未決) の扱い。
   M4-R に GPS を組み込むと「親方針が選定すべき層に先回りで実装が入る」状態になる
2. ただし当該 WIP は単なる実験ではなく、8月オープンキャンパスデモを意図した
   「固定 datum (36.550814, 139.928684) で map フレームをアンカーする」設計
   (yaml ヘッダコメント参照)。これは親方針が要件 R2 (永続的な map 座標系) を
   scan-to-map で実現することを宣言した方向性とは別経路の解。単純削除でなく温存する
3. M4-R の受け入れ基準 3 項目はどれも GPS なしで成立する。組み込むと EKF 二段構成
   (ekf_odom + ekf_map) になり M4-R のスコープが膨張する

WIP 2 ファイルは現在のリポ状態 (untracked) のまま M4-R 期間中は触らない。
M4-R 完了後、別 Issue で次のいずれかに進める判断をする:

- (a) ADR 起案: 「8月デモに向けて GPS datum で map を固定する暫定経路を取るか」
  を `docs/decisions/` に提出し、親方針 §3.2 (scan-to-map で map→odom) と
  R2 の代替実現策として併存させるか単独選定するか決める
- (b) 親方針 §7 の「GNSS/RTK 統合 ADR」の入力資料として参照し、
  M5-R / M6-R が確定した後に再評価する
- (c) 不採用なら削除し、commit 履歴は別 ADR に残す

## 4. 前提条件

- 親方針 §3.3 採用候補の robot_localization EKF を odom 融合に使う
  (`source /opt/ros/humble/setup.bash` 環境に既にバイナリ存在)
- 上流 `ros2_whill` は `whill_msgs/ModelCr2State` を `/whill/states/model_cr2` に
  publish する。motor_angle は累積角 (rad ではなく `whill.cpp` の生変換 = deg 単位
  の可能性あり)、motor_speed は瞬時速度 (km/h かもしれない)。
  単位は実機で 1 回転試験して確定する (Issue M4R-1 で扱う)
- 車体パラメータの初期値: 車輪半径 0.1325 m、tread 0.520 m を URDF
  (`src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf:62, 99`)
  から取る。ただし `whill_node.cpp:115` のコメントは tread = 0.496 m を明示しており
  矛盾する。Issue M4R-1 で実測または noetic 旧実装 (`ros_whill/ros_whill.cpp`)
  との突き合わせで確定する
- IMU は既に `whill_sensors_bringup/imu_launch.py` でライフサイクル auto-activate
  され `/imu/data_raw` (100 Hz) を出している。EKF はこれをそのまま使う
- LiDAR↔IMU 実測値は `docs/ja/m3-extrinsics-from-noetic.md` に転記済み。これは
  「IMU フレームで表した LiDAR 原点」なので、`base_link -> imu_link` を
  別途決めた後、`base_link -> velodyne = base_link->imu_link * imu_link->velodyne`
  で展開する。`base_link` の物理定義 (座席中心 / 後輪車軸中心 / 車体中央) は
  Issue M4R-3 で確定する
- 実機検証は全てユーザー手押し or ジョイスティック操作。Claude は launch /
  メトリクス取得スクリプトまで用意し、実走行は依頼する (CLAUDE.md 規約)

## 5. 受け入れ基準 (親方針 §6 + 本計画の補強)

親方針 §6 の M4-R 3 項目を、観測可能なコマンドと期待値に展開:

- [ ] **A1: TF 一本鎖**
  - コマンド: `ros2 launch whill_localization odom_bringup_launch.py` (新規)
    起動後、別端末で `ros2 run tf2_tools view_frames`
  - 期待: 生成された `frames.pdf` に `map` ノードが存在しない or 孤立し、
    `odom -> base_link -> {imu_link, velodyne, camera_link}` の一本鎖が成立
  - (注: M4-R では `map` は誰も publish しない。`map -> odom` は M6-R で復活させる)
- [ ] **A2: `/odometry/filtered` の品質**
  - コマンド: `ros2 topic hz /odometry/filtered` で 30 Hz 前後 (EKF 既定)、
    `ros2 topic echo /odometry/filtered --once` で frame_id=odom / child_frame_id=base_link が出る
  - 手押し 10 m 直進試験: 起点で `ros2 service call /set_pose ...` 相当でリセット後、
    10 m 直進 (床にメジャーで線を引く)。`/odometry/filtered.pose.pose.position` の
    終端値と実測 10 m の差を記録
  - **合格閾値の提案 (本計画で確定): 終端誤差 ≤ 0.5 m (= 5%)**。
    根拠: WHILL Model CR2 の純粋なホイールオドメトリは出荷時校正で 2-3% 程度
    と期待できる。IMU 融合により回転誤差は減るが、直進では tread 推定誤差・
    タイヤ径の温度依存・床面スリップが残り 3-5%。
    10 m で 5% = 0.5 m は M4-R の合格線として現実的。
    M5-R 以降の scan-to-map で吸収可能な範囲
- [ ] **A3: `tf_bridge_launch.py` 削除**
  - `ls src/whill_navigation/launch/tf_bridge_launch.py` で No such file
  - `colcon build --packages-select whill_navigation whill_localization
    whill_sensors_bringup` が成功
  - `ros2 launch whill_localization odom_bringup_launch.py` が起動可能
    (`nav_launch.py` が壊れることは本フェーズでは許容。M6-R で復旧)

補強基準 (本計画で追加。code-reviewer が確認):

- [ ] **A4: extrinsic の根拠が文書化**
  - `base_link -> imu_link` の値が `docs/ja/legacy-findings/<topic>.md` か
    `docs/ja/m3-extrinsics-from-noetic.md` の追補節に根拠付きで記載
- [ ] **A5: launch 排他性の明示**
  - 新 `odom_bringup_launch.py` と旧 `localization_launch.py` の同時起動を防ぐ
    (両者が `base_link -> imu_link` を競合 publish しないこと)。README に明示

## 6. Issue 分割案

以下 4 件に分割する。各 Issue は単体で build/launch まで通る粒度。

### Issue M4R-1: `Iruazu/ros2_whill` fork に `/whill/odom` publisher 追加 (案 1)

- **目的**: 既存の `Iruazu/ros2_whill` fork (`whill_lab.repos` で指定済、M2 で cold-boot init
  パッチ実績あり) に新ブランチ `feature/add-odom-publisher` を切り、`whill_node.cpp::OnStatesModelCr2Timer()`
  を拡張して `nav_msgs/Odometry` を `/whill/odom` に publish する。fork で PR → merge →
  タグ付与 → 本リポの `whill_lab.repos` の version を新タグに更新する一連の作業
- **方針判断 (2026-06-14 確定)**:
  - **案 1 採用** (fork パッチ)。**案 2 (`whill_odometry` パッケージ新設) は不採用**:
    本リポでラッパーノード化すると 2 ノード構成になり「ドライバが状態を吐く」設計と乖離する。
    M2 と一貫した fork パッチパターンが既に動いており、上流 (whill-labs/ros2_whill) も
    `upstream` remote として追跡可能 (背景は後日起案する ADR-0002 に転記予定)
  - **規約解釈訂正**: 親方針 §3.4 と §5 の「`src/third_party/` 編集禁止」は **本リポ内**
    `src/third_party/` ローカル展開ディレクトリの直接編集を禁じる規約であり、
    上流 GitHub リポを fork して `whill_lab.repos` を fork URL に向けることは規約と独立。
    既に `Iruazu/ros2_whill` fork で M2 パッチ実績あり (`whill_lab.repos`)
- **単位と符号 (Issue #30 で確定済)**:
  - `motor_angle` = rad、`motor_speed` = km/h (上流 `whill.cpp:62-69` のコメント)
  - wrap 処理: ROS 2 標準 `angles::shortest_angular_distance()` (`ros-humble-angles`)
  - odometry 方式: **角度ベース** (publish 頻度 ~3 Hz への頑健性)
  - 符号: **右輪を反転** (`d_right = -angles::shortest_angular_distance(...)`、`d_left = angles::shortest_angular_distance(...)`)
  - `WHEEL_RADIUS = 0.1325 m`、`TREAD = 0.496 m` (公称、`docs/ja/m4r-whill-units.md` の採用値)
  - 詳細な C++ コード例は `docs/ja/m4r-whill-units.md` の「M4R-1 への転記」節
- **受け入れ基準**:
  - [ ] `Iruazu/ros2_whill` fork に `feature/add-odom-publisher` ブランチ作成、PR #2 として open
  - [ ] `whill_driver/package.xml` に `<depend>angles</depend>`、`<depend>nav_msgs</depend>`、`<depend>tf2</depend>`、`<depend>tf2_geometry_msgs</depend>` 追加
  - [ ] `whill_driver/CMakeLists.txt` に `find_package(angles REQUIRED)` 等の追加と `target_link_libraries(... angles::angles)`
  - [ ] `whill_node.cpp` で `/whill/odom` (`nav_msgs/Odometry`) の publisher 作成、`OnStatesModelCr2Timer()` 内で角度ベース odometry を計算 (`docs/ja/m4r-whill-units.md` のコード例ベース)
  - [ ] fork で PR merge 後、annotated タグ `humble-with-odom-2026-MM-DD` を付与
  - [ ] 本リポの `whill_lab.repos` の `third_party/ros2_whill:` version を新タグに更新、`vcs import` 再実行で fork の新版が取り込まれる
  - [ ] `colcon build --packages-up-to whill_driver` 成功
  - [ ] `ros2 topic echo /whill/odom --once` で `nav_msgs/Odometry` が出力される
  - [ ] 実機ジョイスティック前進 1 m で `/whill/odom.pose.pose.position.x` が +1 m ± 5% で増加 (符号正、距離整合)
  - [ ] 実機ジョイスティック左旋回 90 度で `/whill/odom.pose.pose.orientation` の yaw が +π/2 ± 5% で増加 (REP-103 慣習)
- **スコープ外**:
  - EKF 統合 (M4R-3)
  - TF publish (M4R-1 では `publish_tf: false` 相当、TF は EKF が一手に publish)
  - 本リポでの `whill_odometry` パッケージ新設 (案 2 不採用)
  - `whill_node.cpp` の他機能改修 (cold-boot init パッチは既存通り)
- **前提仮定**:
  - Issue #30 完了 (`docs/ja/m4r-whill-units.md` に検証結果記入済)
  - Issue #28 完了 (Phase A〜C 凍結確定、archive 参照のみ許可)
  - WHILL Model CR2 実機が利用可能
- **想定ブランチ (本リポ側)**: `m4r/1-fork-add-odom-publisher` (本リポでは `whill_lab.repos` 更新コミットのみ、本体パッチは fork で)
- **fork 側のブランチ**: `Iruazu/ros2_whill/feature/add-odom-publisher`

### Issue M4R-2: `base_link` 静的 TF を実測値で置換

- **目的**: `whill_sensors_bringup/static_tf_launch.py` の 3 つの identity を
  実測値に置換する
- **受け入れ基準**:
  - [ ] `base_link -> imu_link` / `base_link -> velodyne` / `base_link -> camera_link`
    が 0 でない値で publish される
  - [ ] `ros2 run tf2_tools view_frames` で 4 リンクのツリーが描画
  - [ ] `docs/ja/m3-extrinsics-from-noetic.md` の追補節に
    「base_link の物理定義 (どの点を 0 にしたか)」と「3 個の extrinsic 算出経路」
    が記載
- **スコープ外**: カメラの厳密 extrinsic 再キャリブ。M3 時点の取得が不十分なら
  簡易測定 (メジャー実測) で先に進む。本格再キャリブは M5-R の地図品質要求が
  出てから判断
- **前提仮定**: noetic の LiDAR↔IMU 値は `extrinsic_T = LiDAR_in_IMU` 形式。
  `base_link` を「後輪車軸中心、地面高さ」と仮定するなら、
  IMU は座席下クッション (+0.324 m 高さ - 0.412 m 後方の noetic 注釈) から逆算する
- **想定ブランチ**: `m4r/2-base-link-static-tf`

### Issue M4R-3: robot_localization EKF (ekf_odom) 導入

- **目的**: `/whill/odom` + `/imu/data_raw` を入力に EKF を回し、
  `odom -> base_link` TF と `/odometry/filtered` を出す
- **受け入れ基準**:
  - [ ] `ros2 topic hz /odometry/filtered` が 30 Hz ±5 Hz
  - [ ] `ros2 run tf2_tools view_frames` で `odom -> base_link` が EKF から
    publish されている (fork パッチを当てた `whill_driver` 側は TF publish しない)
  - [ ] 手押し 10 m 直進で終端誤差 ≤ 0.5 m (基準 A2)
- **スコープ外**: navsat / map フレーム / 二段 EKF。本 Issue では `world_frame: odom`
  かつ `two_d_mode: true` (キャンパス屋内は実質平面) の単段 EKF のみ
- **前提仮定**: IMU の `orientation_covariance[0] = -1` (RT 9 軸ドライバの
  生 IMU は orientation 不定) のため、EKF は `imu0` から `roll/pitch/yaw` ではなく
  `angular_velocity_*` と `linear_acceleration_*` のみを取り込む。yaw は
  `/whill/odom` 由来とする
- **想定ブランチ**: `m4r/3-ekf-odom`

### Issue M4R-4: `tf_bridge_launch.py` 削除 + 新 bringup launch + 文書更新

- **目的**: 親方針 §5 禁止 1 の解消と、新 TF 構造を起動する正規 launch を提供
- **受け入れ基準**:
  - [ ] `src/whill_navigation/launch/tf_bridge_launch.py` が削除済
  - [ ] `src/whill_navigation/launch/nav_launch.py` の include 行も削除
    (壊れた状態の nav_launch.py は M6-R で復旧。README に「現在 Nav2 は M6-R 待ち」と明記)
  - [ ] 新 `whill_localization/launch/odom_bringup_launch.py` で
    sensors + whill_driver (fork パッチ済、`/whill/odom` も publish) + EKF が一括起動
  - [ ] `docs/m4r-bench-data/README.md` (雛形) と 10 m 直進試験の手順書が同梱
  - [ ] CLAUDE.md の「進行中の既知課題」P4 の解消を本フェーズ完了時に reflect
    (commit 別、本 Issue では文書下書きまで)
- **スコープ外**: nav_launch.py の Nav2 復旧 (M6-R)
- **前提仮定**: `whill_driver` は上流 `whill_bringup/launch/whill_launch.py` 経由で
  起動する。`port_name` は udev rule で `/dev/whill` のような symlink にしておくのが
  望ましいが、未整備なら本 Issue で udev rule 追補
- **想定ブランチ**: `m4r/4-bringup-and-retire-tf-bridge`

## 7. 実行順序と依存

```
M4R-1 (fork add /whill/odom publisher)
   │
   ├──> M4R-3 (EKF) ──┐
   │                  │
M4R-2 (static TF) ────┤
   │                  │
   └──────────────────┴──> M4R-4 (bringup launch + tf_bridge 削除)
```

- M4R-1 と M4R-2 は依存なし、並列実行可能 (別ブランチ)
- M4R-3 は M4R-1 (whill_odom topic) と M4R-2 (base_link TF) の両方が必要
- M4R-4 は M4R-3 完了が前提

並列化の現実的判断: 単一開発者・実機共有の状況では M4R-1 → M4R-2 → M4R-3 → M4R-4
の直列が安全。並列にすると EKF デバッグ時に「whill_odom 側が悪いのか TF 側か」の
切り分けに余計なコストがかかる。

## 8. 検証戦略

### 8.1 各 Issue の実機検証

| Issue | ユーザー側で実施する検証 |
|-------|--------------------|
| M4R-1 | (1) ジョイスティックで現場でその場 1 回転、yaw 累積を `ros2 topic echo /whill/odom` で読み取り。(2) 手押し 1 m 直進で `pose.position.x` 読み値を実測値と比較 |
| M4R-2 | RViz で `base_link` 固定、各センサ frame の位置オフセットが目視で「車体上の物理位置と一致」することを確認 |
| M4R-3 | (1) `ros2 topic hz /odometry/filtered` が 30 Hz。(2) 手押し 10 m 直進、`/odometry/filtered` 終端誤差 ≤ 0.5 m。(3) 静止状態で 30 秒、yaw ドリフトが ≤ 0.1 rad |
| M4R-4 | `ros2 launch whill_localization odom_bringup_launch.py` 単一コマンドで全部上がる、`view_frames` で M4-R 完成形の TF ツリーが出る |

### 8.2 ベンチデータ規約

`docs/m4r-bench-data/<YYYY-MM-DD>-<run>/` に以下を保存:

- `bag/` (ros2 bag。`/whill/states/model_cr2`, `/whill/odom`, `/imu/data_raw`,
  `/odometry/filtered`, `/tf`, `/tf_static`)
- `README.md` (取得日、車体、床面、操作内容、終端誤差実測値)
- 個別の生バグ (rosbag) は gitignore (CLAUDE.md 既存規約)、README と PDF のみ commit

### 8.3 「手押し 10 m 直進」の合格基準値 提案根拠

- WHILL Model CR2 の出荷時ホイール校正精度: 公称 ±2-3% (社内資料未確認、
  業界標準的レンジ。確定値はメーカーに問合せ可能だが M4-R の合格判定には不要)
- IMU 融合による回転誤差低減効果: 直進では限定的 (yaw 推定はホイールオドメトリと
  IMU 角速度の重み付き平均で、直進中は angular_velocity_z ≈ 0 なのでホイール側支配)
- 床面スリップ: 屋内タイルカーペットなら ≤ 1%
- 合計: ≤ 0.5 m / 10 m = 5% を合格、≤ 0.3 m を「ホイール校正が想定通り」のシグナル

この閾値は M4-R 単独での合格判定にのみ使う。配車運用 (M9) の精度要件は
M6-R で scan-to-map が補正経路を入れたあとの map 座標系で別途定義する。

## 9. リスクと不確実性

### 9.1 リスク

- **TF 構造変更が `nav_launch.py` を壊す**: 既知かつ意図通り。M4-R 完了時点で
  Nav2 は起動不可になる。M6-R の最初の作業として nav_launch.py 復旧を組む。
  代替: `nav_launch.py` を `nav_launch.py.disabled` にリネームしてビルドを通す手も
  あるが、隠蔽の方が事故リスク高いため明示的に壊す方を選ぶ
- **ModelCr2State の単位曖昧性** (Issue #30 で解消済): motor_angle = rad、
  motor_speed = km/h を上流 `whill.cpp:62-69` のコメントと実機検証で確定済 (詳細は
  `docs/ja/m4r-whill-units.md`)。本リスクは閉じたが、参考のため本項に残す
- **tread 値の食い違い** (Issue #30 で暫定確定): URDF 0.520 m vs `whill_node.cpp:115`
  コメント 0.496 m のうち、後者 (0.496 m) を採用値とした (`docs/ja/m4r-whill-units.md`
  の「採用値」表)。実測再評価は M4R-1 の 1 m 直進・90 度旋回試験の誤差が許容内なら
  不要、許容外なら再評価する
- **bag record 構造が変わる**: M3 までの bag は FAST-LIO 中心の topic 構成で
  record されている。M4-R で `/whill/odom` `/odometry/filtered` が増える。
  M3 までの bag は引き続き `fast_lio_launch.py` で replay 可能 (互換性維持)。
  M4-R 以降の新 bag が混在しないよう `docs/m4r-bench-data/README.md` で
  ディレクトリ規約を明示
- **`whill_driver` の TF publish**: 上流コード再確認の結果、`whill_node.cpp` は
  TF を publish していない (good。EKF 側で唯一性確保できる)。ただし上流が
  将来バージョンで TF 出力を足すリスクは残るので、bringup launch で
  `whill_driver` のパラメータが TF を抑制する想定で書く (将来の上流追加に備えた
  防御線)
- **EKF の `world_frame: odom` と `map` フレーム不在の整合**: M4-R では誰も
  `map` を publish しない。RViz Fixed Frame を `odom` に切り替える運用にする。
  この期間のスクリーンショットは `Fixed Frame: odom` を明示

### 9.2 不確実性

- **base_link の物理定義**: 「後輪車軸中心 / 地面高さ」を仮置きするが、
  Nav2 の footprint や M5-R の地図原点と整合させる必要が出た時に再定義する
  可能性がある。Issue M4R-2 で仮定義 + コメントを残す
- **IMU の符号系**: RT 9 軸の取付向きが noetic と humble で一致しているか
  未確認。M4R-1 でジョイスティック旋回試験時に IMU `angular_velocity.z` と
  `/whill/odom` の yaw 微分の符号一致を確認

## 10. 後続フェーズへの引き渡し

M4-R 完了時点で次が確定する:

- TF 構造: `odom -> base_link -> {imu_link, velodyne, camera_link}` の一本鎖
- 入力 topic: `/whill/odom`, `/imu/data_raw` (M3 から継続)
- 出力 topic: `/odometry/filtered` (連続・滑らか, 30 Hz)
- launch: `whill_localization/launch/odom_bringup_launch.py`

これを前提に:

- **M5-R**: マップ作成は本 M4-R の launch を使わない (オフライン bag 後処理)。
  ただし bag を取るときの TF tree が M4-R 構造になっていることで、
  生成された PCD 地図と運用時の TF が整合する
- **M6-R**: scan-to-map localizer が新たに `map -> odom` を publish する。
  M4-R の `odom -> base_link` はそのまま EKF が担当し続け、`map -> odom` の
  ジャンプを `odom -> base_link` 連続性が吸収する (REP-105 の意図通り)。
  initial pose UI も M6-R で追加。`nav_launch.py` の Nav2 復旧と
  `tf_bridge_launch.py` 不要化の完成形が M6-R で実現

## 11. ADR の候補

このフェーズで生まれる技術判断のうち ADR 化を検討すべきもの:

- **ADR-0002: `/whill/odom` 供給方式の選択 (案 1 fork パッチ採用)** — 2026-06-14
  ユーザー判断で確定。本体 ADR は `docs/decisions/0002-whill-odom-supply.md`
  として後日起案 (背景・採用しなかった案・規約解釈訂正の事実調査を転記予定)
- [ ] **ADR-0003 候補: GPS datum 経路の扱い** (§3.A 参照)。M4-R 完了後、
  別 Issue で起案

## 12. 次のアクション

本計画書は `accepted` (Iruazu、2026-06-14) で確定済。M4-R 着手に必要な
残作業:

1. M4R-1〜M4R-4 の 4 件を `gh issue create` で起案する (本計画は分割を
   定めるが、`gh issue create` 自体は別ステップ)
2. M4R-1 (案 1 fork パッチ) に着手する。実装スケルトンは
   `docs/ja/m4r-whill-units.md` に揃っており、Issue #30 は close 済
3. M4R-2 (`base_link` 静的 TF 実測値化) は M4R-1 と並列に別ブランチで
   進められる
4. M4R-1 と M4R-2 が main に merge された時点で M4R-3 (EKF) が着手可能、
   M4R-4 (`tf_bridge_launch.py` 廃止 + bringup launch) でフェーズを閉じる

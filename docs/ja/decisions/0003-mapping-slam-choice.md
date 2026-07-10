# ADR 0003: M5-R マップ作成 SLAM の最終選定

Language: [日本語](0003-mapping-slam-choice.md) | [English](../../en/decisions/0003-mapping-slam-choice.md)

- Status: accepted
- Date: 2026-06-22 (Phase A 起案) / 2026-06-21 (Phase B 計測完了、Decision 確定、accepted 化)
- Deciders: Iruazu

## 背景

親方針 [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §7 (ADR 候補) で次が明示されている:

> ADR: マップ作成 SLAM の最終選定。GPU 母艦は確保済み (9 章) のため GLIM 採用の前提条件は満たされた。実 bag での GLIM vs FAST-LIO SAM 比較後に確定する

§3.3 (採用候補表) では GLIM (第一候補、MIT、ROS 2 humble 公式、GPU 母艦で後処理) と FAST-LIO SAM (代替、VLP-16 実績) を 2 候補として列挙し、§3.4 (ライセンス方針) で「permissive (MIT/BSD/Apache) で構成可能な状態を保つ」「GPL 系は『オフラインのマップ作成ツール』としての分離プロセス利用に限定する」と定めている。

M5-R 実行計画 [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md) §6 受け入れ基準 B4 は「ADR-0003 が実 bag 比較結果を根拠に accepted で merged」、B5 は「ライセンス棚卸し記載」を要求する。本 ADR はこれら 2 件を満たす。

### 比較対象 SLAM の現状

| SLAM | 上流 | ライセンス | 本リポでの整備状態 |
|---|---|---|---|
| GLIM | [`koide3/glim`](https://github.com/koide3/glim) + [`koide3/glim_ros2`](https://github.com/koide3/glim_ros2) | MIT | M5R-1 (#45) で源ビルド完了。CUDA 12.4 + cuDNN 8 で母艦インストール済。詳細 [`../m5r-glim-setup.md`](../m5r-glim-setup.md) |
| FAST-LIO SAM | [`RightTr/FAST-LIO-SAM`](https://github.com/RightTr/FAST-LIO-SAM) | **LICENSE 不在** (上流に LICENSE ファイルなし、`package.xml` のみ `BSD` を自己申告)。派生元 FAST-LIO (HKU-MaRS) は **GPL-2.0** で copyleft 伝播の可能性あり | M5R-2 (#46) で clone-on-demand 経路を整備。`FASTLIO_SAM_LICENSE_ACK=yes` ガード付き。詳細 [`../m5r-fastlio-sam-eval.md`](../m5r-fastlio-sam-eval.md) |

### 評価条件

- 入力 bag: 同一の室内ループ走行 bag (50 m 程度、M4-R bringup launch で `/velodyne_points` + `/imu/data_raw` + `/tf_static` を収録)
- 計測ラッパ: `scripts/m5r3_run_glim.sh` と `scripts/m5r3_run_fastlio_sam.sh` (時間 + VRAM + manifest 自動生成)
- ループ誤差:
  - 公式指標 (B1): CloudCompare で生成 PCD の始終点同一壁面 3 点平均、目標 ≤ 0.5 m
  - 補完指標: `scripts/m5r3_loop_error.py` で TUM trajectory の始終点距離 (SLAM 内部の pose graph 閉鎖状態を見る)
- 操作性: Iridescence (GLIM) / RViz (FAST-LIO SAM) を観察し、manual relocalization の要否、keyframe 発行密度、ループクロージャ発火タイミングを記録
- GTSAM 競合: GLIM 用 4.3a0 (`/usr/local/lib`) と FAST-LIO SAM 用 4.1.1 (`/usr/lib`) の共存状態を `gtsam_env.log` に snapshot

詳細手順は [`../m5r3-comparison-protocol.md`](../m5r3-comparison-protocol.md) を参照。

### Phase 構造

本 ADR は 2 Phase で完成する:

1. **Phase A (本 commit)**: skeleton 起案。計測ラッパ + プロトコル文書 + ADR 構造を整える。Decision 節は placeholder
2. **Phase B (別 commit、ユーザー作業後)**: 実 bag 取得 → 両 SLAM 実行 → 数値 + 操作性メモを Alternatives 表と Consequences 節に転記 → Decision 節を埋めて PR を ready 化。ユーザー承認後 Status を `proposed → accepted` に書き換える

## 決定

**採用 SLAM: GLIM** (`koide3/glim` + `koide3/glim_ros2`、MIT)。

**Commit SHA / Tag pin** (Phase B run 時、`docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml` より):
- 本リポ git_commit: `48b746a` (bag rewrite script を含む M5R-3 ブランチ最終状態)
- 上流 (`install_glim.sh` で source-build 済): M5R-1 (#45) で固定したバージョン。CUDA 12.4 + cuDNN 8 + GTSAM 4.3a0 (UNSTABLE) 構成
- 設定: per-run config copy 経由で `config_sensors.json` の `T_lidar_imu` と `ring_field` を M4R-2 実測 extrinsic / VLP-16 値に上書き (`scripts/m5r3_run_glim.sh` 内に内包)

**判断根拠サマリ**:

1. **走った/走らなかった** (一番重い軸): GLIM は Velodyne + PCMK-G3X (MPU-9250) bag に対して `traj_lidar.txt` (1954 サンプル)、`graph.bin`、17 個の submap を出力してクリーン終了。FAST-LIO SAM は上流 ROS2 port に Velodyne mapping launch が無く、本リポで自作 launch + config を用意した上でも初回 frame 直後に silent crash (DNF)。詳細は本ファイルの「採用しなかった案」表
2. **ライセンス**: GLIM = MIT、permissive。運用スタック (将来配布候補) への組み込み制約なし。FAST-LIO SAM = LICENSE 不在 + 派生元 GPL-2.0 で copyleft 伝播懸念。親方針 §3.4 と整合
3. **ループ誤差** (補完指標、B1 wall-3-point は走った GLIM 側のみ取得可): GLIM は 52.640 m loop で end-to-start drift 0.838 m (~1.6%)。FAST-LIO SAM は trajectory 出力ゼロで比較不能
4. **LiDAR class fit** (本デッキ固有の判断): VLP-16 は中等性能 LiDAR (16 line、屋外特徴 poverty に弱い)。FAST-LIO 系の優位性 (HKU-MaRS 開発元の hardware) は OS-128 / Livox MID-360 など高密度 LiDAR で顕著。VLP-16 で GLIM が clean に動いた以上、本実装の設備規模ではどちらでも結論はほぼ同等。**FAST-LIO SAM を動かすための追加作業 (ROOT_DIR バグ追跡、preprocess crash 原因特定、upstream 改修) の工数は M5-R の他フェーズ (ERASOR、占有格子、パイプライン統合) に振り向ける方が ROI が高い**
5. **GTSAM 競合**: gtsam_env.log で 4.3a0 (`/usr/local`) と 4.1.1 (`/usr/lib`) が ldconfig 上で共存していることを確認、衝突は run 時に顕在化しなかった。GLIM 採用後は 4.3a0 単独で運用するので競合リスク自体が消える

## 採用しなかった案

### FAST-LIO SAM (採用見送り)

| 軸 | GLIM (採用) | FAST-LIO SAM (見送り) |
|---|---|---|
| 走行時間 (wall clock s) | 575 (bag 199 s に対し ~2.9x) | 589 (うち SLAM 実処理は ~0.14 s で crash、残りは死体プロセス待ち) |
| ピーク VRAM (MiB) | 545 | 15 (baseline、SLAM が GPU を握ったことなし) |
| ピーク RSS (KiB) | n/a (GLIM wrapper では未計測) | 0 (RSS poller が pid を捕捉できる前に死んだ) |
| trajectory 内部誤差 (m) | **0.838** (52.640 m loop で 1.6%、`m5r3_loop_error.py` 算出) | dnf (trajectory 出力ゼロ) |
| B1 公式誤差 (壁面 3 点平均、m) | TBD (CloudCompare 未実施、PCD は GLIM 側に submap 17 個として存在。GLIM のみでの値なので比較相手がいない) | dnf (PCD 出力ゼロ) |
| ループクロージャの発火タイミング | run.log に明示エントリなし。屋外直線往復という幾何 (50 m 直進 + 180° turn) では closure 検出条件を満たしにくいのは想定内 | dnf |
| keyframe 発行密度 (枚 / m) | 1954 サンプル / 52.640 m = 37 sample/m。submap 17 個 | dnf |
| manual relocalization 要否 | 不要 (M4R-2 extrinsic 直焼き + bag rewrite 経由で IMU 符号正規化、起点静止 5s 確保で grav_align が収束) | n/a |
| GTSAM 解決状況 | 4.3a0 単独 (`/usr/local/lib`、GLIM source build に同梱) | gtsam_env.log で 4.3a0 (`/usr/local`) + 4.1.1 (`/usr/lib`) 共存、ldconfig 上は両方見える状態。今回 run 時には衝突は顕在化しなかったが、潜在リスクは継続 |
| ライセンス | MIT (permissive) | 上流 LICENSE 不在 + 派生元 FAST-LIO は GPL-2.0。`package.xml` のみ "BSD" 自己申告で実体不一致 |
| build 成否 | OK (M5R-1 #45 で source build 完了) | パッケージ build は OK、ただし mapping launch が Velodyne 用は ROS2 port に存在せず (`airy`/`l2`/`mid360` のみ ship)。本リポで `scripts/m5r3_mapping_velodyne_for_fastlio_sam.launch.py` + `scripts/m5r3_fastlio_sam_velodyne_config.yaml` を新規作成して initialization までは通したが、preprocess `[WARN] No point, skip this scan!` 直後 silent crash (DNF) |

### 補足ノート

- **FAST-LIO SAM 側で何が決定打になったか**: 上流 ROS2 port が Velodyne を一級サポートしていない点が一番大きい。`config/odom/velodyne.yaml` (odometry-only) は存在するが、`config/mapping/` 下に velodyne yaml が無く、`launch_ROS2/mapping/` にも Velodyne 用 launch が無い。本 ADR スコープで自作の最小 launch + config (commit 4af5ffa) を用意したが、起動直後に upstream 側の path-construction 警告 (`~~~~<repo>/src/third_party/FAST_LIO_SAM/ doesn't exist` — 実際には存在するので realpath / trailing-slash 由来のバグ) が出て、初回 frame の preprocess で silent crash。crash 原因の特定には upstream 改修が必要で、M5R-3 スコープ外。詳細は `docs/m5r-bench-data/2026-06-21-loop-outdoor/fastlio-sam-out/manifest.yaml` の notes 節
- **対称性に影響した事実**: GLIM 側は per-run config copy で `T_lidar_imu` (M4R-2 実測値の SE3 inverse) と `ring_field=ring` (VLP-16) を焼き込んだ。FAST-LIO SAM 側は同じ extrinsic を yaml に直書きしたが、そもそも crash で extrinsic が効くフェーズに到達せず。両 SLAM ともに `bag-imu-fixed/` (`scripts/m5r3_fix_imu_bag.py` で PCMK-G3X firmware の accel 重力ベクトル符号を REP-145 specific force に補正済) を入力。GLIM は補正必須 (FAST-LIO 系は自前で gravity 符号を吸収するので補正なしでも本来動く)
- **li_slam_ros2 を評価対象外とした理由**: 親方針 §3.3 で「比較・つなぎ用」と明記。GLIM vs FAST-LIO SAM が代表選定であり、本 ADR は両者比較で結論を出す枠組み。FAST-LIO SAM 不採用は GLIM 採用と一意に結びつくため (Velodyne 対応 + permissive + 動作確認済 という条件で残るのは GLIM)、li_slam_ros2 の再評価は不要

## 結果

### ライセンス棚卸し (B5 達成)

採用 SLAM = GLIM について本リポへの組み込み形態を明示する:

- **GLIM**: MIT、permissive。運用スタックへの link 制約なし。ただし M5-R は親方針 §3.1 が定める「オフラインのマップ作成ツール」フェーズで、ランタイム localizer は M6-R の scan-to-map localizer (別 ADR) が担当する。GLIM 自体を運用スタックに組み込み直す判断は本 ADR では行わず、M6-R の評価結果次第とする
- **GLIM が依存する GTSAM 4.3a0 (BSD-3-Clause、UNSTABLE 含む)**: `install_glim.sh` 経由で `/usr/local/lib` 配下に source-build 済。permissive で配布上の制約なし
- **本リポ内に残る FAST-LIO SAM 評価関連物**: `src/third_party/FAST_LIO_SAM/` は `.gitignore` 済 (clone-on-demand)、本リポへの再配布なし。`scripts/m5r3_run_fastlio_sam.sh` + `scripts/m5r3_mapping_velodyne_for_fastlio_sam.launch.py` + `scripts/m5r3_fastlio_sam_velodyne_config.yaml` は M5R-3 評価のための本リポ独自コード (BSD-3-Clause) で、上流コードの再配布ではない。将来 FAST-LIO SAM 上流が permissive LICENSE を追加して状況が変わった場合は別 ADR で再評価する

### CPU / GPU / メモリ要件 (実測値)

母艦: Alienware x15 R2 (i9-12900H 32 GiB RAM、RTX 3080 Laptop GPU 16 GB VRAM)。GLIM Phase B run (`docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml`):

- 走行時間: 575 s wall clock (bag 199 s に対し ~2.9x)
- ピーク VRAM: 545 MiB
- ピーク RSS: 未計測 (GLIM wrapper では計測項目なし。次回 wrapper 改修で追加候補)
- 平均 playback speed: 0.35x realtime (起動初期は 1.9-6.6x で速いが、submap 増加につれてリアルタイム以下に減速。屋外の中等密度 LiDAR + 50 m 級ループでこの値なら、車載機 (M9 で評価) では 1x realtime 必須要件と擦り合わせる必要あり)

車載機への移行可否は本 ADR では判断せず、M9 (車載分離) で再評価。母艦運用 (GPU 16 GB + 32 GiB RAM) では余裕あり。

### 後続フェーズ (M6-R) への影響

- M6-R の scan-to-map localizer は GLIM が出した静的 PCD を `docs/maps/<site>/static.pcd` 規約 (ADR-0005) に従って入力前提とする
- PCD フォーマット: GLIM は `dump_path` 指定で各 submap dir (`000000/` 〜 `000017/`) に submap 点群と pose を吐く。**単一の global static PCD を作るには `glim_offline` 等の上流ツールでマージが必要** (M5R-4 ERASOR の入力前段で実施)
- 座標系: GLIM の `auto-detected IMU frame ID: imu_link` + `auto-detected LiDAR frame ID: velodyne` の通り、TF 自動検出が動いた。出力 trajectory は `traj_lidar.txt` (LiDAR frame) と `traj_imu.txt` (IMU frame) の両方が ship される
- coordinate frame の整合性: M4-R bringup `/tf_static` の `base_link → velodyne` extrinsic は bag に乗っており、GLIM はこれを使って `base_link` frame の TF を publish 可能 (今回の評価では IMU frame を base に使った)

### IMU 符号規約の波及 (本 ADR の発見、別 Issue 候補)

Phase B 計測中に判明: PCMK-G3X (MPU-9250 + LPC1343F USB firmware) は `linear_acceleration` を REP-145 specific force ではなく gravity-vector で出力する (実測 `linear_acceleration.z = -9.71` at rest)。GLIM は明示補正必須、FAST-LIO 系は自己吸収 (`IMU_Processing.hpp:196` の `init_state.grav = S2(-mean_acc / |mean_acc| * G)`)。本 ADR では M5R-3 評価向け最小経路として `scripts/m5r3_fix_imu_bag.py` で bag rewrite して回避したが、永続対策 (sensor bringup 層に republisher を追加して全下流で REP-145 準拠 IMU を流す) は本 ADR スコープ外で別 Issue を起案する。EKF (M4R-3) への影響可能性も同 Issue で精査する。

### 後続作業

- **M5R-4 (#49) ERASOR**: GLIM の submap 出力 (PCD + per-frame poses) を入力に動的物体除去。本 ADR accepted を受けて着手可能
- **M5R-6 (#50) 占有格子変換**: GLIM → ERASOR 後 PCD を 2D 占有格子に変換、`docs/maps/<site>/occupancy.{pgm,yaml}` に格納
- **M5R-7 (#51) パイプライン統合**: bag → GLIM → ERASOR → 占有格子 → `docs/maps/<site>/` の E2E 文書化
- **IMU 符号永続対策 Issue (本 ADR §「IMU 符号規約の波及」発の派生)**: `whill_sensors_bringup/` に `/imu/data_raw` → `/imu/data_corrected` の REP-145 化 republisher を追加。再録 bag の符号統一、EKF 設定の波及確認

### 本番マップ確定: `docs/maps/campus/` (2026-07-10)

M5-R 出力の最終確定版。取得経路の 25 倍近いスケール (1310 m ループ) で
GLIM を回して品質を実測した。

**取得と定量指標**:

| 指標 | 値 | 出所 |
|---|---:|---|
| bag | 2162 s / 12.8 GiB (2026-07-10-campus-outer-final) | `bag_info` |
| loop length | 1310.098 m | `m5r3_loop_error.py` |
| **end-to-start (trajectory)** | **1.317 m (0.10%)** | 同 |
| per-axis (dx / dy / **dz**) | +0.107 / -0.161 / **+1.303** | 同 |
| yaw drift | -0.16° | 同 |
| **B1 数値代替** (地面 z 層 gap) | **1.394 m** | `m5r3_b1_numeric.py`、traj dz と 7.0% 差で独立一致 |
| GLIM 実行時間 | 691.8 s (bag の 32%) | `manifest.yaml` |
| Peak VRAM (GLIM) | 3297 MiB | 同 |
| starved anchor (占有格子) | **0.0%** | `investigate_thin_corridor.py` (relative z-slice 有効時) |
| 目視判定 (3 視点) | **PASS** | offline_viewer |

**Phase B (2026-06-21) からのスケール変化と学び**:
- ループ長: 52.6 m → 1310 m (25x)
- loop_error: 0.838 m / 1.6% → 1.317 m / **0.10%** (相対誤差は 16x 改善、絶対値は 1.6x 増)
- スケーリング: 47 分 / 1310 m の中規模ループでは per-axis dz が支配的
  (dx/dy はほぼゼロに近く、Z 方向の drift 主体)

**T_lidar_imu 校正の反復**:
- Phase B (06-21) は M4R-2 の noetic 由来 extrinsic を使用
- 07-08 audit で LiDAR がほぼ水平 (pitch -0.5°)、IMU が tilt という
  シナリオが判明 → `GLIM_TLI_FROM_AUDIT=1` env-gate 導入 (commit `494ea77`)
- 07-10 pre-run で IMU 再測定 → RPY(-4.09°, -7.61°, 0°) → audit quat
  を更新 (commit `39bf794`)
- 詳細は `docs/ja/imu-coordinate-audit.md` §8

**残る既知事項 (M6-R 引き継ぎ)**:
- **map tilt 1.81°** (traj z 平面フィット結果、残差 RMS 1.32 m)。
  GLIM の world z axis が真の gravity と 1.81° ズレている。localizer
  の gravity-aware factor 設計時に影響する可能性
- **IMU better ratios 低い** (trans=0.03 / vel=0.07)。bag 47 分の
  大半で LiDAR 主導、IMU 予測寄与が薄い。localization で IMU 予測に
  頼る設計を組む場合はここが弱点になる
- **bias_acc 未収束**。マップ品質には影響しないが、上と同じ理由で引き継ぎ

**成果物**:
- `docs/maps/campus/README.md` — 定量指標 + 特記事項 + 再生成手順
- `docs/maps/campus/metadata.yaml` — ADR-0005 準拠のメタデータ
- `docs/maps/campus/occupancy.{pgm,yaml}` — Nav2 map_server 入力
- `docs/maps/campus/traj_lidar.txt` — 占有格子生成用トラジェクトリ
- `docs/maps/campus/static.pcd` — DUFOMap 出力、gitignore

**M5-R 完了判定**: 上記が揃った時点で M5-R (親方針 §4 マイルストーン表)
は Done。次は M6-R (運用 localizer + Nav2 再統合) に着手可能。

## 関連

- 親方針: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §3.3 (採用候補)、§3.4 (ライセンス方針)、§7 (本 ADR の起案要請)
- M5-R 実行計画: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md) §M5R-3 (本 Issue)、§6 (受け入れ基準 B1〜B5)
- 比較プロトコル: [`../m5r3-comparison-protocol.md`](../m5r3-comparison-protocol.md) — Phase B 実行手順書
- 前置 ADR: [`0005-maps-spec.md`](0005-maps-spec.md) — 採用 SLAM の出力先 `docs/maps/<site>/` の規約
- 前置文書: [`../m5r-glim-setup.md`](../m5r-glim-setup.md) (GLIM source build)、[`../m5r-fastlio-sam-eval.md`](../m5r-fastlio-sam-eval.md) (FAST-LIO SAM clone-on-demand)
- スクリプト: [`../../../scripts/m5r3_run_glim.sh`](../../../scripts/m5r3_run_glim.sh)、[`../../../scripts/m5r3_run_fastlio_sam.sh`](../../../scripts/m5r3_run_fastlio_sam.sh)、[`../../../scripts/m5r3_loop_error.py`](../../../scripts/m5r3_loop_error.py)
- 関連 Issue: #48 (本 Issue、M5R-3)、#45 (M5R-1 GLIM)、#46 (M5R-2 FAST-LIO SAM)、#47 (M5R-5 maps 規約)、#49 (M5R-4 ERASOR)、#50 (M5R-6 占有格子)、#51 (M5R-7 統合)

# M5-R 実行計画: マップ作成パイプライン

Language: [日本語](2026-06-21-m5r-execution.md) | [English](../../en/plans/2026-06-21-m5r-execution.md)

- 日付: 2026-06-21
- 状態: proposed (ユーザー承認待ち)
- 親方針: [`docs/ja/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md)
  §3.1 (二相分離), §3.3 (採用候補), §3.4 (ライセンス), §4 (M5-R), §6 (受け入れ基準),
  §7 (ADR 候補), §9 (開発機材)
- 前段: [`docs/ja/plans/2026-06-13-m4r-execution.md`](2026-06-13-m4r-execution.md)
  (M4-R 完了 = main `1562361`)
- 想定配置: `docs/ja/plans/2026-06-21-m5r-execution.md`
- 読者: 本フェーズに着手する `research-analyst` / `ros2-implementer` /
  `code-reviewer` および実機 bag 取得・成果物検証を実施するユーザー

## 0. ユーザー要件の理解

親方針 §4 の M5-R は「マップパイプライン: GLIM (または FAST-LIO SAM) 導入、
ERASOR で動的除去、`docs/maps/<site>/` への成果物規約 (pcd + pgm + yaml +
取得メタデータ)」と定められている。本書はこれを Issue 単位の実行可能粒度に
分解し、SLAM 候補 (GLIM vs FAST-LIO SAM) の決定経路、動的除去パイプライン、
成果物規約、検証手順を確定させる。

M4-R との性格の違い: M4-R は「実装フェーズ (4 Issue 直列)」だったが、M5-R は
「研究選定フェーズ + パイプライン整備」になる。GLIM の実 bag 評価結果次第で
ADR が分岐するため、Issue 構造に「決定ポイント」を明示的に設ける。

## 1. 背景

### 1.1 解消する既知課題

親方針 §2 診断のうち本フェーズが解消する範囲:

| ID | 内容 | M5-R での解消経路 |
|----|------|----------------|
| P5 (地図品質側) | ゴースト障害物による `use_collision_detection: false` 連鎖、QoS 不一致による obstacle layer 不在の根本原因は「動的物体を含んだまま地図化した点群」。ERASOR で動的除去 + ループクロージャ付き SLAM で静的地図を作る。実 obstacle layer 復活・collision detection 復帰自体は M6-R | 静的 PCD + 占有格子の品質を担保 |

P1〜P4 / P2 残りは M4-R / M6-R 担当で本フェーズ非対象。

### 1.2 M4-R 成果物の前提

M5-R は M4-R で確定した次を入力とする (`docs/ja/plans/2026-06-13-m4r-execution.md`
§10 引き渡し):

- TF: `odom -> base_link -> {imu_link, velodyne, camera_link}` 一本鎖
- 入力 topic: `/whill/odom` (~2.5 Hz)、`/imu/data_raw` (100 Hz)、`/velodyne_points` (10 Hz)
- launch: `whill_localization/launch/odom_bringup_launch.py` (sensors + driver + EKF)

bag 取得時にはこの launch で TF を立てた状態で `/velodyne_points` `/imu/data_raw`
`/tf_static` を録る。これにより**生成された PCD 地図と運用時 TF が原理的に
整合**する (M5-R 中で再キャリブする必要が出ない)。

### 1.3 M5-R で「触らないもの」(凍結維持)

- **FAST-LIO ランタイム強化**: 親方針 §5 禁止 #2。ただし「マップ作成品質の改善
  目的のパラメータ再調整」は §5 で許可されているため、FAST-LIO SAM (FAST-LIO2
  をフロントエンドに使う) は本フェーズの候補に残る
- **scan-to-map localizer 系**: lidar_localization_ros2 / hdl_localization /
  mcl_3dl は M6-R の担当。本フェーズでは入力 (静的 PCD) を整備するだけ
- **Nav2 obstacle layer 復活**: M6-R 担当
- **`navsat_transform_*` WIP**: M4-R §3.A の判断を維持し本フェーズでは触らない

## 2. 作業範囲

### 2.1 扱うもの

1. **NVIDIA ドライバ / CUDA 12.4 / cuDNN 8 のセットアップ状態確認**: Issue #23
   (closed) で `scripts/install_cuda.sh` と `docs/ja/m5r-cuda-setup.md` が
   完成済、動作確認日 2026-06-13 で母艦 (Alienware x15 R2、driver 595、
   RTX 3080 Laptop GPU) でセットアップ済の記録あり。本フェーズでは
   「GLIM ビルド前に再現性確認」のみを行い、再セットアップは不要
2. **GLIM の母艦インストールと最小疎通**: PPA または Docker 経由で
   `glim_ros2` を入れ、サンプル bag (本リポでは M3 期取得の `m3_chair_motion_*`)
   を流して trajectory が出ることを確認する
3. **FAST-LIO SAM の整理**: 現状 `src/third_party/FAST_LIO_SAM/` が
   `whill_lab.repos` に記載されないまま手動 clone された状態で存在
   (`src/third_party/FAST_LIO_SAM/CMakeLists.txt`, `README.md` 確認済)。
   ライセンス・依存・vcs 取り扱いを整理する (後述 §3 参照)
4. **GLIM vs FAST-LIO SAM の実 bag 比較**: 同一の M4-R-bringup bag を入力に、
   GLIM (GPU) と FAST-LIO SAM の両方を回して生成 PCD・ループクロージャ誤差・
   操作性を比較。結果を ADR-0003 (本計画で起案) で確定する
5. **ERASOR 系動的除去パイプライン整備**: ループクロージャ付き SLAM の出力
   (PCD + per-frame poses) を入力に、ERASOR を回して静的点群を出力する
   オフライン処理スクリプトを作る。動的物体ありの bag (run3 相当) で
   「尾を引く残像」が消えることを目視確認
6. **`docs/maps/<site>/` 成果物規約の確立**: 親方針 §6 受け入れ基準 (3) の
   「pcd + pgm + yaml + 取得日 / 経路 / 天候のメタデータ」を README 雛形と
   ディレクトリ規約で固める。既存 `docs/m5-maps/` (旧 M5-b 残骸) との整理方針も
   含む
7. **占有格子変換**: 静的 PCD から 2D 占有格子 (pgm + yaml) を生成する
   スクリプト。旧 M5-b の `pcd_to_occupancy_grid.py` 系統を参考にしつつ、
   `docs/maps/<site>/` 規約に出力する
8. **M5-R 完了文書**: 採用 SLAM、bag 取得手順、後処理パイプライン全体、
   M6-R への引き渡し成果物 (静的 PCD + 占有格子) を README にまとめる

### 2.2 扱わないもの (明示)

- **scan-to-map localizer の選定・実装**: M6-R 担当
- **Nav2 への costmap 流し込み**: M6-R 担当
- **realtime / 車載でのマップ作成**: 親方針 §3.1 で「マップ作成 = オフライン・
  母艦」と定めた以上、車載でこれを回す検討は M5-R では行わない (将来 M9+
  で車載分離する判断が出たら再評価)
- **キャンパス本番経路の bag 収録**: 実機検証は研究室内ループ走行 bag で
  パイプライン完成性を担保。本番キャンパス収録は M5-R 完了後、M6-R 着手と
  同時または直前に行う運用判断
- **新規 IMU / LiDAR キャリブレーション**: M3 / M4-R で確定した extrinsic を
  使う。地図品質要求でズレが顕在化したら別 Issue
- **動的除去アルゴリズムの自前実装**: ERASOR 上流の OSS をそのまま使う

## 3. 既存 WIP コードと残骸の扱い

### 3.1 `src/third_party/FAST_LIO_SAM/` の扱い

事実調査の結果:

- `src/third_party/` は `.gitignore` で除外され、vcs import 経由で再現する規約
  (`whill_lab.repos`)
- `whill_lab.repos` に `FAST_LIO_SAM` のエントリは**ない**
- にもかかわらず `src/third_party/FAST_LIO_SAM/CMakeLists.txt` および
  `README.md` が存在する。誰かが手動で `git clone` して放置した残骸
- 上流は `https://github.com/RightTr/FAST-LIO-SAM.git`、ROS2 humble 対応を
  README で明記、ライセンスは未確認 (Issue M5R-2 で確認)

判断: Issue M5R-2 (FAST-LIO SAM 候補化準備) の冒頭で次のいずれかを実行する:

- (a) ライセンス・依存が permissive 寄りで採用候補に値するなら、`whill_lab.repos`
  に正式エントリを追加し、vcs import 再現性に組み込む
- (b) GPL 系で運用スタックに混入し得るなら本物理ディレクトリを削除し、
  「採用検討時に再度 clone する」運用に倒す。親方針 §3.4 の「GPL 系は
  オフラインのマップ作成ツールとしての分離プロセス利用に限定」が適用される
  ため、マップ作成専用としては許容可能だが、運用パッケージへの link 禁止を
  明示する

### 3.2 `docs/m5-maps/` の旧 M5-b 残骸

事実: `docs/m5-maps/` に `lab.pcd`, `lab.pgm`, `lab.yaml`,
`global_2026-06-04_10min.pcd` の 4 ファイルが存在 (`.gitignore` で `*.pcd` は
除外、`lab.yaml` / `lab.pgm` は tracked)。これらは旧 M5-b フェーズ
(凍結対象 = 親方針 §4 で再定義された M5-R に置換) の試作成果物。

判断: 親方針 §6 受け入れ基準 (3) の `docs/maps/<site>/` 規約に統合する
形で **Issue M5R-5 (成果物規約) で整理**する。具体的には:

- 既存 `docs/m5-maps/` を `docs/maps/lab-legacy-m5b/` にリネームして
  「凍結前の試作品」と明示する (M5-b と M5-R は別物の宣言)
- または、品質基準を満たさないと判断したら削除して履歴のみ legacy-findings に
  残す

最終判断は Issue M5R-5 着手時の現物確認による。

### 3.3 `velodyne_whill.yaml` の `pcd_save_en` / `map_file_path`

事実: `src/whill_localization/config/velodyne_whill.yaml` に
`map_file_path: /home/systemlab/whill_lab0_ros2/docs/m5-maps/lab.pcd` と
`pcd_save_en: true` がコメント「M5-b」付きで残っている。

判断: 本 yaml は M5-R 期間中も FAST-LIO を「マップ作成ツール (オフライン replay)」
として利用するため即時削除はしない。ただし `map_file_path` のハードコードパス
は M5-R 規約の `docs/maps/<site>/` 配下に向け直す。Issue M5R-7 (パイプライン
統合) で対応。

## 4. 前提条件

- M4-R Issue M4R-1〜M4R-4 全件 merged。`/odometry/filtered`、`odom -> base_link`、
  `whill_localization/launch/odom_bringup_launch.py` が機能している
- 母艦 (Alienware x15 R2) で `nvidia-smi` が driver 595 を返し、
  `/usr/local/cuda-12.4/bin/nvcc --version` が CUDA 12.4 を返す
  (Issue #23 で確認済、`docs/ja/m5r-cuda-setup.md` の動作確認手順を再現するだけ)
- WHILL Model CR2 / Velodyne VLP-16 / IMU の実機が利用可能。bag 取得は
  ユーザーが実走行 (手押し or ジョイスティック) する (CLAUDE.md 規約)
- ループ走行 bag (始終点が同一の経路) を最低 1 本、動的物体ありの bag
  (歩行者横断) を最低 1 本、それぞれ研究室内で取得できる
- 親方針 §3.3 の選定を **覆す判断は ADR-0003 として記録** (例: GLIM ではなく
  FAST-LIO SAM を第一候補に据える、ERASOR ではなく Removert を選ぶ)

## 5. 受け入れ基準 (親方針 §6 + 本計画の補強)

親方針 §6 の M5-R 3 項目を観測可能なコマンドと期待値に展開:

- [ ] **B1: ループクロージャの目視整合**
  - コマンド: 同一始終点のループ走行 bag を採用 SLAM (GLIM または FAST-LIO SAM)
    に流し、生成 PCD を CloudCompare / RViz で表示
  - 期待: 始点と終点の構造 (壁・コーナーなど特徴的な物体) が**数十 cm 以内**で
    重なる。判定は目視 + 距離計測ツール (CloudCompare の Point picking)
  - 合格閾値の提案 (本計画で確定): 始終点の同一壁面 3 点平均で **≤ 0.5 m**
    (= ループ長 50 m 想定で 1%。M4 期 FAST-LIO 単独の 18% から桁違い改善が
    ループクロージャ + GLIM 大域最適化に期待される)
- [ ] **B2: 動的物体の除去**
  - コマンド: 歩行者が横切った bag を採用 SLAM に流して PCD を取得 →
    ERASOR を回して静的 PCD を出力 → 両者を比較
  - 期待: 動的除去前は歩行者の軌跡が「尾を引く」点群として残る、
    除去後はそれが消える。占有格子に変換しても同じ
  - 検証スクリプト: 除去前後の PCD を重ねて差分を可視化する Python スクリプト
    (`scripts/m5r_erasor_diff.py` を本計画で新設、Issue M5R-4)
- [ ] **B3: `docs/maps/<site>/` 成果物の完全性**
  - コマンド: `ls docs/maps/<site>/` で次が全て存在
    - `static.pcd` (静的 PCD、ERASOR 後)
    - `occupancy.pgm` (2D 占有格子)
    - `occupancy.yaml` (Nav2 map_server 互換のメタデータ)
    - `metadata.yaml` (取得日 / 経路概要 / 天候 / 採用 SLAM / ERASOR
      パラメータ / 元 bag 名 / コミット SHA)
  - 期待: 上記 4 ファイルが揃い、`metadata.yaml` が必須項目を全て持つ。
    Issue M5R-5 で雛形を確定
- [ ] **B4: ADR-0003 (SLAM 採用) が `accepted`**
  - 本計画で起案する `docs/decisions/0003-mapping-slam-choice.md` が
    実 bag 比較結果 (Issue M5R-3) を根拠に accepted で merged

補強基準 (本計画で追加。code-reviewer が確認):

- [ ] **B5: ライセンス棚卸し記載**
  - 採用した SLAM / ERASOR それぞれについて、ライセンス・運用スタックへの
    link 有無を `docs/decisions/0003-mapping-slam-choice.md` の Consequences 節で
    明示。親方針 §3.4 の「GPL 系は分離プロセス利用に限定」が遵守されている
    ことを文章で確認
- [ ] **B6: M6-R 引き渡し条件の明文化**
  - `docs/ja/m5r-pipeline.md` (新設、Issue M5R-7) に「M6-R の scan-to-map
    localizer はこの規約の `docs/maps/<site>/static.pcd` を入力前提とする」と
    明記。占有格子は Nav2 obstacle layer 復活 (M6-R) の入力前提

## 6. Issue 分割案

以下 7 件に分割する。M4-R より多いのは「選定フェーズ + 4 つの後処理ステップ」
が直列依存のため。各 Issue は単体で完結可能な粒度。

### Issue M5R-1: CUDA / GLIM 母艦セットアップ動作確認

- **目的**: Issue #23 で完了済のはずの CUDA 12.4 + cuDNN 8 環境が現時点でも
  機能していることを確認した上で、GLIM (`glim_ros2`) を母艦にインストールし、
  サンプル bag に対して最小疎通を取る
- **受け入れ基準**:
  - [ ] `nvidia-smi` が driver 595 系を報告
  - [ ] `/usr/local/cuda-12.4/bin/nvcc --version` が CUDA 12.4 を報告
  - [ ] `docs/ja/m5r-cuda-setup.md` §2.4 の `vectorAdd` サンプルが `Result = PASS`
  - [ ] GLIM が apt PPA または Docker でインストールされ、`ros2 run glim_ros
    glim_rosbag <bag>` 相当のコマンドが起動・trajectory 出力
  - [ ] 上記手順を `docs/ja/m5r-cuda-setup.md` の続編または新規
    `docs/ja/m5r-glim-setup.md` として追記
- **スコープ外**: 採用判定 (Issue M5R-3 で実 bag 評価する)、FAST-LIO SAM 側の
  インストール (Issue M5R-2)
- **前提仮定**: CUDA 12.4 セットアップは Issue #23 で done (`docs/ja/m5r-cuda-setup.md`
  の動作確認日 2026-06-13)。本 Issue は再現性確認のみ
- **担当 agent**: `ros2-implementer` (実セットアップ手順は `research-analyst` の
  既存調査結果に基づくため新規 web 調査は不要)
- **想定ブランチ**: `m5r/1-glim-setup`

### Issue M5R-2: FAST-LIO SAM 候補化準備 (整理 + ライセンス確認)

- **目的**: 現状 `src/third_party/FAST_LIO_SAM/` に手動 clone された状態の
  FAST-LIO SAM を、(a) `whill_lab.repos` 経由で正式管理にするか、(b) 物理削除して
  採用検討時に再度 clone する運用にするかを確定し、ライセンス・依存関係を整理
- **受け入れ基準**:
  - [ ] 上流 (`https://github.com/RightTr/FAST-LIO-SAM.git`) の LICENSE 確認 →
    permissive ならば (a)、GPL 系ならば (b)
  - [ ] (a) の場合: `whill_lab.repos` に entry 追加、`vcs import` でクリーン
    再現可能。本リポでは vcs import + .gitignore による自動除外を維持
  - [ ] (b) の場合: 現存ディレクトリを削除し、`docs/ja/m5r-fastlio-sam-eval.md`
    に「採用検討時の clone 手順」を記載
  - [ ] gtsam 4.1 (`libgtsam-dev`) のインストール手順が `docs/ja/m5r-fastlio-sam-eval.md`
    に追記される (FAST-LIO SAM README の prereq)
  - [ ] (a)/(b) どちらでも、M5R-3 着手時に評価担当者が `colcon build --packages-up-to fast_lio_sam` を実行して成功することを確認する (本 Issue では build を実行しない)
- **スコープ外**: 実 bag 比較 (M5R-3)、運用パッケージへの link
- **担当 agent**: `research-analyst` (ライセンス事実確認) → `ros2-implementer`
  (`whill_lab.repos` 編集 or 削除実行)
- **想定ブランチ**: `m5r/2-fastlio-sam-prep`

### Issue M5R-3: GLIM vs FAST-LIO SAM 実 bag 比較 + ADR-0003 起案

- **目的**: 同一の bag (ユーザーが M4-R-bringup 状態で取得した室内ループ
  走行 bag) を入力に、GLIM (GPU) と FAST-LIO SAM の両方を回して、生成 PCD・
  ループクロージャ誤差・操作性 (manual relocalization / keyframe export 等の
  有無) を比較する。結果を `docs/decisions/0003-mapping-slam-choice.md` (新規) で
  確定する
- **受け入れ基準**:
  - [ ] ユーザーがループ走行 bag を 1 本以上取得し
    `docs/m5r-bench-data/<YYYY-MM-DD>-loop/bag/` に配置 (規約は M5R-6 で確定、
    本 Issue 着手前に最低限の取得手順を共有)
  - [ ] GLIM での処理結果: 生成 PCD、ループクロージャ誤差 (B1 基準で評価)、
    所要時間、所要 VRAM が記録
  - [ ] FAST-LIO SAM での処理結果: 同上
  - [ ] `docs/decisions/0003-mapping-slam-choice.md` (proposed) が、上記比較を
    Context 節、選定理由を Decision 節、もう一方を Alternatives 節に整理した
    内容で起案される
  - [ ] ADR の Status は本 Issue では `proposed`。ユーザー承認後に `accepted`
    化する流れ (ADR-0001 の運用に従う)
- **スコープ外**: 動的除去 (M5R-4)、scan-to-map localizer 評価 (M6-R)
- **担当 agent**: `research-analyst` (bag 評価結果の構造化) → ADR 起案は
  `pm-orchestrator` が ADR テンプレートに沿って書く
- **想定ブランチ**: `m5r/3-slam-comparison`

### Issue M5R-4: ERASOR 動的除去パイプライン

- **目的**: 採用 SLAM (M5R-3 で確定) の出力 (PCD + per-frame poses) を入力に、
  ERASOR を回して静的 PCD を出力するオフライン処理スクリプトを作る。動的物体
  ありの bag (歩行者横断、run3 相当) で「尾を引く残像」が消えることを目視
  確認する
- **受け入れ基準**:
  - [ ] ERASOR (`https://github.com/LimHyungTae/ERASOR`、Apache-2.0) を母艦に
    インストール。`scripts/m5r_run_erasor.sh` (新規) で SLAM 出力を入力に
    取り、静的 PCD を出力する流れを冪等化
  - [ ] 動的物体ありの bag (1 本以上) で除去前後の PCD を `scripts/m5r_erasor_diff.py`
    (新規) で重ね、歩行者軌跡が消えていることを目視確認
  - [ ] ERASOR パラメータ (voxel size、PR / RR の閾値等) が
    `docs/ja/m5r-pipeline.md` (Issue M5R-7 で作成) に記録
  - [ ] B2 (動的物体除去) の検証が成立
- **スコープ外**: 占有格子変換 (M5R-6)
- **担当 agent**: `research-analyst` (ERASOR の代替検討 Removert を含めた最終判定)
  → `ros2-implementer` (スクリプト整備)
- **想定ブランチ**: `m5r/4-erasor-dynamic-removal`
- **不確実性**: ERASOR が ROS 2 humble 環境でビルド可能かは要確認 (元実装は
  ROS1 / Ubuntu 18.04 が主)。失敗時の代替案として Removert または
  GLIM 内蔵の dynamic_remover (もしあれば) を Alternatives 節に記録

### Issue M5R-5: `docs/maps/<site>/` 成果物規約の確立

- **目的**: 親方針 §6 受け入れ基準 (3) の「pcd + pgm + yaml + 取得日 / 経路 /
  天候のメタデータ」を、ディレクトリ規約 + README 雛形 + `metadata.yaml`
  スキーマで固める。既存 `docs/m5-maps/` (旧 M5-b 残骸) との整理方針も含む
- **受け入れ基準**:
  - [ ] `docs/maps/README.md` (新規) に次を記載:
    - ディレクトリ規約: `docs/maps/<site>/` 配下に `static.pcd` /
      `occupancy.pgm` / `occupancy.yaml` / `metadata.yaml` を置く
    - `metadata.yaml` の必須フィールド: `acquired_at` (ISO8601 日時) /
      `route_summary` / `weather` / `slam_method` (`glim` or `fast_lio_sam`) /
      `slam_params` (引用元 yaml への参照可) / `erasor_params` /
      `source_bag` (`docs/m5r-bench-data/...` 相対パス) / `commit` (この SHA)
    - 規約サンプル (`docs/maps/_template/` に sample ファイルセット)
  - [ ] `.gitignore` 更新: 既存の `docs/maps/**/*.pcd` (= 大きいファイル除外)
    は維持、`*.pgm` / `*.yaml` は tracked にする (Nav2 で必要)
  - [ ] 既存 `docs/m5-maps/` の処遇を確定 (リネーム or 削除 → legacy-findings
    に履歴記録)
- **スコープ外**: 実 PCD / pgm の生成 (M5R-3, M5R-6 担当)
- **担当 agent**: `pm-orchestrator` (規約決定) → `ros2-implementer`
  (`docs/maps/README.md` と `_template/` 作成)
- **想定ブランチ**: `m5r/5-maps-spec`

### Issue M5R-6: 占有格子変換スクリプト

- **目的**: 静的 PCD から Nav2 互換の 2D 占有格子 (pgm + yaml) を生成する
  スクリプト。旧 M5-b の `pcd_to_occupancy_grid.py` 系統が `docs/m5-maps/lab.yaml`
  を生んだ実績があるため、これを参照しつつ `docs/maps/<site>/` 規約に出力する
- **受け入れ基準**:
  - [ ] `scripts/m5r_pcd_to_occupancy.py` (新規、または旧 M5-b 由来の改修):
    入力 `static.pcd`、出力 `occupancy.pgm` + `occupancy.yaml` (Nav2 map_server
    互換: `image` / `resolution` / `origin` / `negate` / `occupied_thresh` /
    `free_thresh`)
  - [ ] スクリプトの冪等性: 既存ファイルへの上書きはユーザー確認 (`--force`)
    で許可、デフォルトは abort
  - [ ] M5R-3 で採用 SLAM が出した 1 つ以上の bag に対し、エンドツーエンドで
    `static.pcd → occupancy.pgm + .yaml` 変換が成功
  - [ ] `docs/ja/m5r-pipeline.md` (M5R-7) にパラメータ解説 (`resolution`、
    `z_clip` の上下限など) を追記
- **スコープ外**: 動的除去 (M5R-4)、地図品質の Nav2 動作確認 (M6-R)
- **担当 agent**: `ros2-implementer`
- **想定ブランチ**: `m5r/6-occupancy-grid`

### Issue M5R-7: パイプライン統合 + M5-R 完了文書

- **目的**: M5R-1〜M5R-6 の成果を「bag 取得 → SLAM → 動的除去 → 占有格子変換 →
  `docs/maps/<site>/` 格納」の一連パイプライン文書としてまとめる。M6-R への
  引き渡し条件を明文化
- **受け入れ基準**:
  - [ ] `docs/ja/m5r-pipeline.md` / `docs/en/m5r-pipeline.md` (新規、二言語化
    ADR-0001 準拠) に次を記載:
    - bag 取得手順 (M4-R bringup launch を立ててから何を録るか)
    - SLAM 実行コマンド (採用 SLAM ごと)
    - ERASOR 実行コマンド
    - 占有格子変換コマンド
    - `metadata.yaml` 記入要領
    - 最終成果物配置 (`docs/maps/<site>/...`)
  - [ ] `docs/m5r-bench-data/README.md` (新規。雛形は `docs/m4r-bench-data/README.md`
    に倣う) で bag 取得規約を確定。`.gitignore` 更新も含む
  - [ ] `velodyne_whill.yaml` の `map_file_path` ハードコード行を本パイプライン
    出力先に向け直す (即時稼働は要らない、コメントで「M5-R 規約に合わせた」と
    残す)
  - [ ] CLAUDE.md の「進行中の既知課題」P5 (地図品質側) の解消をマーク
    (commit 別、本 Issue では下書きまで)
  - [ ] `0003-mapping-slam-choice.md` を `accepted` 化するユーザー承認手順を
    `docs/ja/m5r-pipeline.md` 末尾に明記
- **スコープ外**: scan-to-map localizer の実装 (M6-R)、Nav2 への costmap 流し込み
  (M6-R)
- **担当 agent**: `pm-orchestrator` → `ros2-implementer` (文書化)
- **想定ブランチ**: `m5r/7-pipeline-doc`

## 7. 実行順序と依存

```
M5R-1 (GLIM setup) ────┐
                       ├──> M5R-3 (比較 + ADR-0003) ──> M5R-4 (ERASOR) ──> M5R-6 (occupancy)
M5R-2 (FAST-LIO SAM ──┘                                                            │
       prep)                                                                       │
                                                                                   ▼
M5R-5 (maps spec)  ────────────────────────────────────────────────────────> M5R-7 (統合文書)
```

- M5R-1 と M5R-2 は依存なし、並列実行可能
- M5R-3 は M5R-1 と M5R-2 の両方が必要 (両者を回して比較する)
- M5R-5 (規約) は他と独立に進められるが、M5R-3 と並列で進めるのが効率的
- M5R-4 → M5R-6 → M5R-7 は直列。最後の M5R-7 で全成果物が `docs/maps/<site>/`
  に揃う

実機共有制約 (WHILL 1 台、bag 取得はユーザー実走) を考慮すると、bag 取得は
M5R-3 着手直前にまとめて行うのが効率的。M5R-1 / M5R-2 はソフトウェア側だけで
完結するので並列実行可能。

## 8. 検証戦略

### 8.1 各 Issue の検証

| Issue | 検証方法 |
|-------|---------|
| M5R-1 | `vectorAdd` PASS、GLIM サンプル bag で trajectory 出力をスクリーンショット |
| M5R-2 | upstream LICENSE 確認 → 物理削除 + clone-on-demand 手順整備 → 受け入れ基準 (a)/(b) 充足。実 colcon build は M5R-3 評価担当者が実施 |
| M5R-3 | 同一 bag に GLIM / FAST-LIO SAM をかけた PCD 2 枚を CloudCompare で重ね、ループ誤差を数値化。ADR-0003 起案 |
| M5R-4 | 動的 bag 入りの除去前後 PCD を `scripts/m5r_erasor_diff.py` で差分表示。歩行者軌跡消失を目視 |
| M5R-5 | `docs/maps/_template/` の全ファイルが規約通りに揃っている。`metadata.yaml` lint (key 検証スクリプトでも可) |
| M5R-6 | サンプル `static.pcd` → `occupancy.pgm + .yaml` 変換成功、RViz の map_server で開ける |
| M5R-7 | E2E: bag → SLAM → ERASOR → occupancy → `docs/maps/<site>/` 配置までを 1 site 分通す |

### 8.2 ベンチデータ規約 (M5R-7 で確定)

`docs/m5r-bench-data/<YYYY-MM-DD>-<run-id>/` に bag (gitignored)、README、
派生中間ファイル (PCD は大きいので gitignored、目視確認用スクショは tracked)
を置く。`docs/m4r-bench-data/` と同じ「README は外に lift して tracked」
方式を踏襲。

最終成果物 (静的 PCD + 占有格子 + メタデータ) は `docs/maps/<site>/` 配下に。
両者は別ディレクトリで、前者は「中間アーティファクト」、後者は「運用入力」と
役割分離する。

### 8.3 ループ閉合誤差 0.5 m の合格根拠

- M4 期 FAST-LIO 単独 (ループクロージャなし): 60 s で 18% (実測、`run2`)
- ループクロージャ付き SLAM (GLIM / FAST-LIO SAM) は大域最適化で
  始終点を強制一致させるため、原理的に「数十 cm オーダー」に収まる
- 屋内 50 m ループでの ICP / NDT 再現性 (M3 期 NDT 評価) は数十 cm
- 50 m × 1% = 0.5 m。これより悪い場合は地図品質が運用 (M6-R scan-to-map) で
  かえって性能低下要因になり得るため、本フェーズで再取得 or パラメータ調整を
  必須化する

## 9. リスクと不確実性

### 9.1 リスク

- **GLIM の GPU メモリ要件超過**: RTX 3080 Laptop の VRAM 16 GB。GLIM 公式
  ベンチは Jetson Orin (8 GB / 32 GB) と x86 GPU で。長時間 bag (10 分超) で
  VRAM 不足が起きる可能性。緩和策: 短時間 bag (1〜3 分) に分割、または
  GLIM の CPU モードで再評価。M5R-3 の比較項目に「所要 VRAM」を入れる
- **ERASOR の humble 対応不確実性**: 元実装は ROS1 / Ubuntu 18.04 中心。
  ROS 2 humble + Ubuntu 22.04 でビルド可能か未確認。緩和策: M5R-4 着手時に
  まず PoC ビルドを行い、失敗時は Removert または GLIM 内蔵 dynamic 処理 (もし
  あれば) に Alternatives 切替
- **bag 取得時の bringup 不整合**: M4-R bringup launch が確実に立ち上がる前提だが、
  実機 USB 認識・udev rule 未整備等で `/whill/odom` が出ないケース。緩和策:
  M5R-3 着手前に「短時間の bag 取得で `/tf_static` `/whill/odom` `/imu/data_raw`
  `/velodyne_points` 全てが録れている」smoke test を user に依頼
- **FAST-LIO SAM のライセンスが GPL 系で運用パッケージへの link 禁止が制約**:
  親方針 §3.4 で許容範囲。ただし採用したら「永続的に運用スタックから分離する」
  契約が発生するため、M5R-2 の Alternatives 節と M5R-7 の `docs/ja/m5r-pipeline.md`
  で明示
- **採用 SLAM が 1 ヶ月以内に上流変更される**: GLIM / FAST-LIO SAM とも活発に
  開発が続いており、API 変更で再ビルド失敗のリスク。緩和策: 採用版を ADR-0003
  で commit SHA / tag で pin し、`whill_lab.repos` (採用なら) または手順書
  (FAST-LIO SAM の場合) に明記
- **`docs/m5-maps/` 残骸の削除判断ミス**: 既存ファイル `lab.yaml` / `lab.pgm` を
  削除すると、それを参照している何かが壊れる可能性。緩和策: M5R-5 着手時に
  grep で参照箇所を確認 → 壊れる場合はリネーム (削除しない) で対応

### 9.2 不確実性

- **キャンパス実走行 bag が M5-R 期間中に取れるか**: 親方針 §6 B1〜B3 は
  研究室内ループ走行 bag で達成可能。本番キャンパス bag は M5-R 完了後・
  M6-R 着手と同時 or 直前に取得する運用判断 (本計画 §2.2)。ただし「キャンパス
  サイズで GLIM が動くか」は M6-R で別途検証する必要が出る可能性。M5-R で
  は **室内 50 m ループスケールの動作確認まで** をスコープとする
- **動的除去の閾値**: ERASOR の `r_min`、`r_max`、`voxel size` 等のチューニングは
  屋内 / 屋外で振る舞いが変わる。M5R-4 では研究室屋内で 1 セット確定、屋外
  本番では M6-R 着手時の bag で再チューニングが必要になる可能性を明記
- **`base_link` 原点の M5-R 影響**: M4R-2 で「後輪車軸中心、地面高さ」と仮置きした
  base_link が、M5-R で生成する地図の原点と整合するかは M6-R で初めて判明。
  本計画では M5-R 中は base_link を再定義しない方針 (M4-R 方針維持)

## 10. 後続フェーズへの引き渡し

M5-R 完了時点で次が確定する:

- 採用 SLAM (ADR-0003 で確定。GLIM または FAST-LIO SAM)
- 採用動的除去手法 (ERASOR または代替。M5R-4 で確定)
- `docs/maps/<site>/` 規約 (static.pcd + occupancy.pgm + occupancy.yaml +
  metadata.yaml)
- パイプライン文書 `docs/ja/m5r-pipeline.md`
- bag 取得規約 `docs/m5r-bench-data/README.md`
- 研究室内テスト site 1 つ分の完成成果物 (例: `docs/maps/lab-loop/`)

これを前提に:

- **M6-R**: scan-to-map localizer (`lidar_localization_ros2` 第一候補) が
  `docs/maps/<site>/static.pcd` を入力に `map -> odom` を publish する。
  initial pose 運用も M6-R で追加
- **M6-R**: Nav2 obstacle layer が `docs/maps/<site>/occupancy.yaml` を
  map_server に流し込み、`use_collision_detection: true` 復帰
- **キャンパス本番経路**: M5-R で確立したパイプラインをそのままキャンパスループ
  bag に適用。M6-R 着手直前または並列でユーザーが本番 bag を取得し、
  `docs/maps/utsunomiya-campus/` を生成

## 11. ADR の候補

このフェーズで生まれる技術判断:

- [ ] **ADR-0003: マップ作成 SLAM の選定 (GLIM vs FAST-LIO SAM)**。Issue M5R-3
  で実 bag 比較後に `docs/decisions/0003-mapping-slam-choice.md` (proposed) を
  起案、ユーザー承認後に accepted。親方針 §7 で「GLIM 採用の前提条件は
  満たされた。実 bag での GLIM vs FAST-LIO SAM 比較後に確定する」と明示済
- [ ] **ADR-0004 候補: 動的物体除去手法の選定 (ERASOR vs Removert vs GLIM 内蔵)**。
  Issue M5R-4 で ERASOR がビルド不可・性能不足だった場合に起案。順調なら
  ADR 起案は不要 (親方針 §3.3 で ERASOR が第一候補と明示済)
- [ ] **ADR-0005 候補: `docs/maps/<site>/` 規約の確定**。Issue M5R-5 で
  `metadata.yaml` スキーマと `_template/` を確定する際、規約として ADR 化する
  価値あり (M6-R / M9 で後発が触る規約のため)。判断は M5R-5 着手時

## 12. 次のアクション

本計画書が `accepted` 化されたら、M5-R 着手に必要な作業:

1. M5R-1〜M5R-7 の 7 件を `gh issue create` で起案する (本計画は分割を
   定めるが、`gh issue create` 自体は別ステップ)
2. M5R-1 (CUDA 確認 + GLIM 母艦インストール) と M5R-2 (FAST-LIO SAM 整理) を
   並列着手
3. M5R-5 (maps 規約) を M5R-3 と並列で進め、雛形を先に作る
4. M5R-1 / M5R-2 完了後、ユーザーに「室内ループ走行 bag」と「歩行者横断 bag」を
   依頼。取得完了後 M5R-3 着手
5. M5R-3 → M5R-4 → M5R-6 → M5R-7 の直列を進め、ADR-0003 を accepted 化して
   フェーズ完了。CLAUDE.md の P5 解消反映と M6-R 着手準備

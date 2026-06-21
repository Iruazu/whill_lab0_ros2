# M5R-3: GLIM vs FAST-LIO SAM 実 bag 比較プロトコル

Language: [日本語](m5r3-comparison-protocol.md) | [English](../en/m5r3-comparison-protocol.md)

## 目的

M5R-3 (Issue #48) の Phase B (実 bag による比較計測) を、評価担当者が再現できる形で一元化する。本文書の到達点は次の 4 点を **ADR-0003 ([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md)) の Context / Alternatives 節に転記できる形で揃える** こと:

- 同一の bag に対する GLIM と FAST-LIO SAM の生成 PCD
- 各 SLAM のループクロージャ誤差 (B1 公式評価 = CloudCompare による壁面 3 点平均、補完指標 = trajectory 始終点距離)
- 各 SLAM の所要時間 / 所要 VRAM (および FAST-LIO SAM 側の所要 host RAM)
- 各 SLAM の操作性メモ (manual relocalization の要否、keyframe 発行密度、ループクロージャ発火タイミング 等の定性データ)

これらが揃った時点で、別 commit で ADR-0003 の Decision 節を埋めて Phase A の PR を ready 化する。本文書は Phase B 実行マニュアルであり、Phase A 着地時点では数値部分は空欄で merge される。

採択経緯と要件のひもづけは [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 と §7、Issue 構造は [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md) §M5R-3 を参照。

## 前提環境

| | |
|--|--|
| M5R-1 (#45) | GLIM 母艦インストール完了。`/usr/local/lib/libgtsam.so.4.3a0`, `libgtsam_points_cuda.so`, `libiridescence.so` が揃い、`ros2 pkg list \| grep glim_ros` が成功する |
| M5R-2 (#46) | FAST-LIO SAM は clone-on-demand 運用。本書 §3 の手順で評価開始前に clone + build する |
| M5R-5 (#47) | `docs/maps/<site>/` 規約確立済 (採用 SLAM 確定後に静的 PCD を流す先)。本 Issue の中間アーティファクトは `docs/m5r-bench-data/` を使う |
| M4-R bringup | `ros2 launch whill_localization odom_bringup_launch.py` で sensors + driver + EKF を起動できる。bag 録音時に `/tf_static` に M4R-2 (#36) 実測 extrinsic が乗る |

## GTSAM 競合 (最重要の前提)

本評価では同一母艦で 2 種類の GTSAM が共存する:

| 用途 | バージョン | 配置 | 由来 |
|---|---|---|---|
| GLIM | 4.3a0 | `/usr/local/lib/libgtsam.so.4.3a0` | `scripts/install_glim.sh` (M5R-1) |
| FAST-LIO SAM | 4.1.1 | `/usr/lib/x86_64-linux-gnu/libgtsam.so.4.1.1` | `scripts/clone_fastlio_sam_for_eval.sh` (M5R-2、borglab PPA) |

両者は ABI 非互換であり、**同一プロセスで両方 link すると挙動不定** (起動時に動的リンカが拾うバージョンに依存して symbol mismatch crash になり得る)。本書の運用方針:

- GLIM 走行と FAST-LIO SAM 走行は **別ターミナル・別シェル** で実行する
- FAST-LIO SAM 側で症状が出たら `LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH` を強制してから再実行する (4.1 を優先させる)
- `scripts/m5r3_run_fastlio_sam.sh` は走行前に `ldconfig -p | grep libgtsam` を `gtsam_env.log` に dump し、両バージョンが見える状態なら警告を出す。この状態は ADR-0003 Alternatives の row として記録する

詳細は [`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md) §3 を参照。

## LiDAR の差異 (GLIM の config 切替)

GLIM 上流のサンプル bag は Ouster OS1-128 (`/points` topic) だが、本リポの M4-R bringup は Velodyne VLP-16 (`/velodyne_points` topic) を出す。`scripts/m5r3_run_glim.sh` は bag の `metadata.yaml` 内に `/velodyne_points` が含まれるかで GLIM の config bundle を切り替える:

- `/velodyne_points` あり → `config_velodyne/` (上流に無ければ `config_velodyne_vlp16/`、それも無ければ標準 `config/` にフォールバックして警告)
- それ以外 → 標準 `config/` (Ouster サンプル用)

フォールバックが発火した場合、その事実は比較条件としての対称性を崩すリスクがあるため、ADR-0003 Alternatives 節に必ず記録する (例: 「Velodyne 専用 config が上流に ship されておらず、Ouster 用 config で走らせた。前処理の ring 仮定が異なるため feature extraction が劣化している可能性あり」)。

## Phase B 実行手順

### 1. bag 取得 (ユーザー作業)

M4-R bringup launch で sensors + driver + EKF を起動した上で、室内ループ走行 bag を取る。

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
ros2 launch whill_localization odom_bringup_launch.py
```

別ターミナルで bag を録音:

```bash
mkdir -p docs/m5r-bench-data/$(date +%Y-%m-%d)-loop
cd docs/m5r-bench-data/$(date +%Y-%m-%d)-loop
ros2 bag record -o bag /velodyne_points /imu/data_raw /tf /tf_static
```

走行条件:

- 室内 50 m 程度のループ (始点と終点を物理的に同じ位置に合わせる。B1 公式評価は始終点の壁面 3 点平均で測るため、始点と終点で同じ壁を視野に入れる必要がある)
- 平均速度 ~0.3 m/s。急加速・急旋回は FAST-LIO 系の IMU bias 推定が暴れるため避ける
- ループ閉鎖の物理マーカ (テープ・床のラインなど) を始終点に置くと CloudCompare での点拾いが容易

`Ctrl-C` で bag 停止。生成された `bag/*.db3` (または `.mcap`) と `bag/metadata.yaml` を確認する。

### 2. GLIM 走行

GLIM 用シェル (LD_LIBRARY_PATH 等を触らないクリーンな状態):

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
RUN_DIR=docs/m5r-bench-data/$(date +%Y-%m-%d)-loop

bash scripts/m5r3_run_glim.sh ${RUN_DIR}/bag ${RUN_DIR}/glim-out
```

走行中の観察ポイント (定性データ、ADR-0003 Alternatives に転記):

- Iridescence のウィンドウで **ループクロージャがどのタイミングで発火したか** (走行後半のどのキーフレームで pose graph がジャンプしたか)
- **キーフレーム発行密度** (1 m あたり何枚程度)
- マニュアル介入の要否 (relocalization、停止からの復帰 等)
- VRAM 使用量のピーク (`${RUN_DIR}/glim-out/vram.log` に 0.5 s 間隔で記録される)

完了後の出力:

- `${RUN_DIR}/glim-out/traj_lidar.txt` (TUM 形式 trajectory)
- `${RUN_DIR}/glim-out/dump.pcd` または `map.pcd` (生成 PCD)
- `${RUN_DIR}/glim-out/manifest.yaml` (実行メタデータ、results 節は TBD のまま)
- `${RUN_DIR}/glim-out/run.log` (`/usr/bin/time -p` 出力含む stdout/stderr)
- `${RUN_DIR}/glim-out/vram.log` (0.5 s 間隔の `nvidia-smi` ダンプ)

### 3. FAST-LIO SAM 走行

初回のみ、clone + build。**§「GTSAM 競合」を読んでから実行する**。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
export FASTLIO_SAM_LICENSE_ACK=yes
bash scripts/clone_fastlio_sam_for_eval.sh
colcon build --packages-up-to fast_lio_sam --symlink-install
```

`colcon build` が失敗した場合 (上流の "Full ROS2 adaptation" TODO が残っているため可能性あり):

1. 失敗ログを `${RUN_DIR}/fastlio-sam-out/build-failure.log` に保存
2. CLAUDE.md 規約により `src/third_party/` の直接編集は禁止。本リポ内 wrapper で対応するか、ADR-0003 Alternatives 節に「FAST-LIO SAM は M5R-3 着手時点で上流 master が build しない」と記録して評価対象外にする
3. 後者の場合、本 §3 以降の手順はスキップし §4 (ループ誤差計測) に飛んで GLIM 側のデータだけで ADR を埋める

build が通った場合、各 bag に対して:

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
RUN_DIR=docs/m5r-bench-data/$(date +%Y-%m-%d)-loop

bash scripts/m5r3_run_fastlio_sam.sh ${RUN_DIR}/bag ${RUN_DIR}/fastlio-sam-out
```

GTSAM 競合警告 (`ldconfig -p` で 4.1 と 4.3 が両方見える) が出た場合の対処:

```bash
# 4.1 を優先させて再実行
LD_LIBRARY_PATH=/usr/lib:${LD_LIBRARY_PATH:-} \
  bash scripts/m5r3_run_fastlio_sam.sh ${RUN_DIR}/bag ${RUN_DIR}/fastlio-sam-out --force
```

完了後の出力 (GLIM と schema 一致):

- `${RUN_DIR}/fastlio-sam-out/traj.txt` (TUM 形式 trajectory、上流のリリースによっては別名)
- `${RUN_DIR}/fastlio-sam-out/map.pcd` (生成 PCD)
- `${RUN_DIR}/fastlio-sam-out/manifest.yaml`
- `${RUN_DIR}/fastlio-sam-out/run.log` + `slam.log` (SLAM ノード stdout/stderr)
- `${RUN_DIR}/fastlio-sam-out/vram.log` + `rss.log`
- `${RUN_DIR}/fastlio-sam-out/gtsam_env.log` (走行時の `ldconfig -p` snapshot)

### 4. ループ誤差計測

#### 4.1 補完指標: trajectory 始終点距離

`scripts/m5r3_loop_error.py` を両 trajectory に対して走らせる:

```bash
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/fastlio-sam-out/traj.txt
```

このスクリプトは **SLAM が自己申告するループ閉鎖誤差** (trajectory の最初の pose と最後の pose の Euclidean 距離) を計算する。SLAM 内部の pose graph が閉じきれていない場合に大きな値が出る。

ADR に転記する数値が必要な場合は `--json` を付ける:

```bash
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt --json
```

#### 4.2 公式指標 (B1): CloudCompare 壁面 3 点平均

[`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md) §6 B1 で「始終点の同一壁面 3 点平均で ≤ 0.5 m」と定めた指標は、生成 PCD に対する物理位置計測で評価する。CloudCompare で:

1. CloudCompare 2.12.x 以降を `sudo apt install cloudcompare` で導入 (Ubuntu 22.04 の universe にあり)
2. `${RUN_DIR}/glim-out/dump.pcd` (または `map.pcd`) を開く
3. **Point picking ツール** (ショートカット `P`、または右クリック → Pick Points; CC のバージョン差で Edit / Display / Tools メニューのどこに置かれるか変わるためショートカットが安定) で 3 点ピックモードに入る
4. 走行開始直後にスキャンした壁面の 3 点 (角・特徴点・床との交点 等) と、走行終端で同じ壁面の対応 3 点をピック
4. 各組の 3D 座標差を取り、3 点平均距離を ADR-0003 Alternatives の `loop_error_wall_3pt_m` row に記録
5. FAST-LIO SAM 側でも同じ壁・同じ 3 点を選んで同様に計測

**4.1 と 4.2 は別物**: 4.1 は SLAM 内部誤差、4.2 は世界座標系での物理誤差。両方記録すること。

### 5. 数値の ADR-0003 反映

各 SLAM の以下をテーブルに転記する:

| データ | 出所 | ADR-0003 反映先 |
|---|---|---|
| 走行時間 (duration_sec) | `manifest.yaml` | Alternatives テーブルの「所要時間」列 |
| ピーク VRAM (max_vram_mib) | `manifest.yaml` | Alternatives テーブルの「ピーク VRAM」列 |
| ピーク RSS (FAST-LIO SAM のみ) | `manifest.yaml` | Alternatives テーブルの「ピーク RSS」列 |
| trajectory 始終点距離 | `m5r3_loop_error.py` 出力 | Alternatives テーブルの「内部誤差」列 |
| 壁面 3 点平均距離 | CloudCompare 計測 | Alternatives テーブルの「B1 誤差」列 |
| GTSAM 解決状況 | `gtsam_env.log` + 警告有無 | Alternatives テーブルの「GTSAM 解決」列 |
| 操作性メモ | 走行中観察 | Alternatives テーブルの「定性所見」列 |
| ライセンス | `m5r-glim-setup.md` / `m5r-fastlio-sam-eval.md` | Consequences 節 |

bag が複数本ある場合は各 bag を独立の row として記録し、最終 Decision は全 row を見て総合判断する。

### 6. Decision と PR ready 化

数値が揃ったら、別 commit で `docs/ja/decisions/0003-mapping-slam-choice.md` と `docs/en/decisions/0003-mapping-slam-choice.md` の Decision 節 placeholder を埋める。記載すべき内容:

- 採用 SLAM (GLIM / FAST_LIO_SAM)
- Commit SHA / Tag pin (採用した時点の `git_commit` を `manifest.yaml` から転記)
- 判断根拠 (B1 誤差、ループクロージャの有無、ライセンス、操作性)

その上で PR を `ready for review` に切り替え、ユーザー承認後に Status を `proposed → accepted` に書き換える別 commit を入れる (ADR-0001 §5 の運用)。

## 関連

- 開発方針: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 (SLAM 候補)、§3.4 (ライセンス)、§7 (ADR-0003 起案要請)
- M5-R 実行計画: [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md) §M5R-3、§6 (受け入れ基準 B1〜B5)
- 前置文書: [`m5r-glim-setup.md`](m5r-glim-setup.md) (GLIM source build)、[`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md) (FAST-LIO SAM clone-on-demand)
- スクリプト: [`scripts/m5r3_run_glim.sh`](../../scripts/m5r3_run_glim.sh)、[`scripts/m5r3_run_fastlio_sam.sh`](../../scripts/m5r3_run_fastlio_sam.sh)、[`scripts/m5r3_loop_error.py`](../../scripts/m5r3_loop_error.py)
- ADR: [`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md) — Phase B 完了後にデータを反映
- 関連 Issue: #48 (本 Issue、M5R-3)、#45 (M5R-1)、#46 (M5R-2)、#47 (M5R-5)、#49 (M5R-4 = 採用 SLAM の出力を ERASOR に流す後続)

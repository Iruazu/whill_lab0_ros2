# M5-R マップ作成パイプライン

Language: [日本語](m5r-pipeline.md) | [English](../en/m5r-pipeline.md)

本書は M5-R (マップ作成) パイプライン全体の参照ドキュメント。Issue #49 で
動的除去 (DUFOMap) section、Issue #50 で占有格子変換 section、Issue #51
(M5R-7 パイプライン統合) で bag 取得 / SLAM 実行 / `docs/maps/<site>/`
配置 / M6-R への引き渡し条件まで整備した M5-R 完成版。

対象読者: M5-R で新規マップを作成する人 (= ユーザー自身が site を生成する
オフライン作業)。M6-R 担当者は本書末尾の「M6-R への引き渡し」節だけ
読めば入力契約が分かる。

## E2E パイプライン

```
[1] bag 録画 (M4-R bringup)
        | /velodyne_points + /imu/data_rep145 + /tf_static
        v
    docs/m5r-bench-data/<run-id>/bag/         (gitignored)

[2] SLAM (ADR-0003: GLIM)
        | scripts/m5r3_run_glim.sh
        v
    docs/m5r-bench-data/<run-id>/glim-out/    (gitignored)
        - NNNNNN/{points,intensities,...}_compact.bin
        - traj_lidar.txt, manifest.yaml

[3] 動的物体除去 (ADR-0004: DUFOMap)
        | scripts/m5r_run_dufomap.sh
        v
    docs/m5r-bench-data/<run-id>/dufomap-out/static.pcd   (gitignored)

[4] 占有格子変換 (Nav2 互換)
        | scripts/m5r_pcd_to_occupancy.py <static.pcd> <output-dir>
        v
    <output-dir>/occupancy.pgm + occupancy.yaml
    (典型: docs/m5r-bench-data/<run-id>/dufomap-out/ に出して
     ステップ 5 で docs/maps/<site>/ に移動、または直接
     <output-dir> を docs/maps/<site>/ にして mv を省く)

[5] docs/maps/<site>/ への配置 (登録)
        - static.pcd        (gitignored、PCD は大きいので)
        - occupancy.pgm     (tracked)
        - occupancy.yaml    (tracked)
        - metadata.yaml     (tracked、ADR-0005 規約)
        - README.md         (tracked、任意)
```

site 1 つ分の所要時間目安 (Alienware x15 R2、200 秒走行 bag):
GLIM ~10 分 (GPU、Iridescence 起動) + DUFOMap ~3 秒 + 占有格子変換 ~1 秒。
ボトルネックは bag 録画と GLIM 後処理。

## bag 録画 (ステップ 1)

### 前提
- M4-R で確定した bringup launch (`whill_localization/launch/odom_bringup_launch.py`)
  が起動可能な状態
- Issue #56 の `imu_sign_corrector` ノードが起動して `/imu/data_rep145` を
  100 Hz で publish 中 (`ros2 topic hz /imu/data_rep145` で確認)
- 走行ルート: ループ走行が望ましい (始終点同一壁面で SLAM ループクロージャ
  精度を測れる)。M5R-3 の検証 bag は 50 m / 200 s 規模、ADR-0003 §評価条件参照
- **ランタイム環境**: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` および
  CPU governor=`performance` であること。両者が揃っていないと
  `/velodyne_points` が 1 Hz 前後に詰まる現象が再現する
  (詳細: `m5r-rmw-cyclonedds.md`)

### 録画コマンド

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ランタイム環境の sanity check (録画前に必ず)
echo $RMW_IMPLEMENTATION                                  # rmw_cyclonedds_cpp
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor # performance
# powersave なら sudo cpupower frequency-set -g performance

# 1. bringup (sensors + driver + EKF)
ros2 launch whill_localization odom_bringup_launch.py
```

別ターミナル:

```bash
mkdir -p docs/m5r-bench-data/$(date +%Y-%m-%d)-<run-id>
ros2 bag record \
  -o docs/m5r-bench-data/$(date +%Y-%m-%d)-<run-id>/bag \
  /velodyne_points /imu/data_rep145 /tf_static
# 走行 → Ctrl-C
```

録画する topic は **3 本だけ**:
- `/velodyne_points` (10 Hz) — VLP-16 raw
- `/imu/data_rep145` (100 Hz) — Issue #56 で REP-145 化済の IMU
- `/tf_static` — sensor extrinsics 群 (base_link → velodyne / imu_link 等)

`/imu/data_raw` も内蔵 driver は publish しているが、SLAM 投入向けは
REP-145 規約の `/imu/data_rep145` の方を録る。`/odometry/filtered` 等の
EKF 出力は M4-R 検証用で、GLIM は IMU + LiDAR しか使わないため録らない
(録ると bag 容量が嵩む)。

**注意: `--compression-mode file --compression-format zstd` は使わない** —
GLIM の `glim_rosbag` は rosbag2 の compression plugin を引いていないため、
zstd 圧縮された `bag_0.db3.zstd` を sqlite3 として直接開こうとして失敗する
("file is not a database" エラー)。容量を抑えたければ録画後に
`ros2 bag convert` で別ディレクトリに書き出すか、外部圧縮で archive する。

`docs/m5r-bench-data/` 配下の `<run-id>` ディレクトリ命名規約と gitignore
規約は [`../../m5r-bench-data/README.md`](../../m5r-bench-data/README.md)
を正本とする。

### 録画後の bag 検証

```bash
ros2 bag info docs/m5r-bench-data/<run-id>/bag
```

期待 (M5R-3 outdoor-loop bag を例にすると):
- Duration: 200 秒前後
- `/velodyne_points` Count: ~2000 (200s × 10Hz)
- `/imu/data_rep145` Count: ~20000 (200s × 100Hz)
- `/tf_static` Count: ≥1

`/imu/data_rep145` の rate が 100 Hz を大きく下回る場合は録画中に bringup
が CPU 飽和した可能性。録画ターミナル以外で RViz や rqt を併走させない。

`/velodyne_points` の count が走行秒 × 10 の半分以下なら、FastDDS の
大メッセージ配送詰まり (`m5r-rmw-cyclonedds.md`) を疑う。`RMW` と
governor を確認して再録画する。

## SLAM 実行 (ステップ 2): GLIM

ADR-0003 で **GLIM** に確定 ([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md))。
FAST-LIO SAM は upstream LICENSE 不在のため棄却。

### 前提
- M5R-1 (#45) の CUDA 12.4 + cuDNN セットアップ完了
  ([`m5r-cuda-setup.md`](m5r-cuda-setup.md) 参照)
- GLIM 母艦ビルド完了 (`ros2 pkg list | grep glim_ros` で確認)
  ([`m5r-glim-setup.md`](m5r-glim-setup.md) 参照)

### 実行コマンド

```bash
scripts/m5r3_run_glim.sh \
  docs/m5r-bench-data/<run-id>/bag \
  docs/m5r-bench-data/<run-id>/glim-out
```

スクリプトは:
- VRAM ピークを 0.5 s 周期でサンプリングして `vram.log` に記録
- 終了時に `manifest.yaml` を出力 (採用 SLAM、bag、開始/終了時刻、duration、
  exit code、CUDA バージョン、git commit、ADR-0003 で参照する results 空欄)
- 冪等性: 既存 `<out-dir>` には `--force` なしで abort

### 旧 bag (Issue #56 以前録画) を扱う場合

`/imu/data_rep145` ではなく `/imu/data_raw` (gravity-vector 規約) で
録画されている場合、GLIM の初期姿勢推定が 171° 反転する既知症状あり。
`scripts/m5r3_fix_imu_bag.py` で先に bag を書き換える:

```bash
python3 scripts/m5r3_fix_imu_bag.py \
  docs/m5r-bench-data/<old-run-id>/bag \
  docs/m5r-bench-data/<old-run-id>/bag-imu-fixed
# その後 m5r3_run_glim.sh の入力に bag-imu-fixed を渡す
```

詳細は同スクリプトの DEPRECATED docstring + ADR-0003 注記参照。
**新規録画では本ステップは不要** (#56 で恒久対応済)。

### 出力

`<out-dir>/`:
- `NNNNNN/` keyframe dir × N (例: 200s 走行で ~18 個)
  - `points_compact.bin` (binary, Eigen::Vector3f ベタ書き、keyframe ローカル)
  - `data.txt` (T_world_origin 4×4、その他)
  - `intensities_compact.bin`, `normals_compact.bin`, `covs_compact.bin`
- `traj_lidar.txt` (TUM format: timestamp tx ty tz qx qy qz qw)
- `manifest.yaml` (ADR-0003 入力用)
- `run.log`, `vram.log`

ループクロージャ誤差を ADR-0003 形式で記録するには:

```bash
python3 scripts/m5r3_loop_error.py <out-dir>/traj_lidar.txt
# end-to-start 距離 / ループ長 / yaw drift を出力
```

## 動的物体除去 (DUFOMap)

### 採用理由

[ADR-0004](decisions/0004-dynamic-removal-choice.md) を参照。要約:
ERASOR 系 (GPL-3.0 + ROS 1 専用) と Removert (LICENSE 不在) を棄却し、
DUFOMap (BSD-3-Clause, `pip install dufomap`, ROS 非依存) を採用。

### セットアップ

母艦 (Alienware x15 R2、Ubuntu 22.04 + Python 3.10) で 1 回だけ:

```bash
# Ubuntu 22.04 default は python3 だけで pip が入っていないので、まず pip を入れる
sudo apt install -y python3-pip
pip install dufomap
python3 -c 'import dufomap; print(dufomap)'   # サニティチェック
```

検証済み環境: Ubuntu 22.04.5 LTS + Python 3.10.x。DUFOMap のホイールには
UFO ライブラリがバンドルされており、追加の native install は不要。

### 実行

GLIM の出力ディレクトリ (`docs/m5r-bench-data/<run>/glim-out/` 等) から
静的 PCD を 1 コマンドで生成:

```bash
scripts/m5r_run_dufomap.sh <glim-out-dir> <output-dir>
```

例:

```bash
scripts/m5r_run_dufomap.sh \
  docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out \
  /tmp/m5r49_dufomap
# -> /tmp/m5r49_dufomap/static.pcd
```

中間ステップを個別に走らせる場合:

```bash
# 1. GLIM keyframe -> per-keyframe PCD (VIEWPOINT ヘッダ埋め込み)
scripts/m5r_glim_to_pcd.py \
  --glim-out <glim-out-dir> \
  --out-dir <staging-dir>

# 2. DUFOMap 本体
scripts/m5r_run_dufomap_core.py \
  --data-dir <staging-dir> \
  --output <static.pcd>
```

冪等性: いずれのスクリプトも既存ファイルへの上書きはデフォルトで abort。
`--force` で許可する (`scripts/m5r3_run_glim.sh` と同じ慣習)。

### パラメータ

上流既定値は `KTH-RPL/dufomap/assets/config.toml`。チューニング指針は
本表の右列、変更時は `docs/maps/<site>/metadata.yaml` の
`dufomap_params` フィールドに記録する (フィールド名は M5R-7/#51 で確定)。

| パラメータ | CLI フラグ | 既定値 | 説明 | チューニング指針 |
|---|---|---|---|---|
| resolution | `--resolution` | 0.1 m | voxel size | 高密度・屋内マップで 0.05、屋外広域で 0.2。VLP-16 屋外ループでは 0.1 で十分 |
| inflate_hits_dist (d_s) | `--d-s` | 0.2 m | hit 周辺の inflation | センサノイズ大なら 0.3〜0.5。VLP-16 距離精度 (~3 cm at 100 m) なら 0.2 でよい |
| inflate_unknown (d_p) | `--d-p` | 2 voxel | unknown 領域膨張 | DUFOMap paper 推奨そのまま。経験則として変更不要 |
| min_range | (未公開) | 0.2 m | ego 除外 | DUFOMap config.toml で固定。自己車体の bbox が大きい場合のみ要再検討 |
| max_range | (未公開) | -1 (無制限) | 遠方除外 | 屋外で 50 m 以上が信頼できない場合のみ要再検討 |
| num_threads | `--num-threads` | 12 | DUFOMap worker thread 数 | 母艦 CPU 物理コア数に合わせる (i9-12900H なら 12-14) |

### 入出力

入力: GLIM keyframe dir (`<glim-out>/NNNNNN/`):

- `points_compact.bin` — Nx3 ``Eigen::Vector3f`` ベタ書き、keyframe ローカル
  座標
- `data.txt` — テキスト dump、`T_world_origin:` 直後に 4x4 row-major float
  matrix (keyframe 原点の world 内 pose)

`m5r_glim_to_pcd.py` がこの 2 ファイルを per-keyframe PCD に変換する。
PCD は ASCII (volume が小さく、binary 化のメリットがファイル可読性損失に
見合わない)、`VIEWPOINT` ヘッダに `T_world_origin` の並進 + quaternion
(PCL 順 `qw qx qy qz`、scipy の `(x, y, z, w)` 順とは異なる) を埋め込む。
点群は keyframe ローカル座標のまま (DUFOMap が `cloud_transform=False` で
取り込み、内部で world 系に変換する)。

出力: 単一の静的 PCD (`<output-dir>/static.pcd`)。M5R-6 (#50) 占有格子変換の
入力、M6-R scan-to-map localizer の入力 (ADR-0005 規約)。

### 目視確認

`scripts/m5r_dufomap_diff.py` で除去前後の点群を重ね、歩行者軌跡が
消えていることを確認:

```bash
# 対話 viewer
scripts/m5r_dufomap_diff.py --before <raw>.pcd --after <static>.pcd

# スクリーンショット (CI / 文書化用)
scripts/m5r_dufomap_diff.py --before <raw>.pcd --after <static>.pcd \
  --screenshot diff.png
```

- before = 赤、after = 青
- 動的物体ありの bag で run しないと判定材料がないので、M4-R bringup を
  立てた状態で歩行者横断シーンを別途録る必要がある (M5R-4 受け入れ基準 B2)
- このスクリプトは `open3d` を要求 (`pip install open3d`)。容量大 (~100 MB)
  なので CI には入れない、対話的目視確認専用

### 既知の懸念点

- DUFOMap `outputMap` の出力ファイル名は version によって `dufomap_output.pcd`
  / `dufomap_output_voxel_map.pcd` 等のばらつきがある。
  `m5r_run_dufomap_core.py` は両方を候補に拾い、見つかった方を `--output`
  指定先に move する fallback を実装済 (LICENSE のクリーンさを保つため
  upstream のソースに patch を当てる方針は採らない)
- パラメータの `min_range` / `max_range` は DUFOMap の Python API で
  明示露出されていない。必要なら upstream `assets/config.toml` を編集して
  `pip install --force-reinstall .` し直す (本パイプラインでは現状未対応)

## 占有格子変換 (Nav2 互換)

DUFOMap が出力した `static.pcd` を Nav2 `map_server` がそのまま読める
`occupancy.pgm` + `occupancy.yaml` に変換する。`scripts/m5r_pcd_to_occupancy.py`
が担当 (Issue #50 / M5R-6)。

旧 `scripts/pcd_to_occupancy_grid.py` (M5-b 時代) はそのまま残してある
(`docs/maps/lab-legacy-m5b/` を `nav_launch.py` がまだ参照しているため)。
新規スクリプトは別ファイルで作っており、旧スクリプトとは独立に動く。
両者の差分の詳細は `scripts/m5r_pcd_to_occupancy.py` モジュール docstring
参照。要旨:

- XY 範囲は入力 PCD の bbox から自動算出 (旧スクリプトは ±20 m ハードコード)
- ray-cast の起点は占有点群の重心 (旧スクリプトは world (0, 0) 固定)
- 出力 YAML は `docs/maps/_template/occupancy.yaml` と同形 (`free_thresh: 0.196`、
  Nav2 公式 default)
- 旧スクリプトの outlier filter / clear-radius は廃止 (前者は DUFOMap が
  代替、後者は「椅子が world 原点から始まる」前提が消えたため不要)

### 使い方

```bash
scripts/m5r_pcd_to_occupancy.py <input.pcd> <output-dir> [options]
```

例 (#49 で作った static.pcd を変換):

```bash
scripts/m5r_pcd_to_occupancy.py \
  /tmp/m5r49_dufomap/static.pcd \
  docs/maps/lab-loop \
  --force
# -> docs/maps/lab-loop/occupancy.pgm
# -> docs/maps/lab-loop/occupancy.yaml
```

冪等性: 既存 `occupancy.pgm` / `occupancy.yaml` への上書きはデフォルト
abort (`--force` 必須)。他の M5-R スクリプトと同じ慣習。

### パラメータ

| パラメータ | CLI フラグ | 既定値 | 説明 |
|---|---|---|---|
| resolution | `--resolution` | 0.05 m | セルサイズ。`_template/occupancy.yaml` と同値 |
| Z slice 下限 | `--z-min` | 0.1 m | これ未満を捨てる。床面ノイズ・坂の除外 |
| Z slice 上限 | `--z-max` | 1.5 m | これ超を捨てる。鴨居・看板・天井等の除外 |
| ray-cast anchor | `--anchor-x`, `--anchor-y` | auto | 省略時は占有点群の XY 重心。U 字経路では明示指定推奨 |
| ray-cast 無効化 | `--no-raycast` | off | 占有点のみ stamp、それ以外 unknown。debug 用 |
| occupied_thresh | `--occupied-thresh` | 0.65 | YAML 出力値 |
| free_thresh | `--free-thresh` | 0.196 | YAML 出力値 (Nav2 公式 default) |
| 余白 | `--padding` | 2.0 m | bbox 外側に追加する余裕。端の障害物が見切れないように |
| 上書き許可 | `--force` | off | 既存 pgm/yaml の上書きを許可 |

### 出力

- `occupancy.pgm`: P5 binary、1 byte/cell、`P5\n# m5r_pcd_to_occupancy.py output\n<W> <H>\n255\n` ヘッダ + ピクセルデータ
- `occupancy.yaml`: `docs/maps/_template/occupancy.yaml` と同形フィールド
  (`image` / `resolution` / `origin` / `negate` / `occupied_thresh` /
  `free_thresh`)。先頭に入力 PCD 名を残す生成コメント 1 行

ピクセル値規約 (ROS `map_server`):

| 値 | 意味 |
|---|---|
| 0 | OCCUPIED (黒) |
| 254 | FREE |
| 205 | UNKNOWN |

### 既知の懸念点

- anchor 自動算出は **占有点群の重心** を使うため、U 字経路や L 字経路では
  centroid が navigable な領域外 (壁の内側等) に落ちることがある。その場合は
  `--anchor-x` / `--anchor-y` で明示指定する
- bbox が極端に細長い bag (長い直線走行) ではグリッドメモリが膨らむ。
  100M セル超で abort する safety を入れてあるので、引っかかった場合は
  `--resolution` を粗くするか PCD を pre-crop する
- ray-cast は **anchor 1 点** からの近似で、per-scan ray-cast (UFO/octomap が
  正攻法) は実装していない。オフライン処理で静的 PCD を入力にする本パイプラインの
  範囲では十分。per-scan が必要になったら別 Issue で keyframe 姿勢を本ステージまで
  運ぶことになる

## site 登録 (ステップ 5): `docs/maps/<site>/` への配置

ステップ 4 まで終わると `dufomap-out/static.pcd` + `occupancy.pgm` +
`occupancy.yaml` が手元にある。これを **登録 registry** である
`docs/maps/<site>/` に正式配置する。

### 手順

```bash
# 1. site directory を _template から複製
cp -r docs/maps/_template docs/maps/<site>
# 例: docs/maps/lab-loop, docs/maps/utsunomiya-yoto-east

# 2. 成果物を配置 (PCD は gitignored、pgm/yaml/metadata は tracked)
cp docs/m5r-bench-data/<run-id>/dufomap-out/static.pcd docs/maps/<site>/
mv path/to/occupancy.pgm  docs/maps/<site>/
mv path/to/occupancy.yaml docs/maps/<site>/

# 3. occupancy.pgm を出力先と同じ dir に置く前提なので、
#    再生成する場合は m5r_pcd_to_occupancy.py の <output-dir> に直接
#    docs/maps/<site>/ を渡す方が手数が少ない
scripts/m5r_pcd_to_occupancy.py \
  docs/maps/<site>/static.pcd \
  docs/maps/<site>/ \
  --force

# 4. metadata.yaml を埋める (次節参照)
${EDITOR:-vi} docs/maps/<site>/metadata.yaml
```

### `metadata.yaml` 記入要領

スキーマは [`../../maps/README.md`](../../maps/README.md) §「`metadata.yaml`
スキーマ」が正本 (ADR-0005 [`decisions/0005-maps-spec.md`](decisions/0005-maps-spec.md))。
M5-R パイプライン由来の値は次の手順で埋める:

| フィールド | 値の取り方 |
|---|---|
| `acquired_at` | `ros2 bag info <bag>` の Start を ISO8601 + タイムゾーンで転記 |
| `route_summary` | 一行で経路を説明 (例: `"lab 50m loop, start/end at NE corner of room 2F"`) |
| `weather` | 屋内は `"indoor"`、屋外は天候と気温を一行で |
| `slam_method` | `glim` 固定 (ADR-0003 確定) |
| `slam_params` | 既定設定なら省略可。カスタム config を使った場合のみ `docs/m5r-bench-data/<run-id>/glim-out/config/` への相対パス |
| `erasor_params` | DUFOMap パラメータを inline (`resolution`, `d_s`, `d_p` を非デフォルト時のみ) |
| `source_bag` | `docs/m5r-bench-data/<run-id>/bag/` の repo 相対パス |
| `commit` | `git rev-parse HEAD` の出力 (40 文字 SHA) |

`erasor_params` キー名は ADR-0005 起案時の歴史的事情で残しているが、
中身は ADR-0004 で確定した **DUFOMap パラメータ** を入れる。

### 配置後の確認

```bash
# Nav2 map_server で読めることを確認 (任意)
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=docs/maps/<site>/occupancy.yaml
# 別ターミナル
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo /map --once --field info
# resolution / width / height / origin が yaml と一致すれば OK
```

RViz での目視は [M5R-6 検証手順](#占有格子変換-nav2-互換) と同じ
(Fixed Frame = `map`、Map display の Durability Policy = Transient Local)。

## M6-R への引き渡し (B6 達成)

M5-R フェーズの**最終成果物 = M6-R の入力契約**:

```
docs/maps/<site>/
├── static.pcd        ← M6-R scan-to-map localizer の入力 (NDT/MCL 地図)
├── occupancy.pgm     ← Nav2 map_server / obstacle layer の入力
├── occupancy.yaml    ← 同上 (resolution / origin / thresh 規約)
├── metadata.yaml     ← 取得日 / 経路 / SLAM パラメータの監査用
└── README.md (任意)
```

M6-R 担当が前提にしてよいこと:
- `static.pcd` は world frame (GLIM map 座標)、動的物体除去済 (歩行者/自転車の
  軌跡が消えている)
- `occupancy.yaml` の `origin` は world frame の左下隅で、`static.pcd` と
  同じ座標系
- `static.pcd` 内の座標は GLIM が決めた map 原点基準 (= 走行開始姿勢
  ではない。`metadata.yaml.route_summary` で位置関係を述べる)

M6-R が前提にできないこと (本フェーズで未対応、別 Issue が必要):
- per-scan ray-cast による正確な occlusion (現状は anchor 1 点近似で
  星形アーチファクトあり、`scripts/m5r_pcd_to_occupancy.py` docstring 参照)
- 屋外連続性 (M5-R 検証は研究室内 + キャンパス 1 site 規模で完了、
  キャンパス全域マップは M6-R 着手と同時に再撮影する運用判断)

## ADR-0003 ステータス

ADR-0003 (SLAM 選定) は #48 で起案、Phase A〜B の比較計測完了後に
**accepted** 化済 ([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md))。
本 Issue 開始時点で既に `Status: accepted` のため追加承認手順なし。
将来 SLAM 選定を変更する場合は新規 ADR (例: ADR-00NN: SLAM 再選定) を
起案する (既存 ADR の改編ではなく追記が ADR 規約)。

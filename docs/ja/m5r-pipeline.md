# M5-R マップ作成パイプライン

Language: [日本語](m5r-pipeline.md) | [English](../en/m5r-pipeline.md)

本書は M5-R (マップ作成) パイプライン全体の参照ドキュメント。本ファイルは
Issue #49 で動的除去 (DUFOMap) section の skeleton として起こされ、
Issue #50 で占有格子変換 section を追加、Issue #51 (M5R-7 パイプライン統合)
で残る bag 取得 / SLAM 実行 / `docs/maps/<site>/` 配置などに拡張される予定。

現時点でカバーする範囲: 動的除去ステージ (DUFOMap) と占有格子変換ステージ
(`m5r_pcd_to_occupancy.py`)。bag 取得手順 / SLAM 実行 /
`metadata.yaml` 自動生成 / 最終配置などは #51 で追記する。

## E2E パイプライン (現状)

```
bag (M4-R bringup) -> GLIM (ADR-0003) -> DUFOMap (ADR-0004) -> 占有格子変換 (#50)
                       |                   |                     |
                       v                   v                     v
                glim-out/NNNNNN/    static.pcd            occupancy.pgm
                                                          occupancy.yaml
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

## 後続 (Issue #51 で記述予定)

- bag 取得手順 (M4-R bringup launch を立ててから何を録るか)
- SLAM 実行コマンド (採用 SLAM = GLIM の per-bag config パッチ手順)
- `metadata.yaml` 記入要領 (採用 SLAM / DUFOMap パラメータ / 取得日 /
  経路 / 天候、commit SHA 自動埋め)
- 最終成果物配置 (`docs/maps/<site>/...`)
- ADR-0003 を `accepted` 化するユーザー承認手順

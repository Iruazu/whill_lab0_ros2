# M5-R マップ作成パイプライン

Language: [日本語](m5r-pipeline.md) | [English](../en/m5r-pipeline.md)

本書は M5-R (マップ作成) パイプライン全体の参照ドキュメント。本ファイルは
Issue #49 で動的除去 (DUFOMap) section の skeleton として起こされ、
Issue #51 (M5R-7 パイプライン統合) で全体に拡張される。

現時点でカバーする範囲は動的除去ステージのみ。bag 取得手順 / SLAM 実行 /
占有格子変換 / `docs/maps/<site>/` 配置などは #51 で追記する。

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

## 後続 (Issue #51 で記述予定)

- bag 取得手順 (M4-R bringup launch を立ててから何を録るか)
- SLAM 実行コマンド (採用 SLAM = GLIM の per-bag config パッチ手順)
- 占有格子変換コマンド (M5R-6 / #50 の `scripts/m5r_pcd_to_occupancy.py`)
- `metadata.yaml` 記入要領 (採用 SLAM / DUFOMap パラメータ / 取得日 /
  経路 / 天候)
- 最終成果物配置 (`docs/maps/<site>/...`)
- ADR-0003 を `accepted` 化するユーザー承認手順

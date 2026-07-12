# `campus` (M5-R 本番マップ)

M5-R (`docs/ja/plans/2026-06-11-platform-pivot.md` §M5-R) の最終成果物。
2026-07-10 の 47 分 / 1310 m キャンパス外周走行 bag から GLIM +
DUFOMap 経由で生成した静的マップ。

## 概要

- **site 名**: `campus`
- **取得日**: 2026-07-10 (走行 15:07 JST 開始、2162 s / 36 分 2 秒)
- **経路**: 宇都宮大学キャンパス外周ループ 1310 m (start/end 同一地点、
  外周を反時計回り 1 周)
- **採用 SLAM**: GLIM (ADR-0003)
- **動的除去**: DUFOMap (ADR-0004)
- **占有格子生成**: `scripts/m5r_pcd_to_occupancy.py` — trajectory-anchor
  raycast + relative z-slice + r=2m 無条件 free (commit `7a9924a` の
  デフォルト設定で実行)

## 構成ファイル

| ファイル | 役割 | tracked? |
|---------|------|:--------:|
| `static.pcd` | DUFOMap 後の静的 PCD (13M 点、208 MB) | no (gitignored) |
| `occupancy.pgm` | 2D 占有格子 (6640 x 6295 @ 0.05 m/px) | yes |
| `occupancy.yaml` | Nav2 map_server メタデータ | yes |
| `metadata.yaml` | 取得経緯・パラメータ・出典 | yes |
| `traj_lidar.txt` | GLIM LiDAR トラジェクトリ (TUM 形式、21300 pose) | yes |
| `README.md` | 本ファイル | yes |

`traj_lidar.txt` は `m5r_pcd_to_occupancy.py` の relative z-slice + trajectory
anchor mode がランタイム入力として要求するため同梱する。ADR-0005 の
規約に厳密には含まれないが、地図の再現性 (この traj で切った slice) を
このディレクトリ単体で完結させる目的で tracked にする。

## 品質数値

観測可能な指標をすべて 1 箇所に集約する。

| 指標 | 値 | 出所 | 判定 |
|------|-----|------|------|
| loop length | 1310.098 m | `m5r3_loop_error.py` | — |
| end-to-start (trajectory) | **1.317 m (0.10%)** | 同 | ✅ ADR-0003 の 0.1-0.3% 基準の最良側 |
| per-axis (dx / dy / **dz**) | +0.107 / -0.161 / **+1.303** | 同 | dz は視認不可レベル |
| yaw drift | -0.16° | 同 | ✅ 数 m 級 yaw 起因 drift 懸念は不発 |
| **B1 数値代替** (地面 z 層 gap) | **1.394 m** | `m5r3_b1_numeric.py`、原点半径 5m 円柱 | ✅ traj dz と 7.0% 差 (+0.091 m) で独立一致 |
| B1 CloudCompare (壁 3 点平均) | 未実施 | — | 数値代替で代用 |
| GLIM 実行時間 | 691.8 s | `manifest.yaml` | bag 2162 s の 32% |
| Peak VRAM (GLIM) | 3297 MiB | 同 | Alienware x15 R2 で余裕 |
| starved anchor 率 (占有格子) | **0.0%** | `investigate_thin_corridor.py` | ✅ (前 commit a180c8b 版は 35.6%) |
| Occupied / Free / Unknown | 1.39% / **15.97%** / 82.64% | occupancy.pgm | 走行沿いだけが free、内側は unknown |
| 目視判定 (3 視点) | **PASS** | offline_viewer | 単線ループ、複製ゴーストなし、Z レイヤー化なし |

## 生成 commit 系譜

このマップと物理的に整合している commit を古い順に:

| commit | 内容 | 影響 |
|--------|------|------|
| `aed1e4d` | 07-09 base_link → imu_link 2nd remount 反映 | 走行前のマウント状態 |
| `d5c6eff` | CycloneDDS lo-only に `MaxAutoParticipantIndex=100` 追加 | bringup 11 ノード生存の前提 |
| `9cc4be2` | 07-10 pre-run base_link → imu_link 再測定 (roll -4.09° / pitch -8.11°) | 走行時の静的 TF chain |
| `39bf794` | GLIM audit T_lidar_imu を 07-10 pre-run 導出値に更新 | GLIM の T_lidar_imu |
| `4ca6704` | 07-10 本番マップ採用 + `m5r3_export_merged_ply.py` | manifest.yaml / audit doc / protocol doc の締め |
| `71b6407` | B1 数値代替 `m5r3_b1_numeric.py` 追加 + マップに適用 | B1 = 1.394 m 記録 |
| `a180c8b` | `m5r_pcd_to_occupancy.py` に trajectory anchor mode 導入 | 前段 occupancy grid |
| `7a9924a` | 相対 z-slice + `--anchor-free-radius` (最終版) | **本占有格子の実行 commit** |

再生成する場合は `a180c8b` 以降の script + `docs/maps/campus/traj_lidar.txt`
を組み合わせれば byte-identical な `occupancy.pgm` が出るはず。ただし
`static.pcd` は `bag` → GLIM → DUFOMap の再実行が必要 (元 bag は
`docs/m5r-bench-data/2026-07-10-campus-outer-final/` に、各パラメータは
`metadata.yaml` を参照)。

## 特記事項 (知っておくべき既知事項)

### 1. マップ全体が 1.81° 傾いている

`traj_lidar.txt` の (x, y, z) に対する平面フィット結果:

```
z = -0.0155·x + +0.0276·y + 1.815
plane tilt vs vertical: 1.81°  (steepest azimuth 119° from +x)
residual RMS = 1.32 m
z_span 総 9.79 m のうち  平面成分 (tilt) 7.10 m (72.5%)
                       残差 (地形) RMS 1.32 m
```

つまり **traj z が最大 9.4 m 変化しているように見えるが、その大半は
マップ側の tilt** (IMU gravity 校正の残留 or GLIM 初期方位の残留)。
実地形の起伏は残差 RMS 1.3 m 級。

**M6-R への影響**:
- Nav2 costmap の垂直面判定は world z axis が真上と仮定する。1.81° の
  ズレは costmap inflation 範囲に効く可能性がある
- Localizer の gravity-aware factor があると、bag マップの tilt vs
  実センサの gravity で不整合が起きうる
- **M6-R 開始前に「マップ側を de-tilt」 or 「localizer 側で許容」の
  どちらかを判断する必要あり**

Tilt の真の原因は 2 候補:
- **仮説 A**: IMU audit を実施した WHILL 静止位置 (start pose 付近) の
  路面が実際に 1.81° 傾いていた。ledger §7.10 の "base_link 水平性は
  水準器で確認したい" が未処理だったツケ
- **仮説 B**: GLIM の初期 gravity alignment が瞬間 IMU 値だけで走り、
  ズレが残った

どちらも「今日の bag を後から de-tilt する」ことでマップとしては
救えるが、根本対応は次回校正時のプロトコル改善 (calibration-ledger §0)
で潰す。

### 2. 中央部分 (キャンパス内側) は未走行 = unknown

`occupancy.pgm` の中央約 200 m x 200 m は灰色 (unknown)。今回の走行は
**外周ループのみ**で建物間の細道や中庭を通っていないため。Nav2 は
デフォルトで unknown を通行禁止として扱うので、当面は外周だけで
経路計画する運用になる。

**v2 走行 (2026-07 or 08) で補完予定**: 内側の細道・広場を通る
routing を追加し、`docs/maps/campus/` に merge or `docs/maps/campus-v2/`
として別 site を立てる。

### 3. Occupancy grid は relative z-slice + trajectory anchor + r=2m free

`scripts/m5r_pcd_to_occupancy.py` (commit `7a9924a`) のデフォルト設定
での生成物。従来の "fixed z-slice + single anchor" 版とは意味的に別物
なので、他マップとの比較時は注意:

- **fixed z-slice [0.1, 1.5]** で切ると、tilt のせいで 35.6% の anchor が
  starved (obstacle 観測ゼロ) になる。今回はこれを relative z-slice
  (traj_z 追従の local ground ± 0.7 m 帯) で解決
- **trajectory anchor** で「走行沿いだけを free 化」する設計。放射状縞
  アーティファクト + 未走行領域の誤 free が消える
- **--anchor-free-radius 2.0** で走行沿いに 2 m の絶対 free 保険を打つ
  (raycast が塗り忘れる隙間対応)

再現時に個別パラメータを覚える必要はない — script のデフォルトに固定
してある。

## 再生成手順

### 占有格子の再生成 (traj + static.pcd から、~43 s)

```bash
python3 scripts/m5r_pcd_to_occupancy.py \
  docs/maps/campus/static.pcd docs/maps/campus/ --force
```

`traj_lidar.txt` は同ディレクトリで auto-detect。default が
`--z-slice-mode relative --anchor-mode trajectory --anchor-free-radius 2.0`
なので追加引数は不要。

### 静的 PCD の再生成 (bag → GLIM → DUFOMap、~1 時間)

```bash
# 1. bringup (別ターミナル。sanity check は calibration-ledger §0 参照)
ros2 launch whill_localization odom_bringup_launch.py

# 2. bag は再走行が必要 (元 bag は 12.8 GiB)
scripts/m6r_record_calib_bag.sh docs/m5r-bench-data/2026-XX-XX-campus/

# 3. GLIM (audit T_lidar_imu 有効化)
GLIM_TLI_FROM_AUDIT=1 ./scripts/m5r3_run_glim.sh \
  docs/m5r-bench-data/2026-XX-XX-campus/bag \
  docs/m5r-bench-data/2026-XX-XX-campus/glim-out-audit-tli

# 4. DUFOMap
python3 scripts/m5r_glim_to_pcd.py \
  --glim-out docs/m5r-bench-data/2026-XX-XX-campus/glim-out-audit-tli \
  --output   docs/m5r-bench-data/2026-XX-XX-campus/dufomap-in/
./scripts/m5r_run_dufomap.sh \
  docs/m5r-bench-data/2026-XX-XX-campus/dufomap-in \
  docs/m5r-bench-data/2026-XX-XX-campus/dufomap-out

# 5. マップとして格納
cp docs/m5r-bench-data/2026-XX-XX-campus/dufomap-out/static.pcd docs/maps/campus/
cp docs/m5r-bench-data/2026-XX-XX-campus/glim-out-audit-tli/traj_lidar.txt docs/maps/campus/
```

## 関連

- 規約: [`../README.md`](../README.md)、[`../../decisions/0005-maps-spec.md`](../../decisions/0005-maps-spec.md)
- SLAM 選定: [`../../ja/decisions/0003-mapping-slam-choice.md`](../../ja/decisions/0003-mapping-slam-choice.md)
- 動的除去: [`../../ja/decisions/0004-dynamic-removal-choice.md`](../../ja/decisions/0004-dynamic-removal-choice.md)
- 校正記録: [`../../ja/calibration-ledger.md`](../../ja/calibration-ledger.md)
- IMU coordinate 検証: [`../../ja/imu-coordinate-audit.md`](../../ja/imu-coordinate-audit.md) §8
- 元 bag: `docs/m5r-bench-data/2026-07-10-campus-outer-final/`
- GLIM 出力: `docs/m5r-bench-data/2026-07-10-campus-outer-final/glim-out-audit-tli/`

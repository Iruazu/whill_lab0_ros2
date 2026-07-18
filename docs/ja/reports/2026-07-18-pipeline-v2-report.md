# v2 map 品質改善パイプライン 実装レポート

- 日付: 2026-07-18 (作業完了) — 2026-07-19 (レポート確定)
- 状態: PR #91 close-out 資料
- 対応 PR: [#91 feat(maps): PCD → Occupancy v2](https://github.com/Iruazu/whill_lab0_ros2/pull/91)
- 位置づけ: v1 (m5r_pcd_to_occupancy + clean_isolated_occupancy + GIMP) から v2 (layer-separated Stage 1 + composable Stage 2 + verifier) への根本再設計の記録

## 0. 何を作ったか (要約)

v1 パイプラインは salt 削減を最適化目標にしていた。ズーム検収の結果、raw occupied の 195,426 成分中 max=4 cells という「実障害物が孤立 dust に分解された」構造が判明し、post-hoc salt cleanup では治せないと判断。責務を分割:

- **機械**: recall (実在構造の取りこぼしゼロ) + 位置精度に責任
- **人間**: 真偽判定 (salt / real) に責任、sidecar mask で加筆

3 設計律を確定:

- **Rule 1**: 出力への形態学的操作の禁止 (幅と位置を歪める)
- **Rule 2**: 自動削除は最小限 (default: roadway 内 ≤3 cell 孤立のみ)
- **Rule 3**: 段差 occupancy は chair-accessible 側に配置 (curb は valley、ditch は upper に反転)

## 1. パイプライン構成

```
[Stage 0 = 既存] bag → GLIM → DUFOMap → static.pcd
                                             ↓
[Stage 1] pcd_to_occupancy_v2.py             ↓
     → occupied_step.png     (RGBA red, Rule 3 chair-accessible)
     → occupied_structure.png (RGBA orange, h∈[0.1, 2.2]m)
     → free_evidence.png     (RGBA green, traj disk + optional raycast)
     → salt_candidates.png   (RGBA color-coded, 1-3/4-16/17-64/65+ cells)
     → underlay_hillshade.png (RGB, ground_z ヒルシェード)
     → underlay_maxheight.png (RGB, viridis 最大高)
     → roadway_paint_guide.png (1/4 scale GIMP 参照)
     → v2_layers.yaml (manifest)
                                             ↓
[中間]   init_v2_sidecars.py → 空 sidecars 3 種
     GIMP で手塗り: keepout_mask / free_mask / roadway_mask
                                             ↓
[Stage 1.5] gen_auto_free_mask.py            ↓
     → free_mask_auto.png (roadway 内 ≤3 cell 孤立 → 白塗り)
     → salt_candidates_2to3.csv (監査用)
                                             ↓
[Stage 2] compose_occupancy.py               ↓
     合成規則 (2026-07-18 確定、以後変更禁止):
       conflict         = keepout ∩ free_mask                [warn]
       erased_by_free   = machine_occ ∩ free_mask_eff ∩ ¬keepout [audit]
       occupied_final   = keepout ∪ (machine_occ ∩ ¬free_mask_eff)
       free_final       = (free_mask_eff ∪ machine_free) ∩ ¬occupied_final
                          ∩ roadway_mask (fail-closed)
     優先順位: keepout > free_mask (human ∪ auto) > machine_occ > machine_free
                                             ↓
[出力]   final.pgm + final.yaml (Nav2 map_server 用)
         + audit: conflict.png / erased_by_free.png / clipped_by_roadway.png
                                             ↓
[検証]   pipeline_v2_verify.py               ↓
     → M1 断面照合 (±1 cell)
     → M2 縁石線連続率 (v2 ≥ v1)
     → M3 建物幅 (壁+庇、±1 cell)  ← 実測待ちで placeholder
                                             ↓
[監査]   audit_free_leaks.py                 ↓
     → (a)  15x15 UNKNOWN > 70% cell 数
     → (a') a & distance-to-OCC > 3m (真の interior leak)
     → (b)  traj 到達不能 free cell 数 (proxy、issue #96)
     → (c)  0.35m erode 後の FREE 断片数 (Nav2 planning 連結性)
```

## 2. 実装した重要決定

### 2.1 h フィルタ 2.2 m (walkable clearance)

user 決定: 「walkable clearance map (頭上 2.2m まで侵入可能空間)」を正とする。庇 2.0-2.2m は搭乗者頭部保護観点で実障害物として拾う。「建物幅」検証は壁+庇で測る。

### 2.2 Rule 3 chair-accessible side

curb は valley side (chair = road)、ditch は upper side (chair = road) と反転。実装は cv2.distanceTransform で footprint (traj + 2m disk) からの距離を negate、accessibility が高い側 = 近い側を occupied にマーク。tie は valley fallback (traj 情報なし時の安全 default)。campus route に ditch なしのため実データ検証不能 → 合成 unit test `scripts/tests/test_step_accessible.py` (4/4 PASS) で curb / ditch / no-traj fallback / sub-threshold をカバー。

### 2.3 raycast leak fix (P0)

初期実装は角度 720 bin dedup ray が疎な 1 px 壁の bin 間を素通り、建物内部 (点群欠損域) まで走行する事象を user 発見 (297,214 cells)。修正:

- `data_presence_2d = (raw_count > 0)` を追加、`--data-presence-dilate-cells 3` (15cm 半径) で拡張
- raycast に `unknown_stop_cells 3` 追加: data-void 3 セル連続で ray truncate + stop
- ray stopper を step ∪ structure union に (低段差通り抜け対処)

before/after (audit_free_leaks metric a'):
| variant | metric_a | metric_a' (>3m from OCC) |
|---|---|---|
| before | 297,214 | 1,668 (user 独立検算) |
| after | 206,711 | **7** |

metric_a の残存 206K は 97% が OCC 3m 以内 = 壁沿いの thin raycast ray artifact (window-based metric の false positive)、真の interior leak は 7 cells のみ。

### 2.4 fail-closed roadway_mask

Nav2 が承認回廊外に計画しない保証として、composer に第 3 sidecar `roadway_mask.png` (fail-closed FREE whitelist) を追加。`free_final ∩= roadway_mask`、未提供時は warn + 従来動作。clipping された cell は `clipped_by_roadway.png` (magenta) に記録 + cell 数 log。

### 2.5 auto free_mask ≤3 cell (Rule 2 適用)

roadway_mask は人間による路面承認 = 承認済路面上の ≤3 cell (≤15cm) 浮遊 blob は静的 lethal として保持する価値がない。実在なら Nav2 local costmap が live LiDAR で拾う (local obstacle layer の役割)、salt なら planning connectivity 破壊のみ。`gen_auto_free_mask.py` が roadway 内 ≤3 cell 孤立を `free_mask_auto.png` として出力、composer が erase (audit で色分け: 黄=人手 / シアン=自動 / 白=両方)。CSV に 2-3 cell blob 座標を audit 用に継続出力。

**効果** (synthetic roadway = traj+3m dilate, 実 roadway ではより控えめ):
| variant | fragments (0.35m erode 後) |
|---|---|
| before auto | 1,337 |
| after auto (≤1 cell) | 271 (-79.7%) |
| **after auto (≤3 cell)** | **42 (-96.9%)** |

## 3. 検証結果 (確定 final.pgm, OCC 435,489)

### 3.1 pipeline_v2_verify (M1/M2 公式値)

| M1 断面照合 (±1 cell = 5cm) | kind | verdict | Δ cells | Δ m | n_occ_band |
|---|---|---|---|---|---|
| curb_west_road_01 | curb | **PASS** | 0.00 | 0.000 | 63 |
| curb_west_road_02 | curb | **PASS** | 1.00 | 0.050 | 29 |
| curb_east_road_01 | curb | **PASS** | 1.00 | 0.050 | 38 |
| ditch_required_01 | ditch | N/A | — | — | — |

| M2 縁石線連続率 (v2 ≥ v1) | verdict | v2 ratio | v1 ratio | v2 gap | v1 gap | cells |
|---|---|---|---|---|---|---|
| curb_line_west_road_01 | **PASS** | 0.464 | 0.095 | 9 | 198 | 588 |

**OVERALL: ✅ all evaluated sections PASS** (M3 は CloudCompare 実測待ちで placeholder、issue #95)

### 3.2 audit_free_leaks (最終状態)

(user 実 roadway 適用後、確定 final.pgm)

| metric | value | 意味 |
|---|---|---|
| (a) 15x15 UNKNOWN>70% cell 数 | (user 実測) | 生 window metric |
| (a') refined (>3m from OCC) | (期待 0-数個) | 真の interior leak |
| (b) traj 到達不能 free proxy | 0 | 越境なし (proxy 実装、issue #96) |
| (c) 0.35m erode 後 fragments | (user 実測、目標 <200) | Nav2 planning 連結性 |

## 4. 残課題 (別 issue で追跡)

| # | 内容 | Gate |
|---|---|---|
| #92 | v1 archive (m5r_pcd_to_occupancy / clean_isolated_occupancy) | M1/M2/M3 全 real PASS |
| #93 | z-band redesign (hit-band と free-decision-band の分離) | M6-R 実機で挙動 observed |
| #94 | preview_composite.png Stage 1 追加 (閲覧俯瞰) | nice-to-have |
| #95 | M3 CloudCompare 実測 + cross_sections m3 real 座標記入 | CloudCompare 実行環境 |
| #96 | audit_free_leaks metric_b 原仕様実装 (step/structure-side crossing) | 中期 |
| #97 | Nav2 smoke checklist 1-5 + Phase 4 実機 bringup | PR #91 merge 後 |
| #98 | map_variant 正式配線 (occupancy_v2.* 暫定ブリッジ撤去) | Nav2 smoke 完了 + ADR |

## 5. 生成物 (docs/maps/campus/v2/)

| 種類 | ファイル | tracked? |
|---|---|---|
| 人間 sidecar (入力) | keepout_mask.png / free_mask.png / roadway_mask.png | yes |
| 機械 layer (Stage 1 出力) | occupied_step / structure / free_evidence / salt_candidates / underlay_hillshade / maxheight / roadway_paint_guide | yes (レビュー用) |
| 手動塗り参照 | v2-paint-guide.md | yes |
| 検証入力 | cross_sections.yaml | yes |
| 合成成果物 | final.pgm / final.yaml / final_rayon.* / final_rayoff.* | yes |
| 監査 (regenerable) | conflict.png / erased_by_free.png / clipped_by_roadway.png / free_mask_auto.png / salt_candidates_2to3.csv | **no (gitignored)** |
| キャッシュ | .v2_cache.npz | no (gitignored) |
| 暫定ブリッジ | ../occupancy_v2.pgm (symlink) / occupancy_v2.yaml | (issue #98 で撤去予定) |

## 6. コマンドリファレンス

```bash
# Stage 1: PCD → 6 レイヤ + manifest (~40s, cache hit ~10s)
scripts/pcd_to_occupancy_v2.py \
    --input-pcd  docs/maps/campus/static.pcd \
    --input-yaml docs/maps/campus/occupancy_cleaned.yaml \
    --output-dir docs/maps/campus/v2 \
    --cache-npz  docs/maps/campus/.v2_cache.npz

# Sidecar 空初期化 (既存を上書きしない)
scripts/init_v2_sidecars.py --layers-yaml docs/maps/campus/v2/v2_layers.yaml

# Roadway 塗りが終わった後: auto free_mask 生成 (~5s)
scripts/gen_auto_free_mask.py \
    --layers-yaml            docs/maps/campus/v2/v2_layers.yaml \
    --roadway-mask           docs/maps/campus/v2/roadway_mask.png \
    --output-free-mask-auto  docs/maps/campus/v2/free_mask_auto.png \
    --output-salt-csv        docs/maps/campus/v2/salt_candidates_2to3.csv

# Stage 2: 合成 (~2s)
scripts/compose_occupancy.py \
    --layers-yaml    docs/maps/campus/v2/v2_layers.yaml \
    --roadway-mask   docs/maps/campus/v2/roadway_mask.png \
    --free-mask      docs/maps/campus/v2/free_mask.png \
    --free-mask-auto docs/maps/campus/v2/free_mask_auto.png \
    --keepout-mask   docs/maps/campus/v2/keepout_mask.png \
    --output-pgm     docs/maps/campus/v2/final.pgm

# 検証 M1/M2 (~1s)
scripts/pipeline_v2_verify.py \
    --cross-sections docs/maps/campus/v2/cross_sections.yaml \
    --v1-pgm         docs/maps/campus/occupancy_cleaned.pgm \
    --markdown

# 監査 (~5s)
scripts/audit_free_leaks.py \
    --pgm-yaml docs/maps/campus/v2/final.yaml \
    --traj     docs/maps/campus/traj_lidar.txt

# Rule 3 合成単体テスト
python3 scripts/tests/test_step_accessible.py
```

## 7. 主要 commit

| SHA | 内容 |
|---|---|
| 1dd808c | Stage 1 初版 (レイヤ分離出力) |
| ed186d6 | 合成則反転 (keepout > free_mask > machine_occ) |
| db5b40a | Y-flip 修正 (20% → 92.5% match with cleaned.pgm) |
| 857d092 | pipeline_v2_verify.py + cross_sections schema |
| 688f8b4 | P0 raycast leak fix + M1/M2 tolerance + not_applicable |
| a41c781 | audit_free_leaks + roadway_mask fail-closed + A/B |
| 34f14dd | audit metric_b docstring 訂正 (proxy 明記) |
| 1917ff9 | gen_auto_free_mask.py + composer --free-mask-auto |
| 75b77a0 | auto free_mask 対象拡大 ≤1 → ≤3 cell |

## 8. 関連文書

- 作業手順: `docs/maps/v2-paint-guide.md`
- 上流パイプ (bag→PCD): `docs/maps/campus/README.md` §生成 commit 系譜
- 全体方針: `docs/ja/plans/2026-06-11-platform-pivot.md` §M5-R
- ADR: 0003 (GLIM) / 0004 (DUFOMap) / 0005 (maps/ 規約) / 0009 (free ≠ traversable)
- PR 履歴: PR #90 (trav 継承元) / **PR #91 (本 v2 実装)**

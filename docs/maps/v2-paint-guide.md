# v2 手塗り作業手順 (GIMP)

`docs/maps/campus/v2/` の Stage 1 出力を GIMP で開き、`keepout_mask.png` / `free_mask.png` の 2 sidecar を作成するためのワークフロー。

## 前提

- Stage 1 は既に走行済 (`v2_layers.yaml` が存在)。
- 合成規則: **keepout > free_mask > machine_occ > machine_free** (2026-07-18 確定、以後変更禁止)。
  - `free_mask` は machine_occ を消去できる → 路面 salt 除去に使う。
  - `keepout` は全てに勝つ → 機械が拾い漏れた障害物 (透明ポール、低段差等) の追加に使う。

## 手順

### 1. 空 sidecar 生成 (再走行 safe)

```bash
scripts/init_v2_sidecars.py --layers-yaml docs/maps/campus/v2/v2_layers.yaml
```

既存 `keepout_mask.png` / `free_mask.png` があれば **上書きせずスキップ** (作業中の消失回避)。強制上書きは `--force`。

### 2. GIMP でレイヤ構成

`docs/maps/campus/occupancy_cleaned.pgm` (下敷きとして) を新規イメージで開いてから、`docs/maps/campus/v2/` の PNG を **「レイヤとして開く」** で順に追加。**下から上の順**:

| 順 | ファイル | 種類 | 役割 |
|---|---|---|---|
| 1 | `occupancy_cleaned.pgm` | 参考 (grayscale) | 既存 v1 map。位置合わせの物差し |
| 2 | `underlay_hillshade.png` | RGB | 陰影で段差線の場所を目視判定 |
| 3 | `underlay_maxheight.png` | RGB viridis | 柱・壁の高さ判別 |
| 4 | `free_evidence.png` | RGBA semi-tr green | 走行実績帯 + raycast の見えた領域 |
| 5 | `occupied_structure.png` | RGBA opaque orange | 相対高 [0.1, 2.2]m の障害物 |
| 6 | `occupied_step.png` | RGBA opaque red | Rule 3 chair-accessible 段差 |
| 7 | `salt_candidates.png` | RGBA color-coded | **手塗り優先度ガイド** (下記) |
| 8 | `keepout_mask.png` | RGBA (空) | **人が加える occupied** — 白で塗る |
| 9 | `free_mask.png` | RGBA (空) | **人が加える free / 機械 salt 消去** — 白で塗る |

### 3. salt_candidates.png 色の読み方

| 色 | cluster size | 意味 | 推奨アクション |
|---|---|---|---|
| マゼンタ | 1-3 cells | 即消去候補 | `free_mask` に塗り足す |
| 赤 | 4-16 cells | salt 可能性大 | hillshade / maxheight で真偽判定 → 大半は `free_mask` |
| オレンジ | 17-64 cells | 判断分かれる | hillshade で段差線があれば残す |
| 緑 | 65+ cells | ほぼ確実に real | 触らない (`free_mask` に含めない) |

Campus 実測: マゼンタ=0 / 赤=84K / オレンジ=66K / **緑=313K (67%)** — 大半は放置で OK、赤+オレンジ 15 万 cell に集中的に判断を投入。

### 4. 塗り作業

- **塗り色**: **純白 (255, 255, 255)** 一択。`--mask-threshold=128` (default) を確実通過する。純赤 (L=76) や青 (L=29) は閾値未満で無視される。
- **透明維持**: `keepout_mask.png` / `free_mask.png` はレイヤ全体を透明のままに保ち、塗った部分だけ不透明にする。GIMP の「透明を保持」オプション OFF で塗り、白ブラシで直接ペイント。
- **ブラシサイズ**: 縁石など細い線には 5-10 px、路面 salt ブロブには 15-30 px 程度。

### 5. Audit ゲート確認 (最初の 1 回)

以下 3 pixels は明らかに `machine_occ` (salt_candidates 緑バンド、traj 至近):

| # | pixel (x, y) | map (m) | 位置 |
|---|---|---|---|
| 1 | (1938, 1266) | (-31.350, 161.050) | 西寄り (traj 距離 1.25 m) |
| 2 | (4298, 1210) | (86.650, 163.850) | 中央 (traj 距離 0.15 m) |
| 3 | (4788, 1216) | (111.150, 163.550) | 東寄り (traj 距離 0.00 m) |

**Nav2 座標慣習**: pixel(x, y) は PNG 画像座標 (原点=左上、y は下向き)。map(x, y) は Nav2 map frame (原点=左下、y は上向き)。関係式:
- `map_x = origin_x + pixel_x * resolution`
- `map_y = origin_y + (H - 1 - pixel_y) * resolution`

**確認手順**:
1. `free_mask.png` に上記 3 pixels を白ブラシ (r=3-5 px) で塗る
2. export
3. `scripts/compose_occupancy.py --layers-yaml docs/maps/campus/v2/v2_layers.yaml --free-mask docs/maps/campus/v2/free_mask.png --output-pgm /tmp/audit.pgm`
4. **成功条件**: 出力の `free_mask erased occ` が 3+ 個 (ブラシ半径分), `erased_by_free.png` が生成される。
5. 塗った 3 dot を消してから本番作業に入る。

### 6. Export 手順 (厳守)

- **File → Export As** で対象の `keepout_mask.png` / `free_mask.png` を上書き
- **フォーマット PNG**、**Interlacing なし**、**RGBA を保持**
- **キャンバスサイズ**: **6640×6295 厳守** (`v2_layers.yaml` の grid 数値と一致)。スケーリング・クロップ厳禁。
- レイヤ結合が必要な場合: File → Export As は自動で見える layer を merge 出力するので、対象 sidecar 以外を非表示にしてから export。

### 7. 合成 → Nav2 用 pgm 生成

```bash
scripts/compose_occupancy.py \
    --layers-yaml  docs/maps/campus/v2/v2_layers.yaml \
    --keepout-mask docs/maps/campus/v2/keepout_mask.png \
    --free-mask    docs/maps/campus/v2/free_mask.png \
    --output-pgm   docs/maps/campus/v2/final.pgm
```

出力:
- `final.pgm` + `final.yaml` — Nav2 map_server 用
- `conflict.png` (keepout ∩ free_mask、非ゼロ時のみ) — 意図衝突の警告
- `erased_by_free.png` (機械 occ が人手 free で消えた cells、非ゼロ時のみ) — audit trail

### 8. 何度でも回せる

Stage 1 (`pcd_to_occupancy_v2.py`) を再走行しても sidecar は保存されるので、`compose_occupancy.py` を再実行するだけで新しい機械側 evidence + 既存人手指定の合成 pgm が得られる。作業は蒸発しない。

## トラブルシューティング

- **shape mismatch エラー**: sidecar PNG のピクセル寸法が Stage 1 grid と違う。GIMP export 時にスケーリングされた or 別サイズで新規作成した可能性。`v2_layers.yaml` の grid.width / grid.height と一致必須。
- **塗ったのに反映されない**: 塗り色が閾値未満 (`--mask-threshold=128`)。純白で塗り直す。または、RGBA モードで塗った pixel が全透明 (alpha=0)。GIMP の「不透明度」を 100% にする。
- **erased_by_free.png が出ない**: free_mask が機械 occupied にヒットしていない。GIMP で `machine_occ` レイヤ (red or orange) の直上を塗る必要あり。
- **conflict.png が大量生成**: keepout と free_mask 両方に同じ領域を塗っている。優先順位は keepout 勝ちなので free_mask 側を削って再 export。

# ADR 0003: M5-R マップ作成 SLAM の最終選定

Language: [日本語](0003-mapping-slam-choice.md) | [English](../../en/decisions/0003-mapping-slam-choice.md)

- Status: proposed (Phase B データ収集後にユーザー承認 → accepted)
- Date: 2026-06-22
- Deciders: Iruazu (Phase B 完了後の承認待ち)

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

```
(PLACEHOLDER) Phase B 完了後に埋める。

採用 SLAM: TBD (GLIM | FAST_LIO_SAM)
Commit SHA / Tag pin: TBD (採用 SLAM の manifest.yaml の git_commit と
                           上流 upstream_commit を転記)
判断根拠サマリ: TBD (B1 誤差、ループクロージャの有無、ライセンス、操作性の
                4 軸での総合判断)
```

本節は Phase B のデータ収集 + 評価担当者の判断後、別 commit で埋めて ユーザー承認のもと accepted 化する。

## 採用しなかった案

Phase B 完了時、本節に以下のテーブルを埋める。**Alternatives = 採用しなかった候補** なので、最終的に採用された側はこの表から除き、Decision 節に書く。

### 比較テーブル (Phase B で埋める)

| 軸 | GLIM | FAST-LIO SAM |
|---|---|---|
| 走行時間 (s) | TBD | TBD |
| ピーク VRAM (MiB) | TBD | TBD |
| ピーク RSS (KiB) | n/a (manifest スキーマで未計測) | TBD |
| trajectory 内部誤差 (m) | TBD | TBD |
| B1 公式誤差 (壁面 3 点平均、m) | TBD | TBD |
| ループクロージャの発火タイミング | TBD (例: 走行 80% 地点で 1 回発火) | TBD |
| keyframe 発行密度 (枚 / m) | TBD | TBD |
| manual relocalization 要否 | TBD | TBD |
| GTSAM 解決状況 | n/a (4.3a0 単独) | TBD (4.1.1 単独 / 共存警告あり / `LD_LIBRARY_PATH` 強制が必要) |
| ライセンス | MIT | LICENSE 不在 + GPL-2.0 伝播可能性 |
| build 成否 | OK (M5R-1 確認済) | TBD (上流 "Full ROS2 adaptation" TODO 残あり) |

### 補足ノート (Phase B で埋める)

- (rejected SLAM 側について) 何が決定打になったか
- li_slam_ros2 を本 ADR で評価対象外とした理由: 親方針 §3.3 で「比較・つなぎ用」と明記。GLIM vs FAST-LIO SAM が代表選定であり、li_slam_ros2 は本 ADR の比較対象外。採用 SLAM が両者とも不適格と判明した場合のみ別 ADR で再検討する
- (オプション) 「Velodyne 専用 config が GLIM 上流に ship されていなかったため Ouster 用 config で走らせた」など、比較条件の対称性に影響した事実を列挙

## 結果

Phase B 完了時に埋める。以下の構造で書く。

### ライセンス棚卸し (B5 達成)

採用 SLAM ごとに本リポへの組み込み形態 / 運用スタックへの link 制約を明示する:

- **GLIM を採用した場合**: MIT、permissive。運用スタックへの link 制約なし。ただし M5-R は「オフラインのマップ作成ツール」フェーズとして位置付け、ランタイム localizer は M6-R の scan-to-map localizer (別選定) が担当する。GLIM 自体を運用スタックに組み込み直す判断は本 ADR では行わず、M6-R の評価結果次第とする
- **FAST-LIO SAM を採用した場合**: LICENSE 不在 = 著作権法上は事実上 "all rights reserved"、派生元 FAST-LIO (HKU-MaRS) は GPL-2.0 で copyleft 伝播可能性。親方針 §3.4 「GPL 系は『オフラインのマップ作成ツール』としての分離プロセス利用に限定」を適用する:
  - `whill_lab.repos` への組み込み: **不可** (clone-on-demand 維持)
  - 運用パッケージへの link: **不可**
  - 生成 PCD / 占有格子のみ `docs/maps/<site>/` に格納する: 可 (上流コードの再配布ではなく評価出力データ)
  - 上流 LICENSE が将来追加されて permissive になった場合の運用切替は別 ADR で扱う

### CPU / GPU / メモリ要件

採用 SLAM の母艦 (Alienware x15 R2、RTX 3080 Laptop GPU 16 GB VRAM、i9-12900H 32 GiB RAM) での実測値を記録する。車載機への移行可否は本 ADR では判断せず、M9 (車載分離) で再評価する。

### 後続フェーズ (M6-R) への影響

- M6-R の scan-to-map localizer は採用 SLAM が出した静的 PCD を `docs/maps/<site>/static.pcd` 規約に従って入力前提とする (ADR-0005)
- PCD フォーマット (binary vs ascii、座標系、座標精度) の互換性確認結果を本節に記録
- coordinate frame の整合性: M4-R bringup `/tf_static` の `base_link → velodyne` extrinsic が bag に乗っているため、生成 PCD が `velodyne` frame で出力されるか `base_link` frame で出力されるかを記録し、M6-R localizer 側の前提と突合する

### 後続作業

- **M5R-4 (#49) ERASOR**: 採用 SLAM の出力 (PCD + per-frame poses) を入力に、動的物体除去。本 ADR の Decision が確定するまでは M5R-4 着手不可
- **M5R-6 (#50) 占有格子変換**: 採用 SLAM 経由の ERASOR 後 PCD を 2D 占有格子に変換、`docs/maps/<site>/occupancy.{pgm,yaml}` に格納
- **M5R-7 (#51) パイプライン統合**: bag → 採用 SLAM → ERASOR → 占有格子 → `docs/maps/<site>/` の E2E 文書化
- **本 ADR の `proposed → accepted` 化**: Decision 節を埋めた別 commit でユーザー承認を取り、Status 行を書き換える

## 関連

- 親方針: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §3.3 (採用候補)、§3.4 (ライセンス方針)、§7 (本 ADR の起案要請)
- M5-R 実行計画: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md) §M5R-3 (本 Issue)、§6 (受け入れ基準 B1〜B5)
- 比較プロトコル: [`../m5r3-comparison-protocol.md`](../m5r3-comparison-protocol.md) — Phase B 実行手順書
- 前置 ADR: [`0005-maps-spec.md`](0005-maps-spec.md) — 採用 SLAM の出力先 `docs/maps/<site>/` の規約
- 前置文書: [`../m5r-glim-setup.md`](../m5r-glim-setup.md) (GLIM source build)、[`../m5r-fastlio-sam-eval.md`](../m5r-fastlio-sam-eval.md) (FAST-LIO SAM clone-on-demand)
- スクリプト: [`../../../scripts/m5r3_run_glim.sh`](../../../scripts/m5r3_run_glim.sh)、[`../../../scripts/m5r3_run_fastlio_sam.sh`](../../../scripts/m5r3_run_fastlio_sam.sh)、[`../../../scripts/m5r3_loop_error.py`](../../../scripts/m5r3_loop_error.py)
- 関連 Issue: #48 (本 Issue、M5R-3)、#45 (M5R-1 GLIM)、#46 (M5R-2 FAST-LIO SAM)、#47 (M5R-5 maps 規約)、#49 (M5R-4 ERASOR)、#50 (M5R-6 占有格子)、#51 (M5R-7 統合)

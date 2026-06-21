# ADR 0004: M5-R 動的物体除去ツールの選定

Language: [日本語](0004-dynamic-removal-choice.md) | [English](../../en/decisions/0004-dynamic-removal-choice.md)

- Status: accepted
- Date: 2026-06-21
- Deciders: Iruazu

## 背景

親方針 [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
§3.3 (採用候補表) は動的物体除去について次のように定めていた:

> | 役割 | 第一候補 | 代替 | 理由 |
> |------|---------|------|------|
> | 動的物体除去 | ERASOR 系 | Removert | 高速・静的点の保全。オフライン処理なので車載要件なし |

M5-R 実行計画 [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
§Issue M5R-4 もこれを継承し、ERASOR を「`Apache-2.0`」のライセンスで採用する
前提で記述している。本 ADR はこの前提が崩れたことを記録し、置き換える。

### 前提が崩れた経緯

Issue #49 (本 Issue) 着手時に DUFOMap が候補に上がり、各候補のライセンスと
maintenance 状況を `gh api` で上流リポを直接確認した結果、以下が判明した:

| 候補 | 上流 | ライセンス | maintenance | カスタムデータ対応 |
|---|---|---|---|---|
| ERASOR v1 | [`LimHyungTae/ERASOR`](https://github.com/LimHyungTae/ERASOR) | **GPL-3.0** (Issue #49 本文の「Apache-2.0」は誤り) | ROS 1 melodic 専用、active maintenance なし | ROS 1 stack を立て直さない限り不可 |
| ERASOR2 | [`url-kaist/ERASOR2`](https://github.com/url-kaist/ERASOR2) | **GPL-3.0** | active | SemanticKITTI 形式入力専用、カスタムデータの adapter なし |
| Removert | [`irapkaist/removert`](https://github.com/irapkaist/removert) | **LICENSE ファイルなし** (= 採用不可。明示ライセンスがない code を本リポへ依存させると配布リスク) | ROS 1 noetic 中心 | ROS 1 stack 前提 |
| GLIM 内蔵 dynamic_remover | [`koide3/glim_ext`](https://github.com/koide3/glim_ext) | MIT | active | — | 公開モジュール一覧に「dynamic removal」相当が**存在しない** (確認: 2026-06-21) |
| DUFOMap | [`KTH-RPL/dufomap`](https://github.com/KTH-RPL/dufomap) | **BSD-3-Clause** | active (2024 年以降も commit あり) | Python API `pip install dufomap` で導入可、ROS 非依存、Ubuntu 22.04 + Python 3.10 で動作確認 |

ライセンス軸: 親方針 §3.4 は「permissive (MIT/BSD/Apache) で構成可能な
状態を保つ。GPL 系は『オフラインのマップ作成ツール』としての分離プロセス
利用に限定する」と定めている。動的物体除去は親方針上「オフラインの
マップ作成ツール」フェーズに属するため GPL も理論上は許容範囲だが、本 ADR
ではより保守的に permissive を優先する。理由は (a) M6-R で運用スタックに
何が漏れ出すか未確定で、permissive で揃えておけば後発の判断が楽、
(b) `src/third_party/` の上流リスト (`whill_lab.repos`) が permissive で
統一されており、GPL を 1 件入れた瞬間に企業提供時のライセンス棚卸しで例外
処理が要る、の 2 点。

技術軸: ERASOR v1 / Removert は ROS 1 のみで、ROS 2 humble 環境では使えない。
ERASOR2 は ROS 2 移植版だが SemanticKITTI 形式専用で本リポの GLIM 出力
(keyframe dir) を直接食えない。DUFOMap は ROS 非依存の Python API なので、
GLIM 出力 → PCD 変換のみで橋渡しできる (本 Issue で converter 実装)。

## 決定

**採用: DUFOMap** (`KTH-RPL/dufomap`、BSD-3-Clause、`pip install dufomap`)。

実装ブリッジ (本 Issue で新規):

- `scripts/m5r_glim_to_pcd.py` — GLIM keyframe dir (`NNNNNN/points_compact.bin`
  + `data.txt` の `T_world_origin`) → DUFOMap 入力用 per-keyframe PCD
  (`VIEWPOINT` ヘッダ付き)
- `scripts/m5r_run_dufomap_core.py` — DUFOMap Python API ラッパ
- `scripts/m5r_run_dufomap.sh` — 上記 2 つを 1 コマンドで回す orchestrator
- `scripts/m5r_dufomap_diff.py` — 除去前後の PCD 重ね表示 (目視確認用)

DUFOMap パラメータの既定値は上流 `KTH-RPL/dufomap/assets/config.toml`
そのまま (`resolution=0.1`, `inflate_hits_dist=0.2`, `inflate_unknown=2`)。
チューニング指針は [`../m5r-pipeline.md`](../m5r-pipeline.md) を参照。

本 ADR は親方針 §3.3 の選定表のうち「動的物体除去」行を上書きする。
親方針本体の書き換えは行わず、本 ADR を参照する形で運用する (親方針は
複数 ADR で参照されており、frozen 化のコストが大きいため)。

## 採用しなかった案

### ERASOR v1 (`LimHyungTae/ERASOR`)

- ライセンス: GPL-3.0 (Issue #49 本文の Apache-2.0 は誤記)
- ROS 1 melodic 専用、active maintenance なし
- 採用見送り理由: 本リポは ROS 2 humble、ROS 1 stack を別途立てる工数を
  払う価値がない。GPL も親方針 §3.4 に照らして例外処理が要る

### ERASOR2 (`url-kaist/ERASOR2`)

- ライセンス: GPL-3.0
- 入力フォーマット: SemanticKITTI 専用 (label + velodyne_bin の組)
- 採用見送り理由: GLIM の keyframe dir を SemanticKITTI 形式に変換する
  adapter を書く工数 (semantic label の生成が必要、未解決問題) が、
  DUFOMap への変換 (純粋に PCD ヘッダを書くだけ) に比べて重い。
  さらに GPL-3.0 で permissive 方針に劣る

### Removert (`irapkaist/removert`)

- ライセンス: **LICENSE ファイルなし** (上流リポに LICENSE / COPYING /
  README 内の明示なし)
- 採用見送り理由: 明示ライセンスのない code は本リポの BSD-3-Clause 方針
  および将来の企業提供時に判断不能。技術的良し悪し以前で除外

### GLIM 内蔵 dynamic_remover

- 上流: `koide3/glim_ext` の公開モジュール一覧に「dynamic removal」相当が
  存在しない (2026-06-21 時点で `gh api repos/koide3/glim_ext/contents`
  を確認)
- 採用見送り理由: そもそも該当機能が存在しない。検討対象として残しておくと
  M6-R 以降に「GLIM 側で済むかも」が亡霊として残るため、本 ADR で明示的に
  「無い」と記録する

### 自前実装

- 採用見送り理由: 動的物体除去は研究領域として独立した課題で、self-rolling
  に十分な benchmark + paper の蓄積がある。本 Issue のスコープ (M5-R
  パイプライン整備) と無関係に時間が溶ける

## 結果

得るもの:

- BSD-3-Clause 1 件追加のみで親方針 §3.4 の permissive 方針が崩れない
- `pip install dufomap` で導入完結、母艦の Ubuntu 22.04 + Python 3.10 で
  動作確認可 (本 Issue で converter まで実装完了。DUFOMap 実行自体は実機
  検証としてユーザーに引き渡し)
- ROS 非依存なので車載スタックに動的除去のランタイムコードが混入する
  リスクがない (DUFOMap はオフラインのマップ作成段でのみ使う)

失うもの (あるいはコスト):

- GLIM keyframe dir → per-scan PCD の変換層が新規に必要 (DUFOMap は
  per-scan PCD + VIEWPOINT ヘッダを入力前提とするため。本 Issue で
  `scripts/m5r_glim_to_pcd.py` を実装)
- 親方針 §3.3 の選定表とのずれが残る (本 ADR で上書きする方針を採るが、
  ADR を読まずに親方針だけ見る agent は混乱する可能性。CLAUDE.md の
  「既知課題」更新は本 Issue スコープ外)

後続作業:

- M5R-4 (本 Issue #49): DUFOMap 実機 run + 静的 PCD 取得 + 目視確認
  (実機検証はユーザー作業)
- M5R-6 (#50): DUFOMap 出力 `static.pcd` を入力に占有格子変換
- M5R-7 (#51): bag → GLIM → DUFOMap → 占有格子の E2E パイプライン文書化。
  本 ADR の参照と、`docs/maps/<site>/metadata.yaml` の `dufomap_params`
  フィールド (ADR-0005 の `erasor_params` 任意フィールドを rename / 兼用)
  も #51 で確定する
- Issue 本文 (#49) の側: スクリプト名 (`m5r_run_erasor.sh` →
  `m5r_run_dufomap.sh`、`m5r_erasor_diff.py` → `m5r_dufomap_diff.py`)
  は本 PR では rename を反映するのみ。Issue 本文の書き換えは別途

## 関連

- 親方針 (上書き対象): [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §3.3 (採用候補表の「動的物体除去」行)、§3.4 (ライセンス方針)
- M5-R 実行計画: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §M5R-4 (Issue #49)、§11 (ADR-0004 候補として明記)
- 前置 ADR: [`0003-mapping-slam-choice.md`](0003-mapping-slam-choice.md)
  (採用 SLAM = GLIM、本 ADR の入力フォーマットを規定)、
  [`0005-maps-spec.md`](0005-maps-spec.md) (`docs/maps/<site>/` 規約、
  本 ADR の出力先 `static.pcd` を規定)
- 関連 Issue: #49 (本 ADR の起点)、#50 (M5R-6 占有格子)、#51 (M5R-7 統合)
- パイプライン文書: [`../m5r-pipeline.md`](../m5r-pipeline.md)
- DUFOMap 上流: [KTH-RPL/dufomap](https://github.com/KTH-RPL/dufomap)
  (BSD-3-Clause, ROS 非依存 Python API)

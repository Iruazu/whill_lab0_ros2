# M5-R 前置: FAST-LIO SAM 評価候補化 (clone-on-demand 運用)

Language: [日本語](m5r-fastlio-sam-eval.md) | [English](../en/m5r-fastlio-sam-eval.md)

## 目的

M5-R (オフラインのマップ作成パイプライン) の SLAM 候補のうち、第二候補に位置づけている [FAST-LIO SAM (RightTr fork)](https://github.com/RightTr/FAST-LIO-SAM) を、後続の M5R-3 (Issue #48「GLIM vs FAST-LIO SAM 実 bag 比較」) で評価できる状態まで整える。具体的には:

- 上流ライセンス状況を確定し、本リポへの取り込み方針を決める
- ローカル評価用の clone 手順 + Ubuntu 22.04 用 GTSAM (PPA) インストール手順を文書化する
- 評価担当者が再現できる冪等スクリプトを `scripts/clone_fastlio_sam_for_eval.sh` に用意する

本文書の到達点は「M5R-3 でビルドと smoke 試験に入れる」までであり、**実 bag での評価・パラメータ調整・ADR-0003 (SLAM 候補確定) の起案は M5R-3 のスコープ**である。

採択経緯は [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md) §3.1 (`src/third_party/FAST_LIO_SAM/` の扱い) を参照。

## ライセンス状況 (最重要)

| 観点 | 事実 |
|---|---|
| 上流リポ | `https://github.com/RightTr/FAST-LIO-SAM` |
| LICENSE ファイル | **存在しない** (`find -iname LICENSE*` で 0 件、WebFetch で本文確認) |
| README の license 宣言 | なし、license バッジもなし |
| `package.xml` の `<license>` | `BSD` と書かれているが、対応する LICENSE 本文が同梱されていない (= 法的拘束力のある合意は不在) |
| 派生元 | HKU-MaRS の [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (**GPL-2.0**)。FAST-LIO SAM はそのコードを取り込んで GTSAM の loop closure / smoothing を追加した派生物。GPL の copyleft 規定により派生物にも GPL-2.0 が伝播し得る |

### 著作権法上の解釈

明示的なライセンス付与がない場合、著作権者の許諾なしには使用・複製・改変・再配布できない。すなわち**事実上 "all rights reserved"** であり、permissive な BSD/MIT より厳しく、GPL-2.0 より不確実な状態に分類される (GPL-2.0 は少なくとも条件下での再配布を許容する)。`package.xml` に `BSD` と書かれているのは upstream の自己申告に過ぎず、対応する LICENSE 文書がない以上、リポジトリ全体に BSD が及ぶと第三者が解釈する根拠としては弱い。

### 本リポでの扱い

親方針 [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.4 (ライセンス方針):

> GPL 系 (FAST-LIO ファミリ等) は「オフラインのマップ作成ツール」としての分離プロセス利用に限定する

を適用する。FAST-LIO SAM はこの「FAST-LIO ファミリ」に該当し、かつ上流 LICENSE 不在で許容範囲はさらに狭まる。本リポでの扱いを次のように決定する:

| 行為 | 可否 | 根拠 |
|---|---|---|
| ローカル clone してマップ作成専用に評価する | 可 (本文書の手順がそれ) | 親方針 §3.4 「オフラインのマップ作成ツールとしての分離プロセス利用」 |
| 評価結果として生成された **静的 PCD / 占有格子のみ** を `docs/maps/<site>/` に格納する | 可 | 上流コードのコピー・改変・再配布ではなく、評価対象 bag に対する**出力データ**なので著作物性は別建て |
| `whill_lab.repos` に正式エントリとして追加する | **不可** | `vcs import` 実行者全員が暗黙に upstream のコピーを保持することになり、license 不確定状態での再配布相当の挙動になる |
| 運用パッケージ (`whill_navigation`、`whill_localization` 等) から link する | **不可** | 本リポ運用スタックの permissive 維持と矛盾。FAST-LIO 派生物の GPL 伝播リスクが残る |
| 上流リポへの fork / 修正 PR | **本 Issue では不可** | スコープ外 (本 Issue は本リポ内のみ)。将来必要なら ADR 必須 |

### なぜ `whill_lab.repos` ではなく clone-on-demand か

`whill_lab.repos` 経路:

- 本リポを clone した全員が `vcs import` で upstream を自動取得する設計
- license 状況が不確定な upstream をその対象に含めると、本リポの clone 自体が暗黙に license 不明コードの再配布起点になり得る
- M5-R 完了後に「FAST-LIO SAM は使わない」と決まった場合、`whill_lab.repos` から外す手間と、その間に各人の作業ツリーに残った clone をどう扱うかの議論が残る

clone-on-demand 経路 (本文書の採択):

- 評価担当者が自分の判断で `scripts/clone_fastlio_sam_for_eval.sh` を実行する形になる。license リスクの引受は実行者の意思決定として明示される
- 環境変数 `FASTLIO_SAM_LICENSE_ACK=yes` を要求する誤実行ガードをスクリプトに入れて、無意識の clone を防ぐ
- M5R-3 で ADR-0003 を accept した時点で、改めて (a) `whill_lab.repos` 組み込み (採用かつ upstream の license が解消された場合) または (b) clone-on-demand の継続 (採用するが license が未解消の場合) または (c) 廃棄 (不採用) を選択する

## 採用検討時の clone 手順

すべて評価担当者が手動で実行する。本リポの clone 直後ではこの手順は走らない (= 再現性ガード)。

### 0. ライセンス受諾の宣言

```bash
export FASTLIO_SAM_LICENSE_ACK=yes
```

この環境変数がない状態で `scripts/clone_fastlio_sam_for_eval.sh` を実行すると、stderr に license caveat (本文書 §「ライセンス状況」の要旨) を出して exit 1 する。誤実行ガードであり、明示的な「私はこの license リスクを引き受ける」シグナルとして機能する。

### 1. clone と GTSAM (PPA 経由) インストール

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash         # ROS_DISTRO=humble を環境に投入
./scripts/clone_fastlio_sam_for_eval.sh
```

スクリプトは次を順に実施する:

1. Ubuntu 22.04 / `ROS_DISTRO=humble` / `FASTLIO_SAM_LICENSE_ACK=yes` を確認 (満たさなければ exit 1)
2. `borglab/gtsam-release-4.1` PPA を追加し、`libgtsam-dev libgtsam-unstable-dev` を投入 (既に同等版が入っていれば skip)
3. `src/third_party/FAST_LIO_SAM/` に upstream を clone (既に clone 済みなら fetch + master 更新)
4. `package.xml` の `<name>` が `fast_lio_sam` であることを確認

`colcon build` は**スクリプト内では実行しない**。理由は (a) license 状況の整理が本 Issue の主目的であり、ビルド成否は M5R-3 評価担当者の判断であること、(b) 上流の ROS 2 対応に "Full ROS2 adaptation"、"ROS2 adaptation Test" が TODO として残っており、master 時点でビルドが落ちる可能性があること。

### 2. ビルド (M5R-3 評価担当者が実行)

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio_sam --symlink-install
```

上流の `build.sh humble` を経由する経路も README に書かれているが、本リポは colcon を統一インターフェイスとするため直接 colcon を呼ぶ。`build.sh` 経由でしか通らない症状が出た場合、その差分は M5R-3 の ADR-0003 で記録する。

### 3. GTSAM の競合に関する警告

本リポでは既に M5R-1 (Issue #45) で **GTSAM 4.3a0 をソースビルドして `/usr/local/lib/libgtsam.so.4.3a0`** に配置している (GLIM 用)。本 Issue で導入する PPA 版は **GTSAM 4.1.1 を `/usr/lib/x86_64-linux-gnu/libgtsam.so.4.1.1`** に配置する。両者は ABI 非互換で、cmake の探索順序によりどちらが拾われるかが変わる:

- `find_package(GTSAM)` は通常 `/usr/local` を `/usr` より優先するため、デフォルトでは GLIM 用の 4.3a0 が選ばれる
- FAST-LIO SAM 側が `GTSAM 4.1` を要求している場合、cmake が見つけても version mismatch で警告 or fail することがある
- 必要に応じて `cmake -DGTSAM_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTSAM` で明示的に PPA 版を指す

M5R-3 着手時に、まずは何も明示せずビルドを試し、落ちたら上記の `GTSAM_DIR` 明示や、`/usr/local/lib/libgtsam*` を一時退避する等の対処を ADR-0003 の Context に記録する。両方を同時にビルドできる状態にする工夫 (例: GLIM 側 GTSAM を non-default prefix に逃がす) は、SLAM 候補確定後に必要なら別 Issue で扱う。

## 既知の不確実性

| 項目 | 内容 | M5R-3 での扱い |
|---|---|---|
| 上流 ROS 2 対応 | README の TODO に "Full ROS2 adaptation" / "ROS2 adaptation Test" が残っている。master 時点で `colcon build` が落ちる可能性あり | ビルドが落ちたら原因 (CMake パス、roscpp 残骸、API ドリフト等) を最小再現で記録し、ADR-0003 の Context に明記。対症修正は本リポ内 wrapper で対応 (CLAUDE.md 規約: third_party 直編集禁止) |
| 上流 LICENSE 追加 | 将来 LICENSE ファイルが追加される or 明示的に GPL-2.0 / MIT 等が宣言される可能性 | 追加された時点で本文書を更新し、`whill_lab.repos` 経路への切替を再評価。permissive 化されれば運用スタック取り込みも検討 |
| GTSAM 競合 | §「GTSAM の競合に関する警告」参照 | M5R-3 着手時にビルド試行 → 競合症状を確認 → 対処を ADR-0003 に記録 |
| sample bag | 上流 README が指す smoke test bag の取得経路 | 本 Issue では扱わない。M5R-3 で本リポの実 bag (M4-R bringup で取得したループ走行 bag) を入力とする方針 |

## M5R-3 (#48) への引き渡し

本文書の手順で clone + build が通る状態を起点に、M5R-3 で次を実施する:

1. 本リポの M4-R-bringup ループ走行 bag を入力に、GLIM と FAST-LIO SAM を**同条件**で回す
2. 生成 PCD のループクロージャ誤差 (始終点同一壁面の 3 点平均、目標 ≤ 0.5 m)、所要時間、VRAM 使用量、CPU/GPU 負荷を計測
3. 動的物体 bag (歩行者横断) でも同様に走らせ、後段 ERASOR への入力としての適性を確認
4. ADR-0003 (`docs/decisions/0003-mapping-slam-choice.md`) の Context 節に、license 状況、build 可否、loop closure 精度、所要 VRAM / 時間を記載
5. Decision 節で第一候補を確定 (GLIM 維持 or FAST-LIO SAM への変更)

ADR-0003 で「FAST-LIO SAM 採用」とならない場合、本文書と `scripts/clone_fastlio_sam_for_eval.sh` は履歴として残し、いずれ別 Issue で deprecation を判断する。

## 関連

- 開発方針: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 (採用候補表で FAST-LIO SAM は GLIM の代替)、§3.4 (ライセンス方針: GPL 系はオフラインのマップ作成ツール限定)
- M5-R 実行計画: [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md) §3.1 (`src/third_party/FAST_LIO_SAM/` の扱い)、§6 (Issue M5R-2 受け入れ基準の原文)
- 対の文書: [`m5r-glim-setup.md`](m5r-glim-setup.md) — 第一候補 GLIM のソースビルド手順
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) — 新規 docs は ja/en 並列で生やす
- スクリプト: [`scripts/clone_fastlio_sam_for_eval.sh`](../../scripts/clone_fastlio_sam_for_eval.sh) — 本文書と対の冪等 clone スクリプト
- 関連 Issue: #46 (本文書とスクリプト)、後続の #48 M5R-3 (実 bag 比較) と ADR-0003

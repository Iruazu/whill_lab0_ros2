# ADR 0005: `docs/maps/<site>/` マップ成果物規約

Language: [日本語](0005-maps-spec.md) | [English](../../en/decisions/0005-maps-spec.md)

- Status: proposed
- Date: 2026-06-21
- Deciders: Iruazu (承認待ち)

## 背景

親方針 ([`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md))
§6 受け入れ基準 (3) は M5-R の出力物として
「`docs/maps/<site>/` に pcd / pgm / yaml / 取得日 / 経路 / 天候のメタデータが
揃う」ことを要求する。M5-R 実行計画
([`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md))
§M5R-5 はこれを「ディレクトリ規約 + README 雛形 + `metadata.yaml` スキーマで
固める」と分解し、本 Issue (#47) で具体化された。

この規約は 1 度確定すると、

- M5R-6 (#50, 占有格子変換) が出力先として参照
- M5R-7 (#51, パイプライン統合) が E2E 文書化の前提
- M6-R (scan-to-map localizer + Nav2 obstacle layer 復帰) が入力前提
- M9 / 屋外拡張 (キャンパス本番経路、GNSS 連携) でメタデータ拡張

の連鎖で長期に渡って参照される。後発が触る規約のため、規約本体だけでなく
「なぜこの形か」「他の選択肢をなぜ棄却したか」を ADR として残す。

## 決定

1. **規約本体は [`../../maps/README.md`](../../maps/README.md) を正本とする**。
   ADR は規約の根拠と棄却案を保持し、規約自体の編集は README で行う。
   DRY のため詳細 (ディレクトリ規約、`metadata.yaml` スキーマ、gitignore
   規約、`_template/` の使い方、`lab-legacy-m5b/` の扱い) は README 参照。
2. **ディレクトリ構造**: `docs/maps/<site>/` 配下に `static.pcd`,
   `occupancy.pgm`, `occupancy.yaml`, `metadata.yaml` (任意で `README.md`)
   を置く。中間アーティファクト (bag、SLAM 直出力 PCD 等) は別の
   `docs/m5r-bench-data/` (M5R-7 で規約確定) に分離する。
3. **`metadata.yaml` 必須フィールド**: `acquired_at`, `route_summary`,
   `weather`, `slam_method`, `source_bag`, `commit`。任意: `slam_params`,
   `erasor_params`。スキーマ拡張時は README の表を更新してから足す。
4. **gitignore 規約**: `docs/maps/**/*.pcd` を再帰的に除外 (PCD は数十〜数百
   MB)。`.pgm` / `.yaml` / `.md` は tracked。
5. **`_template/` の運用**: 新規 site は `cp -r docs/maps/_template
   docs/maps/<site-name>` で開始。placeholder (`<...>`) を全て埋めてから
   commit する。
6. **i18n 例外**: `docs/maps/README.md` 本体は ADR-0001 が定める
   「`docs/{ja,en}/` 配下の narrative docs は二言語化」の対象外とし、
   日本語単独で書く。理由は `docs/m4r-bench-data/README.md` と同じく
   「運用 registry の README は読み手がリポ作業者に限定され、運用詳細を
   両言語維持するコストが利益を上回らない」。本 ADR 自体は ADR-0001 に従い
   二言語化する。

## 採用しなかった案

- **全ファイル ja/en 二言語化** (`docs/{ja,en}/maps/`):
  運用 registry を二言語維持するメリット (英語話者の発見性) より
  コスト (運用ドキュメント更新時の重複コスト、`_template/` の二重維持) が大きい。
  `docs/m4r-bench-data/` で先例が確立済。棄却。
- **`docs/maps/<site>/.gitkeep` 経由の register**:
  規約を `.gitkeep` だけで表現すると、後発が `metadata.yaml` なしで site を
  作ってしまう逸脱が起きやすい。`_template/` を出発点として強制する方が
  逸脱しにくい。棄却。
- **PCD を git LFS 化**:
  git LFS は研究室規模で運用負荷 (storage quota 管理、CI / CD での lfs
  fetch、PR review 時の diff 表示等) が増える。PCD は再生成可能
  (元 bag + SLAM パラメータがあれば作り直せる) ため、生成物ではなく
  「再生成手順」を tracked にする方が筋がよい。棄却。
- **`docs/m5-maps/` を delete のみで処理**:
  `velodyne_whill.yaml` と `nav_launch.py` が直接参照しており、即時削除すると
  active config が壊れる。リネーム経路で「凍結前試作品」と明示し、M5R-7 で
  新規約パスに向け直した時点で削除候補とする方が安全。棄却 (リネーム採用)。
- **`metadata.yaml` を JSON Schema で lint 強制**:
  スキーマ違反検出には有用だが、現時点で `metadata.yaml` は手書き想定 (M5R-7
  で自動生成される範囲もあるが全フィールドではない) で、JSON Schema 維持
  コストが現段階では割に合わない。M9 以降に site 数が増えた場合は再評価。
  本 ADR では棄却。

## 結果

得るもの:

- M5R-6 / M5R-7 / M6-R が「入力 / 出力ディレクトリの形」で迷わない
- 長期 (キャンパス本番経路 → 複数 site 拡張 → 屋外 GNSS 連携) で site が
  増えても、各 site が同じスキーマで揃う
- 再生成手順 (元 bag + パラメータ) が `metadata.yaml` に必ず残るため、
  SLAM / ERASOR のパラメータ調整による再生成が可能

失うもの (あるいはコスト):

- 新規 site 作成時に `metadata.yaml` の placeholder を全て埋める手間
  (M5R-7 で `commit` や `acquired_at` などの自動生成スクリプトを整備して
  軽減する想定)
- 規約変更 (フィールド追加等) のたびに `_template/` と既存全 site を
  揃える必要がある。M5-R 期間中は site 数が少ないので問題にならないが、
  M9 以降は ADR で変更を管理する

後続作業:

- M5R-6 (#50): 占有格子変換スクリプトを本規約の出力先 (`docs/maps/<site>/
  occupancy.{pgm,yaml}`) に向ける
- M5R-7 (#51): bag → SLAM → ERASOR → 占有格子 → `docs/maps/<site>/` 格納の
  E2E パイプライン文書を作成。`commit` 等の自動生成スクリプトもここで整備
- M6-R: scan-to-map localizer が `docs/maps/<site>/static.pcd` を入力に
  `map -> odom` を publish。Nav2 obstacle layer は `docs/maps/<site>/
  occupancy.yaml` を map_server に流す
- M9 / 屋外拡張: `metadata.yaml` に GNSS 関連フィールド (`gnss_used`,
  `utm_zone` 等) を追加する際は本 ADR を superseded by NNNN で更新

## 関連

- 親方針: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §3.1 (二相分離)、§6 (3) 受け入れ基準、§7 (ADR 候補に「ADR-0005 候補:
  `docs/maps/<site>/` 規約の確定」)
- M5-R 実行計画: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §M5R-5、§11 (本 ADR を「ADR-0005 候補」として明記)
- 規約本体: [`../../maps/README.md`](../../maps/README.md)
- リネーム記録: [`../legacy-findings/2026-06-21-m5b-maps-renamed.md`](../legacy-findings/2026-06-21-m5b-maps-renamed.md)
- 関連 Issue: #47 (本 Issue、M5R-5)、#50 (M5R-6)、#51 (M5R-7)

# whill_lab0_ros2 docs

Language: [日本語](README.md) | [English](../en/README.md)

noetic → humble 移植のプロジェクトレベル文書を集約する。本インデックスから辿るか、目的のマイルストーン文書に直接飛ぶこと。

## 開発方針

現行の前向き計画の正本は方針文書 [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) (platform-pivot) である。転換後のフェーズ M4-R 〜 M9、禁止事項 (凍結された M5-d / M5-e、identity な `tf_bridge` の継続利用、FAST-LIO のランタイム強化)、フェーズごとの受け入れ基準をここに定義している。

旧 [移行計画](migration-plan.md) は初期 M1〜M3 の noetic → humble 移植の実行記録として残るが、次に作るべきものの判断根拠ではなくなった。

## 計画 / 研究 / 意思決定

`docs/` 配下のサブディレクトリは性質の異なる 3 種の文書を分離している:

- [`plans/`](plans/) — `pm-orchestrator` が起草する複数フェーズ計画。各計画には受け入れ基準と禁止事項が明示される。新フェーズはここから始まる。
  - [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
- [`research/`](research/) — `research-analyst` が執筆する技術調査。ADR と計画の入力として参照される。
  - [`research/2026-06-11-localization-survey.md`](research/2026-06-11-localization-survey.md)
- [`decisions/`](decisions/) — Architecture Decision Record (`NNNN-*.md`)。下書きは agent が作ることもあるが、`accepted` 行を確定するのは人間。検討中の ADR 候補は [方針文書 §7](plans/2026-06-11-platform-pivot.md) に列挙されている。
  - 採択済み:
    - [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) — docs 両言語化方針

## noetic スタックからの引き継ぎ

- [LiDAR ↔ IMU 外部パラメータ (noetic 由来)](m3-extrinsics-from-noetic.md) — `whill_lab0/FAST_LIO/config/velodyne.yaml` の校正値。M4 の出発点として使用した。
- [旧リポインデックス](legacy-index.md) — `~/whill_lab0/` のエントリポイント・マップ。`legacy-archaeologist` エージェントの起点。

## マイルストーン

初期ロードマップ (M1〜M5) は完了済。転換後のロードマップ (M4-R 〜 M9) は方針文書 §4 とリポ直下 README で追跡する。下表のフェーズ別文書は歴史的な実行記録という位置付け。

| | 文書 | ステータス |
|--|------|-----------|
| M1 | [環境構築](m1-environment-setup.md) | 完了 |
| M2 | [実機での WHILL コアドライバ](m2-whill-core.md) | 完了 |
| M3 | [センサスタック](m3-sensors.md) | 完了 (PR #4 が 2026-05-07 に、PR #5 wrap-up が 2026-05-08 にマージ) |
| M4 | [自己位置推定 — FAST-LIO](m4-localization.md) | 完了 (PR #6 が 2026-05-08 にマージ) |
| M5 | [ナビゲーション — Nav2](m5-navigation.md) | M5-a/b/c/d 完了 (PR #7 が 2026-05-20 にマージ)。M5-d の継続と M5-e は 2026-06-11 のプラットフォーム転換により凍結 |
| M4-R 〜 M9 | — (フェーズ別文書は未起草) | [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §4 を参照 |

## セッションログ

調査作業を日付つきで残す物語形式の記録。同じ袋小路を後続のコントリビュータが繰り返さないために置く。

- [2026-05-06](session-2026-05-06.md) — M2 wrap-up: Model CR2 のコールドブート問題、fork パッチ、E2E 検証
- [2026-05-07](session-2026-05-07.md) — M3 wrap-up: 3 センサ車載 bringup、IMU lifecycle 競合の修正、RealSense 型番の訂正 (D455 → D435)、Velodyne netplan、M4 入力用の静止 / 走行 bag 収録
- [2026-05-08](session-2026-05-08.md) — M4 ベースライン: FAST-LIO bringup、identity extrinsic の遠回りと回復、収録品質が支配的という 3-run 再現性試験

## 規約

- マイルストーン 1 件につき文書 1 つ。命名は `mN-<slug>.md`。各文書は PR レビューで消化できる `Status` 表で終わる
- セッションログは `session-YYYY-MM-DD.md` 形式。結論だけでなく辿った *経路* (誤診断を含む) を残す
- 外部の権威ある参照 (ベンダ PDF、上流 README) はリンクするだけでコピーしない。ただしその *解釈* は本リポに置き、理解の責任は自分たちで持つ
- 言語別の文書は `docs/ja/` と `docs/en/` に並列で配置する (方針: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md))。Issue #15 完了をもって全 docs が両言語化済み

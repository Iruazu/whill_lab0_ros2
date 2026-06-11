## 対応 Issue

Closes #

## 変更サマリ

| ファイル | 変更内容 |
|---------|---------|
|         |         |

## 方針適合チェック (docs/plans/2026-06-11-platform-pivot.md 5 章)

- [ ] `tf_bridge_launch.py` の identity 構成を前提とした機能追加ではない
- [ ] FAST-LIO のランタイム localizer 強化ではない (マップ作成品質目的の調整は可)
- [ ] `use_collision_detection: false` のまま自律走行系の機能を増やしていない
- [ ] 配車 / Web ロジックを Nav2 / localization ノードに密結合させていない
- [ ] `src/third_party/` 非編集・GPL コードのコピペなし

## ビルド・検証結果

- `colcon build --packages-up-to <pkg>`:
- 受け入れ基準の充足 (Issue から転記し、結果を併記):
  - [ ]

## code-reviewer 結果

重大: N / 改善余地あり: N / 好みの問題: N
(重大 0 が PR 提出の条件。レビュー全文は下の details に貼付)

<details><summary>レビュー全文</summary>

</details>

## 実機検証 (ユーザー側に残る手順)

1.

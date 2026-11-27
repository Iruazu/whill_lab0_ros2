## 対応 Issue

Closes #

## 変更サマリ

| ファイル | 変更内容 |
|---------|---------|
|         |         |

## 方針適合チェック (CLAUDE.md の現行方針、docs/plans/2026-10-30-roadmap-m8.md)

- [ ] FAST-LIO の runtime launch と identity TF を復活させていない
- [ ] failsafe / twist_mux を外した構成で cmd_vel を出す機能を増やしていない
- [ ] 下の層の値（実測値、TF、校正）が未確定のまま、上の層のパラメータで打ち消していない
- [ ] 配車 / Web ロジックを Nav2 / localization ノードに密結合させていない
- [ ] `src/third_party/` 非編集・GPL コードのコピペなし

## ビルド・検証結果

- `colcon build --packages-up-to <pkg>`:
- 受け入れ基準の充足 (Issue から転記し、結果を併記):
  - [ ]

## デモ回帰チェック（[手順](../docs/regression-check.md)）

- [ ] `scripts/demo_regression.sh`（全 PR）
- [ ] `scripts/demo_regression.sh --level replay`（安全・位置推定・TF・センサ処理・地図の変更）
- [ ] 屋外でデモのルートを1周（走行に影響する変更）。実施しない場合は理由:

## code-reviewer 結果

重大: N / 改善余地あり: N / 好みの問題: N
(重大 0 が PR 提出の条件。レビュー全文は下の details に貼付)

<details><summary>レビュー全文</summary>

</details>

## 実機検証 (ユーザー側に残る手順)

1.

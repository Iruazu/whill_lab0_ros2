---
description: 旧 noetic 実装から特定機能を ROS 2 humble に移植する全工程をオーケストレート。legacy 調査 → 計画 → 必要なら技術調査 → 実装 → レビューを順に実行。
argument-hint: <移植したい機能名や領域>
---

以下の機能を、旧 `~/whill_lab0/` (ROS noetic) から本リポ (ROS 2 humble) に移植する作業を進めてください。

対象機能:
$ARGUMENTS

## 必須の進行手順

各ステップは独立した subagent に delegate すること。並列実行可能なステップは並列で。

### Step 1: 旧実装の調査
`legacy-archaeologist` agent を Task tool で起動。
入力: 上記の機能名
期待する出力: 構造化された findings レポート (entry point, data flow, 主要パラメータ, 移植上の注意)

### Step 2: 技術代替の調査 (必要なら)
旧実装が古い手法を使っていたり、ROS 2 でより良い選択肢がある場合のみ、`research-analyst` agent を起動。
入力: 旧実装の手法 + ROS 2 humble での代替候補の比較
期待する出力: 候補比較表と推奨案

### Step 3: 実装計画の策定
`pm-orchestrator` agent を起動。
入力: Step 1 と Step 2 の出力
期待する出力: phase 分解された計画。`docs/plans/YYYY-MM-DD-<slug>.md` に保存

### Step 4: ユーザー確認
計画を提示し、承認を得る。質問があれば最大 3 つまで。

### Step 5: 実装
承認後、`ros2-implementer` agent を起動して計画に沿って実装。
phase が複数ある場合は phase ごとに区切る。

### Step 6: レビュー
実装直後に `code-reviewer` agent を起動。
重大の指摘があれば Step 5 に戻る。

### Step 7: ユーザーへの引き渡し
- 変更サマリ
- 実機検証手順 (ユーザーが実行すべきコマンド)
- 残課題

## 守るべきこと

- 旧実装を盲目的にコピーしない。ROS 2 humble の現代的なパターンに合わせて作り直す
- 各エージェントの context window を分離するため、Task tool で起動する。メイン context で直接調査・実装を始めない
- 計画を出してからユーザー承認を必ず取る

---
description: code-reviewer を明示的に起動して指定範囲をレビュー
argument-hint: <レビュー対象のパス、または "uncommitted" で git diff の差分>
---

`code-reviewer` agent を起動し、以下の範囲をレビューしてください。

対象:
$ARGUMENTS

引数が `uncommitted` の場合は `git diff` および `git diff --staged` の出力範囲を対象とする。
それ以外の場合はそのパスを対象に、関連する `package.xml` / `CMakeLists.txt` / 該当パッケージ README も併せて参照すること。

code-reviewer の system prompt 通り、重大 / 改善余地あり / 好みの問題 を分けて報告すること。

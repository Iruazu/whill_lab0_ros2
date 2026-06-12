# whill_lab0_ros2 — プロジェクトメモリ

このファイルは全ての Claude Code セッションに自動で読み込まれる。**最高レバレッジな文脈**なので、雑多な情報の貯蔵庫にしないこと。具体的な how-to は各パッケージの README、各エージェントの system prompt、`docs/` に寄せる。

## プロジェクトの位置づけ

- 宇都宮大学キャンパス内での実機検証を目標とした、WHILL Model CR2 ベース自律移動車椅子スタック
- **旧実装**: ROS noetic, 別リポジトリ `~/whill_lab0/` (本リポと並列に存在)。学内自律走行・運転アシスト等の機能が実装済みだが、リポジトリとしての整理状態が悪い
- **本リポ**: 旧実装を「クリーンに」ROS 2 humble に移植する。既存実装を盲目的にコピーするのではなく、現代の ROS 2 パターン (lifecycle node, ament_cmake, Nav2 standard) に合わせて作り直す
- マイルストーン: M1〜M2 完了。M3〜M5 進行中。M6 が実機統合検証

## アーキテクチャ層

「マップ作成 (オフライン・母艦)」と「運用 (オンライン・車載)」を分離する二相構成を採用する。
詳細と選定理由は `docs/ja/plans/2026-06-11-platform-pivot.md` の 3 章。

```
[マップ作成フェーズ (オフライン, 母艦)]
  手動走行 bag → ループクロージャ付き SLAM → 動的物体除去
    → docs/maps/<site>/ に静的 PCD + 2D 占有格子 + メタデータを保存
  (FAST-LIO はランタイム localizer ではなく、この層の「マップ作成ツール」として扱う)

[運用フェーズ (オンライン, 車載) — REP-105 準拠の TF 構造]
  map -> odom         : scan-to-map localizer (保存済み地図への補正。飛びを含んでよい)
  odom -> base_link   : robot_localization EKF (/whill/odom + IMU。連続・滑らか)
  base_link -> sensors: 実測 extrinsic の static TF
                ↓
        Nav2 (経路計画・追従) + フェイルセーフ (発散検知 → cmd_vel 遮断)
                ↓
        whill_dispatch (配車ゲートウェイ: NavigateToPose wrapper, ジョブキュー, 状態 publish)
                ↓
        Web / タブレット UI (rosbridge 経由)
```

**Claude は必ず関連パッケージの README を読んでから実装に入る**こと。
パッケージ実装の詳細は各 `README.md`、フェーズ計画は `docs/plans/` を参照。

## コーディング規約 (絶対)

- **コメントは "なぜ" を書く**。"何をしている" だけのコメントは要らない (コード読めば分かる)。判断の根拠・捨てた選択肢・既知の制約を残す
- **AI が書いたと一目で分かる文体を出力しない**:
  - 絵文字を装飾に使わない
  - 「Let me ...」「I'll ...」のような独白を出力に残さない
  - 過剰な見出しの装飾・太字の乱用をしない
  - 「素晴らしい質問です」のような追従を入れない
- **冪等性**: スクリプトは再実行可能であること。既存のスクリプト (`scripts/*.sh`) のスタイルを踏襲
- **ライセンス**: BSD-3-Clause で統一
- **package.xml の exec_depend を必ず正確に**書く。ament_cmake は `buildtool_depend` のみ
- **launch ファイル**: `IncludeLaunchDescription` で wrap される可能性を考慮、`LaunchConfiguration` をパス resolve に使わず launch description 生成時にハードコードする (既存の `fast_lio_launch.py` と `nav_launch.py` のコメント参照)

## ファイル所在の規約

| 種類 | 場所 |
|------|------|
| ROS 2 パッケージ | `src/whill_*/` |
| 上流パッケージ (vcs import) | `src/third_party/` (gitignore 済み) |
| マイルストーン文書 | `docs/m{N}-*.md` |
| セッションログ・意思決定 | `docs/session-YYYY-MM-DD.md` |
| ADR (Architecture Decision Record) | `docs/decisions/NNNN-*.md` |
| 旧実装の調査結果 | `docs/legacy-index.md` および `docs/legacy-findings/` |
| ベンチデータ | `docs/m3-bench-data/` (実バグは gitignore、README/PDF のみ commit) |

## 旧 noetic リポジトリ

- パス: `~/whill_lab0/` (環境によって変えるならここを書き換え + `legacy-archaeologist.md` も更新)
- このリポはサイズが膨大かつ整理されておらず、Claude の context window を圧迫する
- **`legacy-archaeologist` エージェント以外は旧リポを直接読まない**。必要な情報は `docs/legacy-index.md` 経由で参照する

## チーム体制 (.claude/agents)

要件次第で自動で適切なエージェントが呼ばれる。明示的に invoke したい場合は `/plan` `/port-feature` `/debug` `/research` `/review` を使う。

| 状況 | 起動すべきエージェント |
|------|-----------------------|
| 方針判断・新フェーズ着手 | まず `docs/ja/plans/2026-06-11-platform-pivot.md` を参照 |
| 「〇〇機能を移植したい」「〇〇を作りたい」 | まず `pm-orchestrator` |
| 「旧実装はどうやっていたか」「noetic 側の挙動を知りたい」 | `legacy-archaeologist` |
| 「ROS 2 で〇〇を実装したい」(計画済み) | `ros2-implementer` |
| 「〇〇の技術選定」「△△と□□の比較」 | `research-analyst` |
| 「動かない」「エラーが出る」「diverge する」 | `debugger` |
| 実装直後 | `code-reviewer` (自動 trigger) |

## 進行中の既知課題

`docs/ja/plans/2026-06-11-platform-pivot.md` 2 章の診断 (P1〜P5) を要約転記する。詳細根拠と
解消経路は同文書の 3 章 (アーキテクチャ) と 4 章 (マイルストーン M4-R 以降) を参照:

- **P1: 運用時の自己位置に補正経路がない** (`map -> camera_init` identity 固定で FAST-LIO ドリフトがそのまま map 誤差化、60s で 18%)。M6-R で scan-to-map localizer に置換予定
- **P2: 初期位置合わせ機構がない** (起動位置 = camera_init 前提)。M6-R の initial pose 運用で解消予定
- **P3: 発散を検知も回復もしない** (歩行者横断で破綻しても TF は出続け Nav2 は走行継続。run3 実測)。M6-R のフェイルセーフノードで遮断する
- **P4: odom フレーム不在・車輪オドメトリ未使用** (`ros2_whill` の `/whill/odom` が未統合)。M4-R で robot_localization EKF を導入し `odom -> base_link` を構築、`map -> odom` を後段の localizer に分離する
- **P5: 地図品質の問題が安全機能を連鎖停止** (ゴースト障害物 → `use_collision_detection: false`、QoS 不一致 → obstacle layer なし)。M5-R のマップパイプライン + M6-R の obstacle layer 復活で解消予定

旧 M5-d (goal-following) / M5-e (tuning) は本方針下で**凍結**。`tf_bridge_launch.py` の identity 構成を前提とした新機能追加と、FAST-LIO のランタイム localizer 強化は禁止 (本文書 5 章)。

## Import

@docs/ja/plans/2026-06-11-platform-pivot.md
@docs/legacy-index.md

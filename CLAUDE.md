# whill_lab0_ros2 — プロジェクトメモリ

このファイルは全ての Claude Code セッションに自動で読み込まれる。**最高レバレッジな文脈**なので、雑多な情報の貯蔵庫にしないこと。具体的な how-to は各パッケージの README、各エージェントの system prompt、`docs/` に寄せる。

## プロジェクトの位置づけ

- 宇都宮大学キャンパス内での実機検証を目標とした、WHILL Model CR2 ベース自律移動車椅子スタック
- **旧実装**: ROS noetic, 別リポジトリ `~/whill_lab0/` (本リポと並列に存在)。学内自律走行・運転アシスト等の機能が実装済みだが、リポジトリとしての整理状態が悪い
- **本リポ**: 旧実装を「クリーンに」ROS 2 humble に移植する。既存実装を盲目的にコピーするのではなく、現代の ROS 2 パターン (lifecycle node, ament_cmake, Nav2 standard) に合わせて作り直す
- マイルストーン: M1〜M2 完了。M3〜M5 進行中。M6 が実機統合検証

## アーキテクチャ層

```
whill_sensors_bringup (M3)   ─ Velodyne VLP-16 + RealSense D435 + RT 9-axis IMU
        │
        ▼
whill_localization (M4)      ─ FAST-LIO (LiDAR-Inertial Odometry)
        │
        ▼
whill_navigation (M5)        ─ Nav2 lifecycle, RPP controller, velocity_smoother
        │
        ▼
whill_bringup (M6)           ─ 統合 launch、on-vehicle 検証 (未着手)
```

詳細は各パッケージの `README.md`。**Claude は必ず関連パッケージの README を読んでから実装に入る**こと。

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
| 「〇〇機能を移植したい」「〇〇を作りたい」 | まず `pm-orchestrator` |
| 「旧実装はどうやっていたか」「noetic 側の挙動を知りたい」 | `legacy-archaeologist` |
| 「ROS 2 で〇〇を実装したい」(計画済み) | `ros2-implementer` |
| 「〇〇の技術選定」「△△と□□の比較」 | `research-analyst` |
| 「動かない」「エラーが出る」「diverge する」 | `debugger` |
| 実装直後 | `code-reviewer` (自動 trigger) |

## 進行中の既知課題

これらは `docs/` の該当ファイルにも書いてあるが、Claude が頻繁に参照する内容なので転載:

- **FAST-LIO のループクロージャ不在**: 60 秒ドライブで 18% のドリフト。`map -> camera_init` を identity でつないでいるため、長距離では破綻する。M5-e の TODO
- **車輪オドメトリ未統合**: M2 で動いている `ros2_whill` の `/whill/odom` が未使用。`odom -> base_link` を車輪駆動、`map -> odom` を Fast-LIO 補正という標準 Nav2 構成への移行が pending
- **M5-b 静的マップにゴースト障害物**: 再キャプチャ必要。それまでは `use_collision_detection: false` で凌いでいる

## Import

@docs/legacy-index.md

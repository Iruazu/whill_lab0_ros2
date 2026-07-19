# whill_lab0_ros2 — プロジェクトメモリ

このファイルは全ての Claude Code セッションに自動で読み込まれる。**最高レバレッジな文脈**なので、雑多な情報の貯蔵庫にしないこと。具体的な how-to は各パッケージの README、各エージェントの system prompt、`docs/` に寄せる。

## プロジェクトの位置づけ

- 宇都宮大学キャンパス内での実機検証を目標とした、WHILL Model CR2 ベース自律移動車椅子スタック
- **旧実装**: ROS noetic, 別リポジトリ `~/whill_lab0/` (本リポと並列に存在)。学内自律走行・運転アシスト等の機能が実装済みだが、リポジトリとしての整理状態が悪い
- **本リポ**: 旧実装を「クリーンに」ROS 2 humble に移植する。既存実装を盲目的にコピーするのではなく、現代の ROS 2 パターン (lifecycle node, ament_cmake, Nav2 standard) に合わせて作り直す
- マイルストーン (platform-pivot §4 の M*-R 系): M4-R / M5-R 完了。M6-R は localizer / failsafe / obstacle layer まで実質完了 (残: M6R-5 統合受入の証跡確定)。M7 (whill_dispatch 配車 API) 着手中。マップ品質の v2 パイプライン (PR #91) は完了・受入済

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
- **launch を編集したら push 前に `ros2 launch <pkg> <launch>.py --show-args` を必ず走らせる**。Python の import エラー (module 名間違い・非公開 API 等) は colcon build を通過するが `--show-args` は launch description を評価するので import 段で落ちる。例: `SetRemap` は `launch_ros.actions` にあり `launch.actions` にはない (2026-07-14 実機で ImportError 発覚済)。build 通過 + syntax OK は不十分
- **main へ直接 commit / push しない**。revert / hotfix / 事故対応でも必ず branch + PR 経由。マージは常にユーザーが行う

## ランタイム環境の前提 (本機 = Alienware x15 R2)

bag 録画 / GLIM オフライン処理 / M6-R 検証で launch する前に、各ターミナルで
以下を確認すること。1 つでも食い違うと再現性が崩れる (2026-06-24 に
`/velodyne_points` 1 Hz 病で実証済。詳細: `docs/ja/m5r-rmw-cyclonedds.md`):

```bash
echo $RMW_IMPLEMENTATION                                  # rmw_cyclonedds_cpp
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor # performance
```

- **RMW**: 既定の FastDDS は `velodyne_msgs/VelodyneScan` 等の大メッセージで
  間欠的に詰まる。`~/.bashrc` に `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を
  入れて永続化する (Claude は `~/.bashrc` を編集しない。ユーザー手動)
- **CycloneDDS の xml は 2 本体制**: 運用は `configs/cyclonedds-runtime.xml`、
  bag 録画時のみそのターミナル限定で `CYCLONEDDS_URI` を
  `configs/cyclonedds-bag-record.xml` に切り替える (lo-only は録画専用。
  経緯: tethering hazard、`docs/ja/plans/2026-06-24-m6r-localization.md` §10)
- **bringup は単一ターミナルのみ**: `m6r_bringup` と sensors / odom 系 launch の
  並行起動は禁止 (2026-07-16 実機で全ノード二重化 → `/velodyne_points` 39.4 Hz)。
  RealSense は opt-in
- **自律走行は failsafe 有効時のみ** (autonomy gate): whill_safety
  (twist_mux + failsafe) を含まない構成で cmd_vel を発行する検証は禁止。
  demo は並走者が介入できる速度・体制で行う
- **CPU governor**: 再起動で `powersave` に戻るため、録画/SLAM 前に毎セッション
  `sudo cpupower frequency-set -g performance` を実行する
- **NVIDIA suspend/resume**: サスペンド→レジューム後、GPU 使用プロセス
  (GLIM 等) が `cudaErrorUnknown` で起動直後に落ちる。初回のみ
  `sudo ./scripts/install_nvidia_suspend_fix.sh` を実行 → 再起動しておく
  (Issue #76、詳細: `docs/ja/host-setup-nvidia-suspend.md`)
- 録画後は `ros2 bag info <bag-dir>` で `/velodyne_points` count ≈ 走行秒 × 10、
  `/imu/data_rep145` count ≈ 走行秒 × 100 を確認。半分以下なら録画破棄して再録

## ファイル所在の規約

| 種類 | 場所 |
|------|------|
| ROS 2 パッケージ | `src/whill_*/` |
| 上流パッケージ (vcs import) | `src/third_party/` (gitignore 済み) |
| マイルストーン文書 | `docs/m{N}-*.md` |
| セッションログ・意思決定 | `docs/session-YYYY-MM-DD.md` |
| ADR (Architecture Decision Record) | `docs/decisions/NNNN-*.md` |
| 旧実装の調査結果 | `docs/ja/legacy-index.md` および `docs/legacy-findings/` |
| ベンチデータ | `docs/m{N}-bench-data/` (M3: `docs/m3-bench-data/`、M4-R: `docs/m4r-bench-data/`。実バグは gitignore、README/PDF のみ commit) |

## 旧 noetic リポジトリ

- パス: `~/whill_lab0/` (環境によって変えるならここを書き換え + `legacy-archaeologist.md` も更新)
- このリポはサイズが膨大かつ整理されておらず、Claude の context window を圧迫する
- **`legacy-archaeologist` エージェント以外は旧リポを直接読まない**。必要な情報は `docs/ja/legacy-index.md` 経由で参照する

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

`docs/ja/plans/2026-06-11-platform-pivot.md` 2 章の診断 (P1〜P5) は **全件解消済**。
経緯詳細は同文書と各計画書・ADR を参照 (ここには結論のみ残す):

- **P1 (解消済、2026-07-12, M6R-2)**: scan-to-map localizer (ADR-0006) が `map -> odom` を補正。屋外 live 受入 PASS (静止 fitness 0.019、12 min 走行 reject 0)。`tf_bridge` の identity 構成は物理削除済
- **P2 (解消済、2026-07-12, M6R-2)**: `/initialpose` 運用で任意地点からのリローカライズが成立
- **P3 (解消済、M6R-3)**: whill_safety のフェイルセーフ (ADR-0007) + twist_mux が発散時に cmd_vel を遮断。preflight gate (`m6r_preflight.sh`) が起動前検査を担う
- **P4 (解消済、2026-06-20, M4-R)**: robot_localization EKF が `odom -> base_link` を 30 Hz publish (`/whill/odom` + `/imu/data_raw` fuse)。統合 bringup は `ros2 launch whill_localization odom_bringup_launch.py`。詳細は `src/whill_localization/README.md`
- **P5 (解消済)**: 地図側は M5-R パイプライン (ADR-0003/0004/0005) + v2 パイプライン (PR #91、layer 分離 + sidecar mask + verifier) で salt 焼き込みまで根治。Nav2 側は M6R-4 で obstacle layer 復活 + `use_collision_detection: true` 復帰 (ADR-0009/0010/0011)

現在の主課題はフェーズ表の通り M7 (配車 API 層) と、実機検証で残る運転品質
(直進時の左右振動等。既存パラメータは実測前の一般論由来のため実測ベースで再調整する)。

旧 M5-d (goal-following) / M5-e (tuning) は本方針下で**凍結**。`tf_bridge_launch.py` の identity 構成を前提とした新機能追加と、FAST-LIO のランタイム localizer 強化は禁止 (本文書 5 章)。

## Import

@docs/ja/plans/2026-06-11-platform-pivot.md
@docs/ja/legacy-index.md

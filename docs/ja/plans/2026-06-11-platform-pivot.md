# 開発方針: 配車プラットフォーム化に向けたアーキテクチャ再編

Language: [日本語](2026-06-11-platform-pivot.md) | [English](../../en/plans/2026-06-11-platform-pivot.md)

- 日付: 2026-06-11
- 状態: accepted (2026-06-11 ユーザー承認。同日 CLAUDE.md の Import / アーキテクチャ層 / 既知課題 / チーム体制 表に反映済み)
- 想定配置: `docs/ja/plans/2026-06-11-platform-pivot.md`
- 読者: 本リポジトリで作業する全ての Claude Code セッションと subagent。
  `pm-orchestrator` は計画策定時、本文書を最上位の入力として扱うこと。
  CLAUDE.md が「規約とファイル所在」を定めるのに対し、本文書は
  「何をどの順で作るか」と「進んではいけない方向」を定める。

## 0. 背景

2026-06 の技術調査 (SLAM / localization 手法、つくばチャレンジ完走チーム構成、
ライセンス、計算資源) と現実装の診断により、以下が確定した:

- 現在の M5-a TF ブリッジ (`map -> camera_init` identity 固定) は
  「短時間・有人・マッピング開始地点からの起動」というデモ条件でのみ成立する近似である
- run3 (歩行者横断で FAST-LIO 発散) が示す通り、この近似は
  「人の往来がある環境での運用」という本来の目標と原理的に両立しない
- この近似の上に M5-d (goal-following) / M5-e (tuning) を積み増す現在の軌道が
  「間違った方向」の正体であり、機能追加を止めて土台を差し替える必要がある

## 1. 北極星 (最終プロダクト像)

搭乗者にタブレットを渡し、Web 上のプラットフォームアプリから

1. 目的地 (配車先) の指定
2. 空車の呼び出し (椅子が無人で迎えに来る)

ができる、キャンパス内自律移動プラットフォーム。

この像から逆算すると、走行スタックの必須要件は次の 6 つになる。
以降の全ての設計判断は、この表に対して説明できなければならない。

| ID | 要件 | 北極星のどこから来るか |
|----|------|----------------------|
| R1 | 任意地点からの起動とリローカライズ | 呼び出しに応じるには「いま地図上のどこにいるか」を自力特定する必要がある。起動位置の暗黙仮定は成立しない |
| R2 | 永続的な map 座標系 | 配車先は地図上の名前付き地点として保存される。セッションごとに座標系が変わる構成では目的地が定義できない |
| R3 | 長時間・反復運用で破綻しない自己位置 | 1 乗車数分 × 1 日多数回。ドリフト蓄積型 (補正なしオドメトリ) は不可 |
| R4 | 無人走行の安全 (発散検知・自動/遠隔停止) | 呼び出し = 搭乗者ゼロでの走行。人が異常に気付いてジョイスティックで止める前提が使えない |
| R5 | 歩行者の動的回避 | キャンパスには人が歩いている。static map のみを根拠に走ることは許されない |
| R6 | Web との明確な操作境界 (API) | タブレット UI と ROS 2 の間に、認証・状態同期・ジョブ管理を載せられる境界が要る |

## 2. 現状の問題 (診断要約)

詳細な根拠は 2026-06 の診断レポートにあるが、要点を転記する。
番号は以降のフェーズ定義から参照される。

| ID | 問題 | 根拠 | 矛盾する要件 |
|----|------|------|------------|
| P1 | 運用時の自己位置に補正経路がない。`map -> camera_init` identity 固定で、FAST-LIO のドリフト (実測 18%/60s) がそのまま map 座標の誤差になる | `tf_bridge_launch.py` | R2, R3 |
| P2 | 初期位置合わせ機構がない。camera_init = 起動位置のため、マッピング開始地点・同方位で起動しないと最初からズレる | `tf_bridge_launch.py`, `pcd_to_occupancy_grid.py` の origin 前提 | R1 |
| P3 | 発散を検知も回復もしない。歩行者 1 人で発散し得る (run3 実測) のに TF は出続け、Nav2 は走行を継続する | `whill_localization/README.md` run3 | R4 |
| P4 | odom フレーム不在・車輪オドメトリ未使用。補正導入時のジャンプ緩衝材がなく、LiDAR 縮退時のバックアップもない | `nav2_params.yaml` コメント、CLAUDE.md 既知課題 | R3, R4 |
| P5 | 地図品質の問題が安全機能を連鎖停止させている。ゴースト障害物 -> `use_collision_detection: false`、QoS 不一致 -> obstacle layer なし。運用中の歩行者が costmap に一切映らない | `nav2_params.yaml` コメント | R5 |

## 3. 採用アーキテクチャ (決定事項)

### 3.1 二相分離

「マップ作成フェーズ (オフライン・高精度)」と「運用フェーズ (オンライン localization)」を
分離する。つくばチャレンジ完走チーム・商用配送ロボット・WHILL 社実証研究に共通する
標準構成であり、車載 PC の要求スペックを下げる効果も持つ
(マッピングの重い大域最適化は bag を母艦で後処理すればよい)。

```
[マップ作成 (オフライン, 母艦)]
  手動走行で bag 収録
    -> ループクロージャ付き SLAM で点群地図生成
    -> 動的物体除去 (歩行者トレースの削除)
    -> 静的 PCD + 2D 占有格子を docs/maps/<site>/ に保存

[運用 (オンライン, 車載)]
  保存済み地図 + scan-to-map localization が map -> odom を供給
  車輪オドメトリ + IMU の EKF が odom -> base_link を供給
  Nav2 が経路計画・追従
  配車ゲートウェイが Web からのジョブを Nav2 action に変換
```

### 3.2 TF と責務 (REP-105 準拠)

```
map -> odom        : localizer (scan-to-map 補正。飛びを含んでよい)
odom -> base_link  : robot_localization EKF (/whill/odom + /imu/data_raw。連続・滑らか)
base_link -> sensors: 実測 extrinsic の static TF (現状の identity placeholder を置換)
```

- FAST-LIO は「マップ作成ツール」に役割変更する。ランタイム localizer としての
  利用・強化は凍結 (理由: P1〜P3 を構造的に解決できないため)
- `tf_bridge_launch.py` の identity 2 本は M4-R 完了時点で廃止する

### 3.3 採用候補 (第一候補 / 代替 / 選定理由)

| 役割 | 第一候補 | 代替 | 理由 |
|------|---------|------|------|
| マップ作成 SLAM | GLIM (MIT, ROS 2 humble 公式, GPU 母艦で後処理) | FAST-LIO SAM / li_slam_ros2 (VLP-16 実績, つなぎ・比較用) | 大域最適化と対話的地図修正。permissive ライセンス |
| 動的物体除去 | ERASOR 系 | Removert | 高速・静的点の保全。オフライン処理なので車載要件なし |
| 運用 localization | lidar_localization_ros2 (NDT_OMP) | hdl_localization / Autoware ndt_scan_matcher / FAST_LIO_LOCALIZATION 系 | つくばチャレンジ 2024 実績。odometry 拘束併用が前提 |
| odom 融合 | robot_localization EKF | — | Nav2 標準。/whill/odom は M2 で既に動作済 |
| フェイルセーフ | 自作の小ノード (マッチングスコア / 共分散監視 -> cmd_vel 遮断) | — | R4 の最小実装。localizer 選定時はリセット機構 (emcl2 / mcl_3dl の膨張リセット思想) の有無も評価軸に入れる |
| 動的障害物 | pointcloud_to_laserscan 等の QoS 橋 -> obstacle layer 復活 -> `use_collision_detection: true` 復帰 | — | P5 の解消。R5 の最低条件 |

選定を覆す場合は ADR (`docs/decisions/`) を必須とする。

### 3.4 ライセンス方針

- 運用スタック (車載で動き、将来配布し得る部分) は permissive (MIT/BSD/Apache) で
  構成可能な状態を保つ
- GPL 系 (FAST-LIO ファミリ等) は「オフラインのマップ作成ツール」としての
  分離プロセス利用に限定する。ソース改変・ヘッダ取込み・リンクをした成果物は
  GPL になることに注意
- `src/third_party/` 非同梱 (vcs import + gitignore) と
  「BSD-3-Clause 以外のコードのコピペ禁止」(code-reviewer チェック項目) を維持する
- 企業への成果物提供が具体化したら、third_party 各上流の LICENSE 棚卸し表を
  `docs/` に作成し、大学知財部門に確認する

### 3.5 プラットフォーム層の境界

- ROS 側に `whill_dispatch` パッケージ (仮称) を新設し、ゲートウェイノードが
  (a) 名前付き地点 (semantic waypoint) の解決
  (b) 配車ジョブのキュー管理 (受付・実行・キャンセル)
  (c) Nav2 `NavigateToPose` action の発行と進捗中継
  (d) 車両状態 (位置・バッテリ・走行状態) の publish
  を担う。Web 側はこの境界より先を知らない
- Web との接続は rosbridge_suite (websocket) を第一候補とする。
  認証・複数台管理が視野に入った時点で、FastAPI 等の独立ゲートウェイ API への
  置換を ADR で判断する
- タブレットアプリは別リポジトリでもよい。本リポの責務は API 境界までとする

## 4. マイルストーン再定義

旧 M5-d / M5-e は凍結し、以下に置き換える。各フェーズの詳細計画は
着手時に `pm-orchestrator` が本文書を入力として `docs/plans/` に作成する。

| フェーズ | 内容 | 解消する問題 | 主担当 agent |
|---------|------|------------|-------------|
| M4-R | odom 基盤再構築: robot_localization EKF (/whill/odom + IMU)、TF 再配線、base_link -> sensor の実測 extrinsic 反映、tf_bridge 廃止 | P4, P2 の一部 | ros2-implementer |
| M5-R | マップパイプライン: GLIM (または FAST-LIO SAM) 導入、ERASOR で動的除去、`docs/maps/<site>/` への成果物規約 (pcd + pgm + yaml + 取得メタデータ) | P5 (地図品質) | research-analyst -> ros2-implementer |
| M6-R | 運用 localization + Nav2 再統合: scan-to-map localizer 導入、initial pose 運用、フェイルセーフノード、obstacle layer 復活と collision detection 復帰 | P1, P2, P3, P5 (安全) | ros2-implementer + debugger |
| M7 | 配車 API 層: whill_dispatch (named waypoints, ジョブキュー, NavigateToPose wrapper, 状態 publish)、rosbridge 接続 | R6 | pm-orchestrator -> ros2-implementer |
| M8 | タブレット Web アプリ: 地図表示・目的地指定・呼び出し UI | R6 | (別リポ可) |
| M9 | 統合検証: 無人呼び出し走行を含む実機検証、E-stop / 遠隔停止の確認 | R4 | debugger + ユーザー実機 |

順序の理由: M4-R を最初に置くのは、odom フレームが無いままでは M6-R の補正導入が
コントローラへのジャンプ直撃になるため。各フェーズは単体で検証可能な単位に切ってある。

## 5. Claude Code への行動規範 (本方針下での絶対則)

禁止:

1. `tf_bridge_launch.py` の identity 構成を前提とした新機能の追加 (旧 M5-d の続行を含む)
2. FAST-LIO をランタイム localizer として強化する作業
   (パラメータ再調整はマップ作成品質の改善目的に限り可)
3. `use_collision_detection: false` のまま自律走行系の機能を増やすこと
4. 配車・Web 層のロジックを Nav2 / localization のノード内に密結合で書くこと
   (境界は 3.5 の通り whill_dispatch に置く)
5. (既存規約の再掲) `src/third_party/` の編集、GPL コードのコピペ

必須:

1. 各フェーズ開始時、`pm-orchestrator` が本文書を読み、phase / 受け入れ基準 /
   リスクを `docs/plans/` に展開してからユーザー承認を取る
2. 3.3 の選定を覆す・変更する判断は ADR に記録する
3. 受け入れ基準は観測可能なコマンドと期待値で書く (6 章の粒度を最低線とする)
4. 実機 (WHILL / Velodyne / RealSense / IMU) が必要な検証はユーザーに手渡す (既存規約)

## 6. 受け入れ基準 (フェーズ別・観測可能)

- M4-R:
  - `ros2 run tf2_tools view_frames` で `map -> odom -> base_link` の一本鎖になっている
  - `/odometry/filtered` が車輪 + IMU 由来で publish され、手押し 10 m 直進で
    終端誤差が許容内 (具体値は計画時に設定)
  - tf_bridge_launch.py が削除済みで、ビルド・launch が通る
- M5-R:
  - 同一始終点のループ走行 bag から生成した地図で、始終点が目視整合 (数十 cm 以内)
  - 歩行者が横切った bag でも、除去処理後の占有格子に「尾を引く」残像がない
  - `docs/maps/<site>/` に pcd / pgm / yaml / 取得日・経路・天候のメタデータが揃う
- M6-R:
  - RViz の initial pose 指定から localization が収束し、キャンパス経路 1 周で
    TF の飛び (閾値超えのジャンプ) がゼロ
  - 歩行者がセンサ前を横切っても自己位置が破綻しない (run3 相当条件の再現試験)
  - マッチングスコアを人為的に劣化させた試験で、フェイルセーフが cmd_vel を遮断する
  - obstacle layer が前方の人を costmap に反映し、`use_collision_detection: true` で走行
- M7:
  - websocket 経由で名前付き地点への配車ジョブを発行 -> 走行 -> 完了通知が返る
  - 走行中の位置・状態が Web 側から購読できる
- M8:
  - タブレット実機で「地点選択 -> 呼び出し -> 到着」の一連が操作できる
- M9:
  - 無人呼び出し走行が連続 N 回成功 (N は計画時に設定)、物理 E-stop と遠隔停止が機能する

## 7. 未決事項 (ADR 候補)

- [ ] ADR: マップ作成 SLAM の最終選定。GPU 母艦は確保済み (9 章) のため GLIM 採用の
      前提条件は満たされた。実 bag での GLIM vs FAST-LIO SAM 比較後に確定する
- [ ] ADR: localizer の最終選定 (lidar_localization_ros2 vs hdl_localization。
      リセット機構・初期位置 UX を評価軸に含める)
- [ ] ADR: Web 接続方式 (rosbridge 直結 vs 独立ゲートウェイ API) と認証方式
- [ ] ADR: 無人走行の安全要件 (物理 E-stop、遠隔停止、速度上限、監視者の要否。
      大学の安全審査プロセスとの接続を含む)
- [ ] ADR: 屋外拡張時の GNSS/RTK 統合 (屋根付き区間での切替方式を含む)

## 8. CLAUDE.md への反映 (accepted 後に実施)

1. 「アーキテクチャ層」の図を 3.1 / 3.2 の二相分離 + REP-105 構成に差し替える
2. 「進行中の既知課題」を P1〜P5 ベースに更新する
   (「FAST-LIO のループクロージャ不在」「車輪オドメトリ未統合」は本方針で解消経路が決まった旨を明記)
3. 「チーム体制」の表の先頭に「方針判断はまず本文書を参照」の 1 行を足す
4. Import 節に `@docs/plans/2026-06-11-platform-pivot.md` を追加する

## 9. 開発機材 (2026-06-11 確認)

Alienware x15 R2 (ホスト名 systemlab-Alienware-x15-R2):

- CPU: Core i9-12900H (14 コア 20 スレッド) / RAM: 32 GiB / SSD: 2 TB
- GPU: NVIDIA 搭載 (モデル名は `nvidia-smi` で要確認。GNOME Settings の表示が
  "NVIDIA Corporation / Mesa Intel Graphics" 止まりなのは、NVIDIA 専用ドライバが
  未導入か非アクティブな兆候。GLIM の GPU モードはドライバ + CUDA が前提)
- OS: Ubuntu 22.04.5 LTS — ROS 2 humble (jammy) の要件と一致

含意:

- スペック目安のティア B (車載運用) とティア C (マッピング母艦) を 1 台で兼用できる。
  GLIM 採用のハードウェア前提は満たされた。NVIDIA ドライバと CUDA のセットアップを
  M5-R の先行タスクとして計画に含めること
- ライブ運用時の /Odometry 1.4 Hz 問題は、この CPU では「スペック不足」ではなく
  「record + RViz + 全ドライバの同時実行負荷」という従来診断を裏付ける。
  運用 launch から record / RViz を外す方針を維持する
- 開発段階はこの 1 台で完結させる。省電力な車載専用機 (ミニ PC / Jetson) への
  分離は M9 以降の課題とし、いまは投資しない

研究室在庫の Jetson TX2 Developer Kit (2017。Pascal 256 CUDA cores /
Denver2 x2 + Cortex-A57 x4 / RAM 8 GB / eMMC 32 GB) は車載候補から除外する。
理由: ソフトウェア対応が JetPack 4 系 (Ubuntu 18.04, CUDA 10.2) で打ち止めのため、
ROS 2 humble (Ubuntu 22.04 必須) も GLIM の GPU 要件 (CUDA 12 系) も満たせない。
Docker で CPU-only humble を載せる抜け道はあるが、ARM 6 コア + 共有 8 GB で
localization + Nav2 を回すのは厳しく、EOL 基盤に工数を払う価値がない。
TX2 は教材・予備機の扱いとし、M9 で車載分離する場合は Orin 世代
(GLIM 公式検証済) か x86 ミニ PC を新規調達する。

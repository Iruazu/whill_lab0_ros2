# ADR 0011: 地面除去前処理 — Patchwork++ core + 自前 ROS 2 wrapper (M6-R)

Language: [日本語](0011-ground-removal-choice.md) | [English](../../en/decisions/0011-ground-removal-choice.md)

- Status: **accepted** (2026-07-14 bag replay 検証で AC1-AC3 PASS、AC4 は M6R4-3 走行で確定)
- Date: 2026-07-14 起草 / 2026-07-14 accepted
- Deciders: Iruazu

## 背景

M6R4-2 で Nav2 obstacle_layer に `pointcloud_to_laserscan` の 2D 輪切りを
食わせる経路を実装した (`whill_navigation/config/pointcloud_to_laserscan.yaml`,
[`../plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../plans/2026-07-14-m6r4-nav2-obstacle-layer.md))。
2026-07-14 の屋外 Phase B (工農研横) で構造的な限界が実測された:

- 輪切りは `base_link` 基準の水平面 (`min_height` = 定数) で切るため、
  勾配 5° 前後の路面と局所凸凹 (マンホール、轍、路面切れ目) が閾値を
  超えて lethal 化する
- `min_height` を -0.2 → 0.25 まで持ち上げると平坦部はクリーンだが、
  起伏路面では spike が残る (2026-07-14 実測)
- しきい値の上げ下げは trade-off でしかない: 上げれば低段差と
  crouched child (< 0.25 m) が見えなくなる

原因は「単一しきい値の輪切り」が terrain-flat な世界を前提としていること。
地形に追従する地面推定を入れれば根治する。

## 決定

3 部構成の決定:

### 1. アルゴリズム: **Patchwork++** (Urban Robotics Lab @ KAIST, IROS 2022)

Concentric Zone Model (CZM) で点群を距離帯 × 角度セクタに分割し、パッチ
ごとに Region-wise Vertical Plane Fitting + Adaptive Ground Likelihood
Estimation (A-GLE) で局所地面を推定する。単一平面フィット (RANSAC) では
原理的に届かない「勾配 + 局所凸凹」に対して、パッチ単位で地面レベルが
再推定される設計が要件に一致する。

### 2. コード採用範囲: BSD-2-Clause の C++ core のみ

上流リポ `url-kaist/patchwork-plusplus` v1.4.1 は以下のライセンス構成:

| 場所 | 記載 |
|------|------|
| root `LICENSE` | BSD 2-Clause (KAIST, 2024) — C++ core `cpp/` 用 |
| `ros/LICENSE` | MIT (KISS-ICP 系 authors, 2022) |
| `ros/package.xml` | **GPL-3.0** |
| `ros/src/*.cpp` | ライセンスヘッダなし |

`ros/LICENSE` と `ros/package.xml` の記載が矛盾する上流バグを、機械可読の
`package.xml` 側を尊重して GPL-3.0 と解釈する。本リポの
[`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §3.4
「運用スタックは permissive で、GPL 系はオフラインのマップ作成ツール限定」
方針に従い、**`ros/` サブツリーは採用しない**。

BSD-2-Clause の `cpp/` core (Params 構造体と PatchWorkpp クラス) のみを
CMake `add_subdirectory` で取り込み、リンクする。core は Open3D 不要
(examples が INCLUDE_CPP_EXAMPLES ON の時のみ)、Eigen3 と optional TBB
のみで済む。

### 3. ROS 2 wrapper: 自前実装 (`whill_perception` パッケージ)

`src/whill_perception/` を新設し、BSD-3-Clause で PatchWorkpp をラップする
薄い node (~200 行) を書く。契約:

```
sub  cloud_in         sensor_msgs/PointCloud2  (VLP-16, SensorDataQoS)
pub  cloud_no_ground  sensor_msgs/PointCloud2  (~10 Hz, xyz only、
                                                 同一 header frame_id)
```

パラメータは全て `config/patchworkpp.yaml` に外出し。`sensor_height`
(0.79 m、M4-R 静的 TF 実測値) と `min_range` / `max_range` の 3 つのみを
struct default から override、それ以外は Patchwork++ の default を維持
(KITTI 64ch tuning がベースだが VLP-16 で既定値の性能検証は未実施)。

### 4. パイプライン統合

```
/velodyne_points  ──▶ patchworkpp_node  ──▶ /velodyne_points_no_ground
                                                    ▼
                       pointcloud_to_laserscan_node  ──▶ /scan
                                                              ▼
                                                    obstacle_layer
                                                    (Nav2)
```

M6R4-1+2 (PR #81, session B が M6R4 branch で作業中) が merge され次第、
downstream の `p2ls_node` の subscribe を `/velodyne_points` から
`/velodyne_points_no_ground` に切り替える 1 行 remap 変更 PR を続けて
出す (本 ADR で決定する対象外、統合フェーズの follow-up)。

## 代替案

### 代替 A: 上流 ROS 2 wrapper (`patchworkpp_node` in ros/) をそのまま採用

- **利点**: 実装工数ゼロ、上流と同じ configuration
- **欠点**: ライセンスが GPL-3.0 と表明。運用スタックへの初の GPL 依存
  になり、plan §3.4 の明示的方針に抵触
- **却下**: 学内研究目的の一時採用でも「新規先例」を作ることの重み。
  MIT 表明されている `ros/LICENSE` に依拠して MIT 解釈で使う手もあるが、
  上流バグを他人任せで解決する道は保守可能性が低い

### 代替 B: Autoware `scan_ground_filter` (Apache-2.0)

- **利点**: ライセンス clean、勾配パラメータが明示的
  (`global_slope_max_angle_deg`, `local_slope_max_angle_deg`)、
  autoware 車両で運用実績豊富
- **欠点**: autoware_universe monorepo からの切り出しになり、
  `pcl_msgs` や autoware 固有ユーティリティへの依存が付随する。
  本リポは autoware 生態系を他で使っていないため、依存管理コストが
  Patchwork++ 単体パッケージより明らかに重い
- **将来切替条件**: Patchwork++ 実測で VLP-16 100 ms 超え or CPU 予算
  超過。この場合 monorepo 切り出しのコストを受け入れて移行

### 代替 C: linefit_ground_segmentation_ros2 (BSD-3-Clause)

- **利点**: 旧 whill_lab0 (noetic) で継承使用実績、依存軽い、ROS 2 port 存在
- **欠点**: ROS 2 port は個人リポで star 20、単独開発者、活動規模小。
  アルゴリズム自体もセクタ単位のライン fit で、A-GLE ほど局所凸凹に強く
  ない
- **将来切替条件**: Patchwork++ の Open3D 依存 (実際は examples 用のみ
  だが、build system 経由で紛れ込む可能性) が本機の CUDA と衝突した場合

### 代替 D: RANSAC 単一平面 (`pcl_ros SACSegmentation`)

- **利点**: apt 一発、実装最速、依存皆無
- **欠点**: 単一グローバル平面が前提。5° 勾配 + 局所凸凹の混在で「勾配
  側に寄せるか凸凹側を見逃すか」の二択となり、要件を構造的に満たさない。
  今回の 2D 輪切り破綻と同じ根本原因 (単一しきい値/単一平面) を共有する
- **却下**: デモ日程が近い (2026-08-01) 中で「速いが要件を満たさない」案は
  技術負債を積むだけ

## 結果

- **build 系**: `src/third_party/patchwork_plusplus/` を vcs import 後に
  `touch src/third_party/patchwork_plusplus/ros/COLCON_IGNORE` する運用が
  必要。README に明記済 (`src/whill_perception/README.md`)。vcs import の
  ラッパースクリプトを書く場合はそこにも仕込む
- **CPU 予算**: Alienware x15 R2 で 10 Hz 実測は本 PR の bag replay
  検証で確定する。Patchwork++ の SemanticKITTI 64ch レイテンシは 13-48 ms
  (競合手法論文の比較表、バイアス注意)。VLP-16 は点数 1/4 なので余裕見込み
  だが、直接根拠なし
- **downstream 統合**: M6R4-1+2 (PR #81) merge 後、`p2ls_node` の subscribe
  を `/velodyne_points_no_ground` に切り替える follow-up PR (本 ADR の
  scope 外) が必要
- **min_height 見直し**: 地面除去が入れば slice の `min_height` を再度
  緩めて低段差 / 人の脚を捕獲する方向にチューニングし直せる。ADR-0009
  §検証結果 2026-07-15 A/B で 0.05 に確定 (実地縁石は 5 cm 前後で本層の
  検出対象外という運用も併せて確定)
- **上流ライセンス issue 報告**: `url-kaist/patchwork-plusplus` に対して
  `ros/LICENSE` (MIT) と `ros/package.xml` (GPL-3.0) の矛盾を報告する PR /
  Issue を出すのが upstream fix として妥当。本 ADR 範囲外だが follow-up

## Accept 化条件

- **AC1** (build & runtime): `colcon build --packages-select whill_perception`
  通過 (現状 PASS 済)、`ros2 topic hz /velodyne_points_no_ground` が
  VLP-16 実機 or bag replay で 9-11 Hz を 30 s 安定
- **AC2** (visual): RViz で `/velodyne_points` (raw) と
  `/velodyne_points_no_ground` を並置して、地面点が消え建物・柱・人が
  残ることを目視確認
- **AC3** (CPU): `top -p <patchworkpp_node pid>` で単一 core 80% 以内
- **AC4** (走行): M6R4-3 実機走行で `/local_costmap/costmap` の
  false-lethal spike が Phase B 実測 (2026-07-14) 比で有意に減少

AC1-AC3 は bag replay で成立可能。AC4 は M6R4-3 の V4 完了時に併せて
判定する。全 pass で本 ADR を accepted に昇格。

## 検証結果 (2026-07-14 bag replay)

`docs/m6r-bench-data/2026-07-14-verify-campus/bag/` を再生した本 branch
(commit `ceb3bb3`) の実測:

| AC | 判定 | 実測値 |
|----|------|--------|
| AC1 build | PASS | `colcon build --packages-select whill_perception` 7.16 s、stderr なし |
| AC1 rate  | PASS | `/velodyne_points_no_ground` = **9.857 Hz** (window 100, 30 s+, std dev **0.0004 s**) |
| AC2 visual| PASS | RViz 目視でアスファルト・ランプ・マンホールの ring 消失、建物・柱・歩行者が残存 (スクショ取得済) |
| AC3 CPU   | PASS | `patchworkpp_node` = **2.3% CPU 全体 / 1.6-2.7 ms per frame** (単一 core 80% headroom 十分) |
| AC4 drive | 未確定 (M6R4-3 走行で判定) | — |

代表 frame の内訳 (bag 中盤): `in 29184 pts / ground 8047 / non-ground 21137`。ground 比率 **26-30%** は M6R4-b 全体を通して安定。

**AC1-AC3 が pass したので本 ADR を accepted に昇格した** (Status 参照)。
AC4 は M6R4-3 の V4 で判定するが、下流の p2ls `min_height` を 0.05 に
緩められることは ADR-0009 §検証結果 2026-07-15 A/B で確認済。

### 低マウントでの ground 比率について

ground 比率 26-30% は WHILL の低マウント (`sensor_height = 0.79 m`) では
構造的に正常。VLP-16 の 16 ring は水平面 ±15° に分布しており、マウントが
低いほど地面ヒットする ring 数が減る (地面までの立体角が小さい)。Autoware
車両の高マウント (~1.9 m) では ground 比率が 60-70% に達するのが標準だが、
本車両でその値を目指すと非地面点を過剰に落とすことになる。将来の読者が
この差分を異常と誤解しないよう明記しておく。

### Silent-failure 顛末 (2 件)

M6R4-b の実装過程で 2 件の silent failure を踏み、いずれもガード + 環境
恒久化で再発を潰した。次に類似ノードを書く人向けの記録として残す。

1. **RNR intensity 列欠落** (2026-07-14 bag replay 1 回目)。Patchwork++
   core は RNR 有効時に入力の 4 列目 (intensity) を必須とし、無い場合
   stdout に `RNR requires intensity information !` を出して frame を弾く。
   初回実装は Nx3 (xyz のみ) だったため入力全 frame が rejected、
   `/velodyne_points_no_ground` は publish されるが getGround / getNonground
   両方 0 点。RViz では「何も見えない」だけで、ROS layer には ERROR も
   WARN も来なかった。修正 (`ceb3bb3`): (a) 変換を Nx4 に、(b) 入力に
   `intensity` field が無ければ WARN_THROTTLE、(c) `ground.rows() == 0 &&
   nonground.rows() == 0` を出力後に検知して WARN_THROTTLE。core stdout の
   単一 print に依存しない ROS 側のガードを 2 段入れた。

2. **UDP フラグメント損失** (2026-07-14 bag replay 2 回目)。大型 PointCloud2
   の bag replay は DDS の default UDP recv buffer で溢れ、断片的にフレーム
   ロスが出た。実機ドライバは burst が緩いので問題化しなかったが、bag
   player は intended send rate の何倍もの瞬発を出すため露見した。
   `/etc/sysctl.d/60-ros2-dds-buffer.conf` で `net.core.rmem_max` / `rmem_default`
   を 25 MB に恒久化して解消。M6R4-3 の bag record 併用走行でも同リスクが
   あるため、同 sysctl が入っていないホストで本ノードを回すときは要注意。
   本 ADR は「ノード側の障害検知」を主目的にする範囲を超えるが、環境要件
   としてここに記録する (詳細は `src/whill_perception/README.md` の
   Environment 節を参照)。

## 関連

- [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §3.4 (ライセンス方針)、§4 M6-R
- [`../plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
  §"Follow-up: ground removal" (M6R4-b の位置付け)
- ADR-0009 (session B が draft 中、p2ls パラメータ選定): 本 ADR の後段
  で `min_height` の再チューニングと連動
- 上流: <https://github.com/url-kaist/patchwork-plusplus> (v1.4.1, BSD-2-Clause)
- 論文: <https://arxiv.org/abs/2207.11919>

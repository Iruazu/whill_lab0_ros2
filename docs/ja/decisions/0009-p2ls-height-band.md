# ADR 0009: pointcloud_to_laserscan の高さ帯選定と QoS bridge (M6-R)

Language: [日本語](0009-p2ls-height-band.md) | [English](../../en/decisions/0009-p2ls-height-band.md)

- Status: **accepted** (2026-07-15 A/B 実測で min_height 0.05 確定、AC1-AC5 全て PASS)
- Date: 2026-07-14 (起草) / 2026-07-15 (accepted)
- Deciders: Iruazu

## 背景

Nav2 obstacle_layer は sensor_msgs/LaserScan を reliable QoS で購読する。
一方 VLP-16 の `/velodyne_points` は sensor_msgs/PointCloud2 の best-effort
sensor QoS。この直接接続は QoS 不一致で成立しないため、`pointcloud_to_laserscan`
(以下 p2ls) を bridge として挟む必要がある。M5-c コメントに「obstacle_layer なし」
と書かれていた根本原因もこの不整合。

p2ls は 3D 点群を base_link 基準の水平面帯 `[min_height, max_height]` で
切り出して 2D LaserScan にする。**この輪切りが構造的に扱える範囲**を定め、
将来の起伏路面対策 (ADR-0011: Patchwork++ 地面除去) と役割分担するのが
本 ADR の目的。

## 決定

`src/whill_navigation/config/pointcloud_to_laserscan.yaml` に以下を確定する。

### 1. 座標系とフィルタ帯

| パラメータ | 値 | 根拠 |
|-----------|-----|------|
| `target_frame` | `base_link` | Nav2 costmap の `robot_base_frame` と一致 |
| `transform_tolerance` | 0.1 s | M4-R EKF 30 Hz + M6R-2 localizer 10 Hz を許容する下限 |
| `min_height` | **0.05 m** (2026-07-15 A/B 確定値) | Patchwork++ (ADR-0011) が地面を上流除去した状態で 0.05 vs 0.10 の 2 系統 p2ls を屋外並列比較。両値とも起伏路面 (マンホール、轍) で偽ヒット 0。0.05 の方が人の脚を常時 ~4 点捕捉し costmap 用途に十分。0.10 は情報量が減るだけで安全マージン増にならないため 0.05 採用 (下記 §検証結果 2026-07-15 A/B) |
| `max_height` | 1.6 m | 立位人物 (torso + 頭部) を含め、屋内 bringup 時の天井 return を無視する上限 |
| `range_min` | 0.5 m | WHILL 車体上面の自己反射除去 (LiDAR 原点から 0.5 m 圏内) |
| `range_max` | 25.0 m | obstacle_layer の `raytrace_max_range` と一致。片方短いと通過後の clearing が塗り忘れる |

### 2. 角度分解能

| パラメータ | 値 | 根拠 |
|-----------|-----|------|
| `angle_min` / `angle_max` | -π / +π | 全周走査 |
| `angle_increment` | 0.00873 rad (0.5°) | VLP-16 の水平分解能 (10 Hz 回転) に一致。細かくすると同じ ring を複数 bin に散らす |
| `scan_time` | 0.1 s | 10 Hz 出力の逆数 |
| `use_inf` | true | 遠方 clearing に `+inf` を吐かせて obstacle_layer の raytrace が正しく通過認識 |
| `inf_epsilon` | 1.0 | `use_inf` 用のマージン (nav2_costmap_2d 側で `+inf` を扱う既定値) |

### 3. QoS

- p2ls の subscribe `/velodyne_points`: best-effort (SensorDataQoS)。upstream
  velodyne driver 側が reliable publish しても downgrade 受入可能
- p2ls の publish `/scan`: **reliable** (nav2 humble ObstacleLayer が期待する
  default に合わせる)。QoS override は不要 (p2ls 標準出力が reliable)

### 4. `/scan` の単一 publisher 化

VLP-16 upstream launch (`velodyne-all-nodes-VLP16-launch.py`) は
`velodyne_laserscan_node` を含み、これが素の 3D 点群を高さフィルタなしで
`/scan` に垂れ流す。**height フィルタが無いため obstacle_layer に食わせる
べきではない**。M6R4-2 で p2ls を追加すると `/scan` の dual publisher に
なり、両 costmap が両方の入力を混合する (19.7 Hz = 9.86 × 2、2026-07-14
Phase B 実測)。

`whill_sensors_bringup/launch/sensors_launch.py` で `GroupAction +
SetRemap('/scan' → '/scan_raw')` で velodyne 側の出力を rename。p2ls が
出す `/scan` のみが obstacle_layer に到達する経路に整流する (`2f26d0b`)。

## 代替案

### 代替 A: `min_height` を下げて 5 cm 級の低段差を捉える

- **却下 (Patchwork++ 前)**: 2026-07-14 Phase B で -0.2 が起伏路面で false
  positive を大量生成することを実測。単一しきい値の輪切りが terrain-flat
  な世界を前提としている以上、`min_height` の上げ下げは trade-off でしかない
- **代替経路**: ADR-0011 (Patchwork++ 地面除去) で地面自体を上流除去する
  ことで、輪切りの `min_height` を再度緩められる — 2026-07-15 A/B で確認
- **Patchwork++ 後の限界 (2026-07-15 A/B)**: 実地の縁石は想定 12-15 cm では
  なく実測 5 cm 前後で、これは 0.05 のカットライン以下。さらに下げると
  路面凸凹との分離が原理的に不可能になる (Patchwork++ は「地面 vs 非地面」
  の二値判定で、地面すれすれの 5 cm 段差は「地面」側に分類される)。
  この段差帯 (~5 cm) は本層では検出対象外とし、車体走破可 + 経路設計側
  (map annotation / operator 判断) で対処する運用に確定した

### 代替 B: velodyne_laserscan_node の `/scan` をそのまま使う

- **却下**: 高さフィルタ非対応。天井 return と地面 return を無条件で拾い、
  obstacle_layer が誤 lethal を大量生成する
- upstream の `/scan` は診断用途として `/scan_raw` へ rename (§4 参照)

### 代替 C: `nav2_costmap_2d::VoxelLayer` へ移行 (3D 判定)

- **却下 (現時点)**: Nav2 obstacle_layer の代替として voxel_layer を採用する
  と `min_height` / `max_height` の 2D スライスが 3D grid に置き換わり、
  傾斜追従できる可能性がある
- ただし computational cost が上がる (Alienware x15 R2 で 10 Hz 保証未確認)、
  かつ設定パラメータが大幅増。M7 (`whill_dispatch`) 以降で ADR 別立てで検討

### 代替 D: 独自の QoS bridge を書く

- **却下**: p2ls は BSD-3-Clause の apt package (`ros-humble-pointcloud-to-laserscan`)
  で即入手可能。自前 bridge を書くのは車輪の再発明。ただし将来
  「Patchwork++ 出力 (best-effort) → obstacle_layer (reliable)」の bridge が
  必要になれば voxel_layer 移行か p2ls をもう 1 段挟む選択肢がある

## 結果

- **5 cm 級の低段差 (実地縁石を含む) は本層では検出対象外**。ADR-0011 の
  地面除去 + 本 ADR の輪切り 0.05 m という組み合わせでは原理上分離不能
  なため、routing 側 (map annotation / operator 判断) で対処する
- **人の脚 (立位・歩行)** は 0.05 m カットで常時 ~4 点捕捉、`local_costmap`
  で lethal 化する — costmap 用途に十分な情報量 (2026-07-15 A/B 実測)
- **背の高い雑草** は両値で lethal 化する。コード側で分離不能なため、
  デモ前の経路整備 (刈る / 迂回路選定) を [デモ準備チェックリスト](../m6r-demo-prep-checklist.md)
  に運用手順として残す
- **operator-in-the-loop 前提**: ADR-0007 §Demo-scope reduction と併せて、
  デモは operator 随伴・ジョイスティック介入可能な運用で成立させる
- **`/scan` の単一 publisher 化**は本 ADR で確定 (velodyne_laserscan_node
  → `/scan_raw`)。M6R4-b でも同じ経路を使うため、`/scan` = p2ls の出力
  という契約は変わらない

## 検証結果 (2026-07-14 Phase B, 工農研横)

`f110f2f` (min_height = 0.25) 状態での屋外実測:

| 項目 | 結果 |
|------|------|
| Phase A/T1 | PASS: 6 ノード全 active (initialpose 投入後 27 秒で costmap on_activate 完了、実測) |
| Phase A/T3 | PASS: `/alignment_status` fitness 0.0124-0.027 (has_converged: true)、閾値 6.0 比 200-480 倍 |
| `/scan` 単一系 | PASS: 9.86 Hz (2f26d0b の SetRemap 有効確認済) |
| 3 層 costmap 稼働 | PASS: static + obstacle + inflation が RViz で表示 |
| RViz 目視 (静止) | 平坦部は clearing donut 正常 |
| U3-U6 (人立たせ) | **中断**: 起伏路面部で min_height 0.25 でも spike が残り、人物 obstacle との弁別困難。ADR-0011 の Patchwork++ 地面除去 landing 後に再開 |

**U3-U6 の中断は本 ADR の暫定値の限界であって、bridge 統合そのものの
失敗ではない**。ADR-0011 が accepted (2026-07-14) となり、Patchwork++
出力を p2ls 入力に差し替える M6R4-c PR が landing する 2026-07-15 A/B
で min_height 再チューニングと合わせて U3-U6 相当条件が成立した (下記)。

## 検証結果 (2026-07-15 A/B, 屋外)

Patchwork++ (ADR-0011 accepted) 経由の `/velodyne_points_no_ground` を
入力とした p2ls を、`min_height = 0.05` と `0.10` の 2 系統並列で起動
して同一シーンを同時観測。

| 観測項目 | 0.05 | 0.10 | 採用理由 |
|---------|------|------|---------|
| 勾配 + 盛り上がりマンホール上の偽 lethal | **0** | **0** | Phase B 偽 obstacle の根治を実機 confirm。Patchwork++ の地面除去が両値の下限マージンを吸収 |
| 人 (立位、前方 2 m) の脚捕捉数 | **~4 点常時** | 減 | costmap 用途に十分な情報量。0.10 は情報量が減るだけで安全マージン増にはならない |
| 縁石 (実測 ~5 cm) 検出 | 未検出 | 未検出 | 設計どおり (§代替 A 参照)。5 cm カットライン以下、Patchwork++ が「地面」側に分類 |
| 背の高い雑草 | lethal | lethal | 両値で問題化。コード対処不可、経路整備で対処 |

**結論**: `min_height = 0.05` を確定採用。

## Accept 化条件 — 全て PASS

- **AC1** (build & QoS): `colcon build --packages-select whill_navigation
  whill_sensors_bringup` 通過 (PASS)、`ros2 topic info /scan` publisher
  count = 1 (SetRemap 有効)、`ros2 topic hz /scan` 9-11 Hz 30 s 安定 (PASS)
- **AC2** (Phase A/T1-T5): 2026-07-14 実測で PASS
- **AC3** (Phase B/T3-T4): 2026-07-14 実測で PASS
- **AC4** (U3-U6, 人立たせ obstacle): 2026-07-15 A/B で PASS。人の脚が
  local_costmap に描画され、退去で clearing が動くのを目視確認
- **AC5** (min_height re-tune): 2026-07-15 A/B で PASS。0.05 で false-lethal
  spike が Phase B 2026-07-14 ベースラインから 0 に落ち、人脚 obstacle は
  維持

## 関連

- [`../plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
  §3 M6R4-2 (p2ls パラメータの一次記録)
- ADR-0011 (accepted 2026-07-14): 地面除去前処理。本 ADR の min_height
  再チューニングの前提条件
- 上流 `ros-humble-pointcloud-to-laserscan` (BSD-3-Clause, apt 提供)
- 2026-07-14 実測: `docs/m6r-bench-data/2026-07-14-verify-campus/`

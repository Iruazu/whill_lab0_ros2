# ADR 0009: pointcloud_to_laserscan の高さ帯選定と QoS bridge (M6-R)

Language: [日本語](0009-p2ls-height-band.md) | [English](../../en/decisions/0009-p2ls-height-band.md)

- Status: **proposed** (2026-07-14 起草、M6R4-b (ADR-0011) landing 後の min_height 再チューニングで accepted 化)
- Date: 2026-07-14
- Deciders: Iruazu (承認待ち)

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
| `min_height` | **0.25 m** (M6R4-b landing 後は再チューニング) | 2026-07-14 Phase B で -0.2 が路面凸凹 (マンホール、轍) を lethal 化するのを実測。0.25 に上げて平坦部はクリーンだが起伏路面では spike が残る (下記 §結果) |
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

### 代替 A: `min_height` を下げて curb (~0.15 m) を捉える

- **却下**: 2026-07-14 Phase B で -0.2 が起伏路面で false positive を大量
  生成することを実測。単一しきい値の輪切りが terrain-flat な世界を前提と
  している以上、`min_height` の上げ下げは trade-off でしかない
- **代替経路**: ADR-0011 (Patchwork++ 地面除去) で地面自体を上流除去する
  ことで、輪切りの `min_height` を再度緩められる

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

- **curb (~0.15 m) と crouched child (< 0.25 m) は本 ADR の暫定値では
  掴まない**。M6R4-b (ADR-0011) landing 後に `min_height` を curb 捕獲方向
  (~0.05 m) に緩め直す — その時点で本 ADR を accepted に昇格
- **operator-in-the-loop 前提**: ADR-0007 §Demo-scope reduction と併せて、
  デモは operator 随伴・ジョイスティック介入可能な運用で成立させる
- **`/scan` の単一 publisher 化**は本 ADR で確定 (velodyne_laserscan_node
  → `/scan_raw`)。M6R4-b でも同じ経路を使うため、`/scan` = p2ls の出力
  という契約は変わらない
- **downstream 統合**: ADR-0011 accept 化と同時に、`pointcloud_to_laserscan_node`
  の `cloud_in` remap を `/velodyne_points` → `/velodyne_points_no_ground` に
  切り替える 1 行 PR を出す (本 ADR scope 外の follow-up)

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
出力を p2ls 入力に差し替える follow-up PR が landing すれば min_height
再チューニングと合わせて U3-U6 は成立する見込み。

## Accept 化条件

- **AC1** (build & QoS): `colcon build --packages-select whill_navigation
  whill_sensors_bringup` 通過 (現状 PASS 済)、`ros2 topic info /scan` の
  publisher count = 1 (SetRemap 有効の確認)、`ros2 topic hz /scan` が
  9-11 Hz を 30 s 安定
- **AC2** (Phase A/T1-T5): 2026-07-14 実測で PASS 済
- **AC3** (Phase B/T3-T4): 2026-07-14 実測で PASS 済
- **AC4** (U3-U6, 人立たせ obstacle): ADR-0011 landing + p2ls remap flip
  follow-up 後に再確認。RViz で人物 obstacle が local_costmap に描画され、
  退去で clearing が動くこと
- **AC5** (min_height re-tune): AC4 成立後、min_height を 0.05 m 程度に
  緩め直しても false-lethal spike が Phase B 2026-07-14 実測ベースライン
  比で有意に減少 (ADR-0011 AC4 と併せて判定)

AC1-AC3 pass、AC4-AC5 は ADR-0011 と連動して follow-up 検証。

## 関連

- [`../plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
  §3 M6R4-2 (p2ls パラメータの一次記録)
- ADR-0011 (accepted 2026-07-14): 地面除去前処理。本 ADR の min_height
  再チューニングの前提条件
- 上流 `ros-humble-pointcloud-to-laserscan` (BSD-3-Clause, apt 提供)
- 2026-07-14 実測: `docs/m6r-bench-data/2026-07-14-verify-campus/`

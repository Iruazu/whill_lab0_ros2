# M6-R デモ準備チェックリスト

Language: [日本語](m6r-demo-prep-checklist.md) | [English](../en/m6r-demo-prep-checklist.md)

デモ本番前に**現地で**実施する運用手順。コード側で扱えない
経路依存の問題を、事前踏査と物理的な経路整備で吸収するためのリスト。

デモ形態が変わったら (会場変更、季節変更、コース変更) 再走査すること。
現時点の適用範囲は M6-R 統合デモ (キャンパス外周)。

## 経路整備

### 背の高い雑草の刈り取り / 迂回

- **理由**: `min_height = 0.05 m` (ADR-0009 accepted) と Patchwork++ 地面
  除去 (ADR-0011 accepted) の組み合わせでは、地面から立ち上がる形状は
  すべて obstacle として lethal 化される。人と雑草を分離する情報がない
- **手順**:
  - デモ 1-2 日前にコースを踏査、腰高 (~50 cm) 以上の雑草群を特定
  - 経路 lethal 化を避ける方法: (a) 刈り取り (b) map annotation で
    迂回経路を敷く (c) `raytrace_max_range` 内で回り込ませる
- **記録**: 対応済み雑草 patch の位置と対応方法を `docs/m6r-bench-data/
  <demo-date>-<site>/route-grooming.md` に残す

### 5 cm 級の路面段差

- **仕様上の扱い**: 検出対象外 (ADR-0009 §結果)。車体走破可
- **踏査項目**: WHILL の走破可能限界を超える段差 (~10 cm 以上) が新規に
  現れていないかを目視確認。ある場合は map annotation で迂回路を用意

## デモ当日の起動前チェック

環境前提は [CLAUDE.md §ランタイム環境の前提](../../CLAUDE.md) を参照
(RMW, CPU governor, sysctl, NVIDIA suspend fix)。ここでは M6-R 統合
デモに固有の項目のみを列挙する。

### 起動手順 (bringup は 1 terminal のみ)

```
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
```

これが sensor drivers + WHILL driver + M4-R EKF + M6-R localizer +
safety layer を全て起動する。`sensors_launch.py` や
`odom_bringup_launch.py` を並行起動しないこと (2026-07-16 field で
全ノード二重化、`/velodyne_points` 39.4 Hz、RealSense USB contention
loop、AC4 中断)。詳細は `src/whill_safety/README.md` §Mutual exclusion。

### 検証チェック (bringup 起動 ~20 秒後)

- [ ] **ノード重複ゼロ** (必須): `ros2 node list | sort | uniq -c | sort -rn | head`
      で全 count = 1。`2 /velodyne_driver_node` 等が出たら並行起動している
      → 余分な launch を止めてから AC 実施
- [ ] **/velodyne_points が 10 Hz**: 本環境では `ros2 topic hz` が受信ゼロに
      なることがある (2026-07-19 field 確定) ため echo カウントで測る:
      `timeout 6 ros2 topic echo /velodyne_points --field header.frame_id | grep -c -- ---`
      → 40-55 (≈10 Hz×4-5 秒分) なら正常。80 超は duplicate bringup の兆候
- [ ] **localizer の odom 拘束が配線されている** (Issue #108 の再発防止):
      `ros2 node info /lidar_localization` の Subscribers に
      `/odometry/filtered: nav_msgs/msg/Odometry` が出ること。加えて
      `timeout 6 ros2 topic echo /odometry/filtered --field header.frame_id | grep -c -- ---`
      → 120 以上 (≈30 Hz) であること
- [ ] `map -> odom -> base_link` の TF chain が 1 本鎖 (`ros2 run
      tf2_tools view_frames`)
- [ ] `/alignment_status.has_converged: true` かつ `fitness < 1.0`
      (静止状態、初期位置合わせ後)
- [ ] `/scan` の publisher count = 1 (velodyne_laserscan の
      `/scan_raw` remap 有効): `ros2 topic info /scan`
- [ ] operator 随伴、ジョイスティック介入可能 (ADR-0007 §Demo-scope
      reduction)

### 走行前 gate — `scripts/m6r_preflight.sh` (blocking、必須)

2026-07-16 late incident (silent QoS mismatch で Layer D 無音、接触)
以降、走行前 gate は **blocking script 経由**を必須化。

```bash
scripts/m6r_preflight.sh
```

exit 0 まで **絶対に goal を発行しない**。exit 1 なら script が指す
原因 (failsafe_node 未起動 / DEAD INPUT / /cmd_vel_safety 未 publish)
を先に潰す。script の中身は:

1. `use_collision_detection: true` の実効値
2. `/failsafe_node` alive
3. dead-input watchdog 経路: 12 秒待って `/rosout` に `DEAD INPUT`
   がないこと (subscription が届いていない layer があれば失敗)
4. Live-fire hand test: 手を chair 前方 1.5 m に翳し、
   `/cmd_vel_safety >= 15 Hz` publish

### 走行前 gate: use_collision_detection + Layer D armed (必須)

Layer D (前方扇形 perception gate、ADR-0007 §Layer D proposed) が active
であること、および `use_collision_detection: true` が effective に
反映されていることを**走行前に必ず**確認する:

```bash
# collision_detection の effective 値
ros2 param get /controller_server FollowPath.use_collision_detection
# 期待: Boolean value is: true

# Layer D armed の startup log
ros2 topic echo /rosout | grep -E "failsafe_node ready|forward_blocked"
# 期待: "forward_blocked > 5 pts in ±30° @ 0.5-2.0 m, hysteresis 0.5s"

# 動作テスト (手を chair 前方 1 m 距離に翳して 2 秒)
ros2 topic hz /cmd_vel_safety
# 期待: 遮断中は 20 Hz publish、手を外すと publish 停止
```

3 つとも期待通りでなければ demo 開始してはならない (V2 停止要件が
成立していない状態)。

### マップ variant 選択 (Task #13 salt cleanup)

`docs/maps/campus/occupancy.pgm` は M5-R 時代 (Patchwork++ 導入前) の
地面ノイズを salt として焼き込んでいる (2026-07-16 field 立証)。demo
本番では salt を除去した cleaned 版を使う:

```
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=cleaned
```

初回起動時に `/map` を RViz OccupancyGrid で表示し、traversed 経路上の
黒 salt が消えていることを目視確認する (cleaning_diff.png と照合)。

### 配車 UI (whill_dispatch, M7) — 3 段目の terminal

M7 以降のデモは、RViz で initial pose を打った後の goal 発行を CLI では
なく**タブレットの Web UI** から行う。bringup (terminal A) + Nav2
(terminal B) に加えて 3 段目を起動する:

```
# terminal C (bringup / Nav2 とは別。TF も cmd_vel も足さないので非干渉)
ros2 launch whill_dispatch dispatch_launch.py use_mock:=false
```

`use_mock:=false` は terminal B の Nav2 `bt_navigator` を実
`/navigate_to_pose` server として使う (mock は実機なし検証専用)。起動後の
確認:

- [ ] `ros2 node list | sort | uniq -c` で `dispatch_node` /
      `rosbridge_websocket` を含め全 count = 1 (bringup 二重化と同様に
      dispatch も 1 本のみ)
- [ ] `ss -ltnp | grep -E '9090|8000'` で ws (9090) と http (8000) が LISTEN
- [ ] タブレットのブラウザで `http://<host>:8000` を開く (同一 LAN・非 TLS)。
      ヘッダが「接続済み」、map 背景 + 地点マーカ + 目的地ドロップダウンが出る
- [ ] `scripts/m6r_preflight.sh` exit 0 を確認**してから** UI で地点を選び
      「配車」を押す (走行前 gate は M7 でも不変)。progress バーが進み、到着で
      状態 SUCCEEDED
- [ ] 地点座標は現地実測で差し替え済みか: `docs/maps/campus/waypoints.yaml`
      の x/y/yaw が placeholder (0.0 等) のままなら、各地点に WHILL を運び
      `/pcl_pose` を実測して差し替え → dispatch_launch を再起動

Web UI は Nav2 の `/navigate_to_pose` を直接叩かない。goal 発行も車両位置の
配信も dispatch_node が境界として一元化する (platform-pivot §5 #4)。詳細は
[`../../src/whill_dispatch/README.md`](../../src/whill_dispatch/README.md)。

### RealSense (opt-in、通常 off)

D435 は M6-R runtime stack が消費していない。USB 2.1 認識問題があるため
起動対象から外している (`sensors_launch.py` の `realsense` arg default
false)。camera-specific test を意図的に走らせるときのみ `realsense:=true`
を bringup コマンドに付与。付与時は改めて USB 点検 (`lsusb` で D435
検出 + `/dev/bus/usb/` の権限) をチェックリストに追加すること

### 走行 bag の録画 (再現解析のため必須)

2026-07-19 の Issue #108 (reject 連鎖 → `map -> odom` 凍結 → Nav2 abort) は
bag が無く replay 検証ができなかった。以降、走行は必ず bag を録画する。
録画端末のみ bag 用 DDS xml に切り替える (`~/.bashrc` は runtime xml のまま):

```bash
export CYCLONEDDS_URI=file:///home/systemlab/whill_lab0_ros2/configs/cyclonedds-bag-record.xml
ros2 daemon stop && ros2 daemon start
ros2 bag record \
  /velodyne_points /odometry/filtered /imu/data_rep145 \
  /pcl_pose /alignment_status /tf /tf_static
```

- **`/velodyne_points` / `/odometry/filtered` / `/imu/data_rep145`**: localizer
  の入力 (scan) と odometry 拘束の入力 (Issue #108 で配線) を揃える。この 3 本が
  あれば localizer を off-board で再走行させ、reject 連鎖を再現・解析できる
- `/pcl_pose` / `/alignment_status`: 当日の fitness / reject 判定の実測ログ
- `/tf` / `/tf_static`: `map -> odom -> base_link` の凍結有無を後追いする
- 録画後 `ros2 bag info <dir>` で `/velodyne_points` count ≈ 走行秒 × 10、
  `/imu/data_rep145` count ≈ 走行秒 × 100 を確認 (半分以下なら破棄して再録、
  CLAUDE.md §ランタイム環境の前提)

## 関連

- [ADR-0009: p2ls 高さ帯 + QoS bridge](decisions/0009-p2ls-height-band.md)
- [ADR-0011: 地面除去手法選定](decisions/0011-ground-removal-choice.md)
- [ADR-0007: failsafe / twist_mux](decisions/0007-failsafe-design.md)
  §Demo-scope reduction
- [`../maps/campus/README.md`](../maps/campus/README.md) §3 (map salt
  の焼き込み経緯と対策)

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
- [ ] **/velodyne_points が 10 Hz**: `ros2 topic hz /velodyne_points` で
      9-11 Hz。20 Hz 前後や 40 Hz 近辺なら duplicate bringup の兆候
- [ ] `map -> odom -> base_link` の TF chain が 1 本鎖 (`ros2 run
      tf2_tools view_frames`)
- [ ] `/alignment_status.has_converged: true` かつ `fitness < 1.0`
      (静止状態、初期位置合わせ後)
- [ ] `/scan` の publisher count = 1 (velodyne_laserscan の
      `/scan_raw` remap 有効): `ros2 topic info /scan`
- [ ] operator 随伴、ジョイスティック介入可能 (ADR-0007 §Demo-scope
      reduction)

### マップ variant 選択 (Task #13 salt cleanup)

`docs/maps/campus/occupancy.pgm` は M5-R 時代 (Patchwork++ 導入前) の
地面ノイズを salt として焼き込んでいる (2026-07-16 field 立証)。demo
本番では salt を除去した cleaned 版を使う:

```
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=cleaned
```

初回起動時に `/map` を RViz OccupancyGrid で表示し、traversed 経路上の
黒 salt が消えていることを目視確認する (cleaning_diff.png と照合)。

### RealSense (opt-in、通常 off)

D435 は M6-R runtime stack が消費していない。USB 2.1 認識問題があるため
起動対象から外している (`sensors_launch.py` の `realsense` arg default
false)。camera-specific test を意図的に走らせるときのみ `realsense:=true`
を bringup コマンドに付与。付与時は改めて USB 点検 (`lsusb` で D435
検出 + `/dev/bus/usb/` の権限) をチェックリストに追加すること

## 関連

- [ADR-0009: p2ls 高さ帯 + QoS bridge](decisions/0009-p2ls-height-band.md)
- [ADR-0011: 地面除去手法選定](decisions/0011-ground-removal-choice.md)
- [ADR-0007: failsafe / twist_mux](decisions/0007-failsafe-design.md)
  §Demo-scope reduction
- [`../maps/campus/README.md`](../maps/campus/README.md) §3 (map salt
  の焼き込み経緯と対策)

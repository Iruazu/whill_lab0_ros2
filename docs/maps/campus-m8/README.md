# `campus-m8`（M8-2 で作り直す campus 地図）

M8-2（#152〜#155）で、07-10 の地図用 bag から作り直す campus 地図。旧 `campus` は残し、別の site として置く。**採用は未定**。実機・屋外での確認（#156）と、ユーザーによる手塗りの確定（#154）が済むまで、デモは旧 `campus` を使う。

旧地図との違いは、点群の校正だけを正しくしたこと（Hi-Res の実機に VLP16db.yaml を使っていた。#144）。経路と元の記録は同じ。

## 状態

| 段階 | Issue | 状態 |
| --- | --- | --- |
| 1. 元 bag の点群を Hi-Res に補正 | #152 | 済（オフライン検証まで） |
| 2. GLIM → DUFOMap で static.pcd | #153 | 下の節を参照 |
| 3. 占有格子と手塗りの候補 | #154 | 下の節を参照 |
| 4. waypoint とローカライザの再生確認 | #155 | 下の節を参照 |

## 1. 元 bag の補正（#152）

元の bag（`~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag/`、2162 秒、点群 21311 個）にはパケットが無く、`/velodyne_points` だけがある。点に `ring` があるので、VLP16db.yaml で計算された点を、距離と方位を保ったまま Hi-Res の仰角へ解析的に戻せる（式は `scripts/velodyne_reproject_bag.py` の冒頭）。元の bag は読むだけで、変えていない。

### 実行したコマンド

```bash
cd ~/whill_lab0_ros2   # worktree でもよい
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
P=$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud/params
B=~/bags/m5r-bench-data/2026-07-10-campus-outer-final
python3 scripts/velodyne_reproject_bag.py $B/bag $B/bag-hires \
  --src-calib $P/VLP16db.yaml --dst-calib $P/VLP16_hires_db.yaml
```

- 出力：`~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires/`（13 GB、sqlite3）。点群以外の話題は、受信時刻と QoS ごとそのまま写す。
- 所要：14 分（1 スレッド）。出力先があると止まるので、やり直すときは出力先を変える。
- 結果：点群 21311 個を補正、他のメッセージ 216195 個を複写。合計 237506 個で、元の bag と一致する。読んだ点から距離を計算し直したときの z の食い違いは最大 3.96 µm（float32 の丸め）。

### 検証

1. **ドライバそのものとの一致**（`scripts/velodyne_reproject_verify.sh`）。パケットと点群の両方がある段階Aの bag で、(a) 記録された点群を本ツールで補正したものと、(b) パケットを `velodyne_transform_node` に VLP16_hires_db.yaml で流した出力を、点ごとに比べた。点の並び（organized、NaN の位置）、ring、座標を見る。

   | bag | 比べた点群 / 点 | 最大の差 | p99.9 | NaN・ring の不一致 |
   | --- | --- | --- | --- | --- |
   | `sensor_check_20261028/chk_empty_0m_160726` | 113 / 2,276,593 | 0.015 mm | 0.004 mm | 0 |
   | `sensor_check_20261028/chk_chair_3m_160533` | 108 / 2,263,144 | 0.015 mm | 0.004 mm | 0 |

   対照として、補正していない記録を同じ基準と比べると最大 3.2 m ずれ、FAIL になる（比較が校正の違いを見分けられることの確認）。
   Issue の「0.00 mm で一致」は、小数第2位までなら 0.02 mm であり、厳密な 0 ではない。差は float32 の丸め（130 m で 1 回 0.008 mm、入力・補正・ドライバの出力でそれぞれ丸める）で、判定の許容は 0.05 mm にした。

   ```bash
   scripts/velodyne_reproject_verify.sh ~/bags/sensor_check_20261028/chk_empty_0m_160726 <新しい作業ディレクトリ>
   ```

2. **全行程の出力と、#147 の区間の一致**。#147 で別に作った先頭 300 秒（`~/bags/imu_investigation_20261030/seg_0_300_hires`）と、今回の全行程の先頭 2957 個は、6184 万点すべてで差 0.000 mm（同じ式なので当然だが、全行程を通しても壊れていないことの確認）。

   ```bash
   python3 scripts/velodyne_compare_clouds.py ~/bags/imu_investigation_20261030/seg_0_300_hires \
     ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires
   ```

確かめていないこと：補正後の点群が実物の形に合うこと（真値が無い）。これは地図の品質（下の節）と屋外の静止確認（#149）で見る。

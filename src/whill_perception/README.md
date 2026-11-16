# whill_perception

Patchwork++のC++ coreを使って点群から地面を除去する自作ROS 2 wrapper。

## パイプライン

`/velodyne_points` → `patchworkpp_node` → `/velodyne_points_no_ground` → pointcloud_to_laserscan → `/scan`。

`ground_removal_launch.py` は `whill_navigation/nav_launch.py` に内包される。Nav2起動後に別途重複起動しない。地面除去後の点群はfailsafeのLayer Cも監視する。

設定は `config/patchworkpp.yaml`。斜面・路面の誤検知を減らすための前処理だが、段差や人をすべて検知する保証ではない。現用設定の近距離除外は1mで、failsafe Layer Dの1〜2m帯にも関係する。

## sensor_height の出典と検証

`sensor_height` は [校正台帳](../../docs/calibration-ledger.md) §2.1 の base_link→velodyne z（0.771 m、無人）に揃える。台帳が変わったら同時に変える（#192）。人が座ると床に対する pitch が約 −0.75° 変わり、前方の床の見かけの高さは 1 m 先ごとに約 1.3 cm 変わる（台帳 §2.1 B）。

コアは `sensor_height` を最内 zone の seed の下限と RNR の下限にだけ使い、毎フレーム床の推定で書き換える。そのため数 cm の差は出力に出ない。`scripts/regression/patchwork_height_compare.py` で 0.79 と 0.771 を同じフレームに同時にかけた結果（2026-11-16、domain 192・localhost）：

| bag | フレーム | ground（平均） | no_ground（平均） | 床の漏れ | 物の保持 | 0.79 と 0.771 の差 |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-07-24 走行（`rosbag2_2026_07_24-19_05_26-hires`、73 秒から 180 秒） | 1774 | 7862 | 11088 | 10.6 % | 88.1 % | 全フレームで 0 点 |
| 2026-11-16 屋外静止・物なし（`empty_start_140915`） | 117 | 7766 | 14306 | 9.1 % | 82.4 % | 全フレームで 0 点 |
| 2026-11-16 屋外静止・コーン 3 m（`cone_3p_141709`） | 116 | 7575 | 13407 | 9.4 % | 84.0 % | 全フレームで 0 点 |

- 点数は 1 m < r ≤ 25 m（コアの判定範囲）。床の漏れは 1〜8 m で入力の床平面から 5 cm 以内の点のうち no_ground に残った割合、物の保持は 0.20〜2.0 m の点のうち残った割合。床平面は 1 枚で当てているので、路面の凹凸・勾配の分も漏れに入る。絶対値は地面除去の性能ではなく、設定を変えたときの比較の基準として使う。
- 比較の感度：物なし bag で 0.55 にすると no_ground の平均が 3 点変わる（1.0 では 0 点）。ずれが 20 cm を超えても差は小さい。
- 未確認：着座の bag での地面除去。

再現：

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/regression/patchwork_height_compare.py \
    --bag ~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires \
    --start-offset 73 --duration 180 --heights 0.79 0.771
```

## ビルド

上流C++ coreは `src/third_party/patchwork_plusplus/cpp/`（`whill_lab.repos` から取得）。自作wrapperがcoreを直接組み込むため、上流の `ros/` / `cpp/` / `python/` はcolconで個別にビルドしない（`COLCON_IGNORE`）。

上流wrapperのライセンス表記の不整合を避ける設計判断は [ADR-0011](../../docs/decisions/0011-ground-removal-choice.md) に保存している。車載ビルド手順は [開発環境](../../docs/development.md)。

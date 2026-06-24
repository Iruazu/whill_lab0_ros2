# 2026-06-24 loop-outdoor-ext 取得メモ

この run の bag (`bag/bag_0.db3`、Duration 239.45 s) は健全:

| topic | count | 実効レート | 期待 | 判定 |
|---|---|---|---|---|
| `/velodyne_points` | 2361 | 9.86 Hz | ~10 Hz | OK |
| `/imu/data_rep145` | 23945 | 99.99 Hz | 100 Hz | OK |
| `/tf_static` | 4 | — | ≥1 | OK |

## 同日に発生した `/velodyne_points` 1 Hz 病について

この bag (13:49 開始) 録画の**後**に追加 take2 を試行した際、
`/velodyne_points` が 0.94 Hz まで詰まる現象が再現した。原因切り分けと
恒久対策は `docs/ja/m5r-rmw-cyclonedds.md` に集約 (FastDDS の大メッセージ
配送詰まり、CycloneDDS で解消)。

**この bag 自体は症状発生前のセッションで取得しており、再録不要**。
今後 GLIM/DUFOMap 工程はこの bag をそのまま使ってよい。

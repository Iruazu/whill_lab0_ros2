# 2026-07-07 campus-half-v3 取得メモ

M6-R (demo 想定経路上での GLIM 地図生成) 用の 2 本目の bag。
1本目 (`2026-07-07-campus-loop`) が速度変動 + 非計画停止で外乱が多かった
ため、速度固定・停止なしで再取得した対照データ。**GLIM の第一候補は本 bag**。

## 健全性

Duration 466.26 s (7 分 46 秒)。CLAUDE.md 規約 (走行秒 × rate の半分以下なら
破棄) は余裕でクリア。

| topic | count | 実効レート | 期待 | 判定 |
|---|---|---|---|---|
| `/velodyne_points` | 4590 | 9.85 Hz | ~10 Hz | OK |
| `/imu/data_rep145` | 46626 | 100.0 Hz | 100 Hz | OK |
| `/tf_static` | 4 | — | ≥1 | OK |

## 走行プロトコル

- 開始 30 秒静止 (IMU bias 初期化)
- WHILL 速度モード 3 固定、加速なし、その場旋回なし、停止なし
- 有人ジョイスティック
- 7 号館前起点、反時計回りの外周 1 周 (デモ想定領域の約 30% 区間)

## 環境

- 2026-07-07 19:30 開始 / 19:38 終了 (JST)
- 薄暮〜夜間、街灯下
- 路面ほぼ乾燥
- 自転車 4 台程度の往来、通行を妨げる車両なし

## 収録中の DDS ログについて

CycloneDDS が `10.130.7.163:74xx` (本機自身の WiFi `wlo1`) 宛の
`ddsi_udp_conn_write ... failed with retcode -1` を大量に出したが、
これは屋外走行中の WiFi 瞬断で peer discovery が失敗しただけ。Velodyne
(Ethernet, 192.168.1.201) と IMU (USB) の実データ経路は WiFi と独立の
ため録画本体は無事、上記健全性表の通り。

## 次工程

1. 本 bag に GLIM を適用 (`scripts/m5r3_run_glim.sh <bag> <out>`)
2. run.log で loop closure が発火したか確認
   - 発火した場合: そのまま occupancy grid 生成 (`scripts/m5r_pcd_to_occupancy.sh`)
   - 未発火の場合: 1 本目 (`campus-loop`) との比較材料にする。**M6-R
     方針では <1% 大域ループ閉合を G1-G3 ゲートに含めないため、
     未発火でも先に進む**
3. `docs/maps/campus-half-v3/` 相当 (site 名は要検討) を作成し、
   lidar_localization_ros2 の smoke test 入力にする

## 1本目 (`2026-07-07-campus-loop`) との違い

| 軸 | 1本目 (campus-loop) | 本 bag (campus-half-v3) |
|---|---|---|
| Duration | 821 s (13.7 分) | 466 s (7.8 分) |
| 距離 | 想定領域を広めに 1 周 | 上記の約 30% 区間 1 周 |
| 速度 | モード 3 → 4 変動あり | モード 3 終始固定 |
| 停止 | 車両待ちで非計画停止あり | なし |
| 外乱 | 多い | 少ない |
| 用途 | 参考データ | **GLIM 主入力** |

# whill_safety

実機統合bringup、failsafe、twist_muxを提供する。2026-10-14 にlaunch・設定・`failsafe_node.py`を照合。運用手順は [runbook](../../docs/ja/runbook-live-demo.md)。

## 起動単位

`m6r_bringup_launch.py` は odom bringup（センサ・WHILL・EKF）、NDT localizer、`safety_launch.py`（failsafe・twist_mux）を含む。内包するlaunchを別途起動しない。Nav2 / dispatch は別ターミナルで併用する。

`site:=campus` は `docs/maps/campus/static.pcd` を読む。`WHILL_MAPS_ROOT` で親ディレクトリを変更できる。PCDはGit対象外。RealSenseは既定false。

## Localizer

`config/m6r_lidar_localization.yaml` を一時YAMLに展開し地図パスを設定する。NDT_OMP、`use_odom: true`、`predict_pose_from_previous_delta: false`。odomを `/odometry/filtered` にremapする。直接IMU入力は使用しない。初期位置は `/initialpose`。

localizerの受理閾値 `score_threshold: 6.0` と、failsafeのfitness閾値1.0は別の値。

## 監視層

| 層 | 入力 | 作動条件 |
| --- | --- | --- |
| A | `/reinitialization_requested` | true受信後1秒保持 |
| B | `/alignment_status`, `/pcl_pose` | fitness>1.0または非収束が2秒継続、pose受信後の途絶>1秒 |
| C | `/velodyne_points_no_ground` | 初回受信後の途絶>2秒 |
| D | `/scan` | 前方±30°、1〜2mの有限点が1点以上。最後の検知から0.5秒保持 |

Dは点の検知で、人の分類ではない。`DEAD INPUT` watchdogは開始10秒後に未受信入力をエラー通知するが、ログを出すだけで起動禁止を保証するものではない。pose / perception の途絶監視は初回受信で開始する。

作動中は `/cmd_vel_safety` にゼロ指令を20Hzで出す。無発報は正常の証明ではなく、未受信や未起動も切り分ける。

閾値はすべてROSパラメータで、既定値は上表と同じ（`fitness_max`、`forward_min_m`、`forward_sector_half_angle_deg` など。一覧は `failsafe_node.py` の `__init__`）。パラメータを渡さない現行の起動は従来と同じ動作になる（2026-10-28、bag 2本で作動・解除の順序と時刻の一致を確認）。

| 追加パラメータ | 既定 | 用途 |
| --- | --- | --- |
| `layer_d_shape` | `sector` | `rectangle` で前方1〜2m・左右±`layer_d_rect_half_width_m`（既定0.5m）の長方形に切替。回避モード用で、Collision Monitorと併用する前提 |
| `extra_perception_topics` | 空 | Layer Cと同じ途絶監視をPointCloud2トピックに追加する |

`m6r_bringup_launch.py avoidance:=true` のときだけ、`safety_launch.py` が failsafe に `layer_d_shape: rectangle`、`layer_d_rect_half_width_m: 0.5`、`extra_perception_topics`（既定は Collision Monitor の入力2本。`failsafe_extra_perception_topics:=` で変えられる）を渡す。`avoidance:=false` では `use_sim_time` 以外を渡さないので、閾値は上表の既定値のまま（`test/test_safety_launch_params.py`）。07-24 の bag を再生すると、Layer D の作動は扇形で39回、長方形で3回だった（[報告](../../docs/ja/reports/2026-10-30-layer-d-engagements/README.md)）。

状態は `/failsafe/state`（`std_msgs/String`）に、作動中の層をカンマ区切り、解除中は `OK` として、変化時と1秒ごとに出す。

## 調停と制約

| 入力 | 優先度 | タイムアウト |
| --- | --- | --- |
| `/cmd_vel_safety` | 100 | 0.5秒 |
| `/cmd_vel_teleop` | 50 | 0.5秒 |
| `/cmd_vel_nav` | 10 | 0.5秒 |

設定は `config/twist_mux.yaml`。最高優先度の有効入力が勝ち、出力は `/cmd_vel`。後段にNav2のvelocity_smootherがあるため、ゼロ指令も平滑化される。物理E-stopとの統合や全周監視をこのパッケージが完成済みとは扱わない。

preflightは `scripts/m6r_preflight.sh`。check 4の単発rosout検査に制約があり、runbookの入力確認を併用する。手動中のsafety優先停止について過去記録に未検証注記がある。実機受入の証跡を確認するまで完了表記にしない。

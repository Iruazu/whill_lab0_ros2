# whill_localization

車輪速度とIMUを融合するオドメトリ層。

## 入出力

| 種類 | 内容 |
| --- | --- |
| 車輪入力 | `/whill/odom`。poseではなくvx / vy / vyawを融合 |
| IMU入力 | `/imu/data_rep145`。角速度・加速度を使用しorientationは融合しない |
| 出力 | `/odometry/filtered`、TF `odom → base_link`。設定30Hz |
| モード | `two_d_mode: true`、`world_frame: odom` |

IMUの符号補正と静的TFは `whill_sensors_bringup` が担当する。

## launch

- `odom_bringup_launch.py`：センサ + WHILL driver + EKF。driverの `port_name` は `/dev/whill`。
- `ekf_odom_launch.py`：EKFのみ。

実機統合時は `whill_safety/m6r_bringup_launch.py` がodom bringupを含む。別途起動しない。`map → odom` はNDT localizerが担当する。

地図作成はGLIMのオフライン処理。FAST-LIOのruntime launchは削除済み。

運用は [runbook](../../docs/runbook-live-demo.md)、校正履歴は [校正台帳](../../docs/calibration-ledger.md)。センサの物理配置を変更したら既存の数値をそのまま再利用しない。

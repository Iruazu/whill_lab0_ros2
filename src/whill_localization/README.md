# whill_localization

車輪速度とIMUを融合するオドメトリ層。2026-10-14 にlaunchと `config/ekf_odom.yaml` を照合。

## 入出力

| 種類 | 内容 |
| --- | --- |
| 車輪入力 | `/whill/odom`。poseではなくvx / vy / vyawを融合 |
| IMU入力 | `/imu/data_rep145`。角速度・加速度を使用しorientationは融合しない |
| 出力 | `/odometry/filtered`、TF `odom → base_link`。設定30Hz |
| モード | `two_d_mode: true`、`world_frame: odom` |

生の `/imu/data_raw` をEKF入力とする旧説明は現用設定と異なる。IMUの符号補正と静的TFは `whill_sensors_bringup` が担当する。

## launch

- `odom_bringup_launch.py`：センサ + WHILL driver + EKF。driverの `port_name` は `/dev/whill`。
- `ekf_odom_launch.py`：EKFのみ。

実機統合時は `whill_safety/m6r_bringup_launch.py` がodom bringupを含む。別途起動しない。`map → odom` はNDT localizerが担当する。

旧FAST-LIO runtime launch / configは削除済み。地図作成はGLIMのオフライン処理。旧 `localization_launch.py` を実行入口にしない。

運用は [runbook](../../docs/ja/runbook-live-demo.md)、校正履歴は [校正台帳](../../docs/ja/calibration-ledger.md)。センサの物理配置を変更したら既存の数値をそのまま再利用しない。

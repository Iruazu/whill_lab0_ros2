# whill_sensors_bringup

VLP-16、RT USB 9軸IMU、任意のRealSense D435と静的TFを起動する。

## 起動構成

`sensors_launch.py` はVelodyne、`imu_launch.py`、静的TFを含む。`realsense:=true` の場合だけカメラを追加する。Velodyne側のscanは `/scan_raw` にremapされ、Nav2が使用する `/scan` と区別する。

IMUはlifecycleのconfigure / activateをlaunchで行う。`imu_sign_corrector` は `/imu/data_raw` の加速度3軸を反転し `/imu/data_rep145` に再配信する。EKFは補正後を使う。

統合bringup / odom bringupに内包されるため、実機デモで個別に追加起動しない。センサ単体試験だけで `ros2 launch whill_sensors_bringup sensors_launch.py` を使う。

## デバイス

IMUの既定portは `/dev/imu`、frameは `imu_link`。udevのVID:PID規則は `udev/99-whill-stack.rules`。WHILLは `/dev/whill`。ttyUSB / ttyACMの番号に依存しない。

VLP-16は有線UDP接続。DDS用XMLのNIC名と実際の有線設定を別々に確認する。カメラは既定無効で、現用ナビゲーションには使われていない。

## 現用静的TF

`launch/static_tf_launch.py` の値。並進はm、回転はrad。

| 子frame | x, y, z | roll, pitch, yaw |
| --- | --- | --- |
| imu_link | 0.38, -0.03, 0.47 | -0.0713, -0.1415, 0 |
| velodyne | 0.484136, 0.381548, 0.793704 | -0.035342, 0.156983, -0.005527 |
| camera_link | 0.54, 0.382, 0.79 | 0, 0, 0 |

親はすべて `base_link`。カメラは概算、LiDARは旧校正由来の値を含む。根拠と履歴は [外部パラメータ記録](../../docs/ja/m3-extrinsics-from-noetic.md) と [校正台帳](../../docs/ja/calibration-ledger.md) を参照。

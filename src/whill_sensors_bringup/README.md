# whill_sensors_bringup

VLP-16、RT USB 9軸IMU、任意のRealSense D435と静的TFを起動する。2026-10-14 にlaunchを照合。

## 起動構成

`sensors_launch.py` はVelodyne、`imu_launch.py`、静的TFを含む。`realsense:=true` の場合だけカメラを追加する。カメラの設定は `config/realsense.yaml`（深度848×480・15Hz、2×2間引き、色なし点群 `/camera/camera/depth/color/points`）。Velodyne側のscanは `/scan_raw` にremapされ、Nav2が使用する `/scan` と区別する。

IMUはlifecycleのconfigure / activateをlaunchで行う。`imu_sign_corrector` は `/imu/data_raw` の加速度3軸を反転し `/imu/data_rep145` に再配信する。EKFは補正後を使う。

統合bringup / odom bringupに内包されるため、実機デモで個別に追加起動しない。センサ単体試験だけで `ros2 launch whill_sensors_bringup sensors_launch.py` を使う。

## デバイス

IMUの既定portは `/dev/imu`、frameは `imu_link`。udevのVID:PID規則は `udev/99-whill-stack.rules`。WHILLは `/dev/whill`。ttyUSB / ttyACMの番号に依存しない。

LiDARの実機はVLP-16 Puck Hi-Res（上下±10°）だが、ドライバは±15°用の `VLP16db.yaml` を読んでおり、`/velodyne_points` の高さは歪んでいる（2026-10-28判明。修正は別課題）。VLP-16は有線UDP接続。DDS用XMLのNIC名と実際の有線設定を別々に確認する。カメラは既定無効で、現用ナビゲーションには使われていない。

## 現用静的TF

`launch/static_tf_launch.py` の値。並進はm、回転はrad。

| 子frame | x, y, z | roll, pitch, yaw |
| --- | --- | --- |
| imu_link | 0.38, -0.03, 0.47 | -0.0713, -0.1415, 0 |
| velodyne | 0.484136, 0.381548, 0.793704 | -0.035342, 0.156983, -0.005527 |
| camera_link | 0.47, 0.378, 0.73 | 0, 0, 0 |

親はすべて `base_link`。camera_linkは2026-10-28の巻尺実測とD435のURDF寸法による（静止物bagで高さ誤差+1cm未満を確認）。LiDARは旧校正由来の値で、実測（x 0.42、y 0.36、z 約0.78、pitch 約+1°）と食い違う。表は現在コードに入っている値であり、最新の実機配置で再校正済という主張ではない。根拠と履歴は [外部パラメータ記録](../../docs/ja/m3-extrinsics-from-noetic.md) と [校正台帳](../../docs/ja/calibration-ledger.md) を参照。

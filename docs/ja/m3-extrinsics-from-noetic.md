# noetic スタックから引き継ぐ LiDAR ↔ IMU 外部パラメータ

Language: [日本語](m3-extrinsics-from-noetic.md) | [English](../en/m3-extrinsics-from-noetic.md)

noetic 側の `whill_lab0` リポは校正済みの LiDAR-IMU 外部パラメータを `FAST_LIO/config/velodyne.yaml` に持っていた。M4 (humble 上の FAST-LIO) がゼロから再キャリブせずに既知の良好な設定から始められるよう、正確なポーズを下に記す。

## 出典

[`whill_lab0/FAST_LIO/config/velodyne.yaml`](https://github.com/Iruazu/whill_lab0/blob/main/FAST_LIO/config/velodyne.yaml) の `mapping:` セクション、`extrinsic_T` / `extrinsic_R` フィールド。

## 値

並進 `extrinsic_T` (IMU フレームで表した LiDAR 原点、メートル):

```
[ 0.104136, 0.411548, 0.323704 ]
```

回転 `extrinsic_R` (3×3、row-major、LiDAR → IMU):

```
[  0.987688,  0.000000,  0.156434,
  -0.005459,  0.999391,  0.034470,
  -0.156339, -0.034900,  0.987087 ]
```

これは Z 軸まわりにほぼ +9.0° の yaw (`acos(0.987688) ≈ 8.96°`) と、小さな ~2° の pitch/roll 成分の組み合わせ。

同 yaml の関連 FAST-LIO 入力:

- `lid_topic: /velodyne_points`
- `imu_topic: /imu/data_raw`
- `lidar_type: 2` (Velodyne)
- `scan_line: 16`, `scan_rate: 10` (VLP-16 の 10 Hz と一致)
- IMU ノイズ: `acc_cov: 0.1`, `gyr_cov: 0.1`, `b_acc_cov: 1e-4`, `b_gyr_cov: 1e-4`

## M4 での適用方法

FAST-LIO の ROS 2 fork を `whill_lab.repos` に追加したら、これらの値をそのまま humble 側の同等 config ファイルにコピーする。椅子で小さなループを走らせて mapping させ、ドリフトを確認することで検証する。引き継いだ外部パラメータが間違っている (例えば noetic 時代と humble 時代の間にセンサが物理的にマウントし直された) 場合は、LI-Init スタイルのキャリブレーションを再実行する。

センサマウントが noetic 時代から変わっていなければ、本キャリブはまだ有効なはず。

# Inherited LiDAR ↔ IMU extrinsic calibration (from noetic stack)

The noetic-side `whill_lab0` repo carried a calibrated LiDAR-IMU extrinsic
transform inside `FAST_LIO/config/velodyne.yaml`. The exact pose is below
so M4 (FAST-LIO on humble) can start from a known-good configuration
instead of re-calibrating from scratch.

## Source

[`whill_lab0/FAST_LIO/config/velodyne.yaml`](https://github.com/Iruazu/whill_lab0/blob/main/FAST_LIO/config/velodyne.yaml),
section `mapping:`, fields `extrinsic_T` / `extrinsic_R`.

## Values

Translation `extrinsic_T` (LiDAR origin expressed in IMU frame, metres):

```
[ 0.104136, 0.411548, 0.323704 ]
```

Rotation `extrinsic_R` (3×3, row-major, LiDAR → IMU):

```
[  0.987688,  0.000000,  0.156434,
  -0.005459,  0.999391,  0.034470,
  -0.156339, -0.034900,  0.987087 ]
```

That rotation is approximately a yaw of +9.0° about Z (`acos(0.987688) ≈
8.96°`) with a small ~2° pitch/roll component.

Other related FAST-LIO inputs from the same yaml:

- `lid_topic: /velodyne_points`
- `imu_topic: /imu/data_raw`
- `lidar_type: 2` (Velodyne)
- `scan_line: 16`, `scan_rate: 10` (matches VLP-16 at 10 Hz)
- IMU noise: `acc_cov: 0.1`, `gyr_cov: 0.1`, `b_acc_cov: 1e-4`, `b_gyr_cov: 1e-4`

## How to apply in M4

When the FAST-LIO ROS 2 fork is added to `whill_lab.repos`, copy these
values verbatim into the equivalent humble config file. Validate by
mapping a small loop on the chair and checking drift — if the inherited
extrinsic is wrong (e.g. the sensors were physically remounted between
the noetic build and the humble build), re-run a LI-Init style calibration.

If the sensor mounts are unchanged from the noetic era, this calibration
should still hold.

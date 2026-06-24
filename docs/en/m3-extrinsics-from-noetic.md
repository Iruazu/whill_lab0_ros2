# Inherited LiDAR ↔ IMU extrinsic calibration (from noetic stack)

Language: [日本語](../ja/m3-extrinsics-from-noetic.md) | [English](m3-extrinsics-from-noetic.md)

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

That rotation is approximately a pitch of +9.0° about Y
(`-asin(R[2][0]) = -asin(-0.156339) ≈ +8.99°`) with small roll (~-2°)
and yaw (~-0.3°) components.

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

## `base_link` placement (M4R-2 + Issue #61)

The Japanese version of this document carries the detailed derivation of
`base_link → {imu_link, velodyne, camera_link}` from this noetic-inherited
extrinsic plus the measured IMU position. The base_link is defined as the
rear-axle midpoint projected to ground (REP-103 axes). Issue #61
(2026-06-24) replaced the M4R-2 placeholder values with measured ones:

| Parent → Child | Translation [m] | Rotation (RPY rad) | Source |
|---|---|---|---|
| `base_link → imu_link` | (0.38, -0.03, 0.47) | (0, 0, 0) | Measured (Issue #61) |
| `base_link → velodyne` | (0.484136, 0.381548, 0.793704) | (-0.035342, +0.156983, -0.005527) | Computed: imu_link + noetic `extrinsic_T` |
| `base_link → camera_link` | (0.54, 0.382, 0.79) | (0, 0, 0) | Placeholder (rigid co-mount with LiDAR; M6-R target-based recalibration pending) |

See [`../ja/m3-extrinsics-from-noetic.md`](../ja/m3-extrinsics-from-noetic.md)
§ "base_link 基準 extrinsic の算出" for the full derivation, measurement
procedure, and the `Rz·Ry·Rx` decomposition of `extrinsic_R`.

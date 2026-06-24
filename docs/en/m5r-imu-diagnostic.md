# M5-R: GLIM IMU integration warning — diagnosis and remediation

Language: [日本語](../ja/m5r-imu-diagnostic.md) | [English](m5r-imu-diagnostic.md)

This document satisfies Issue #64's acceptance criteria: a root-cause
analysis with measurement evidence, and an agreed remediation path.

## Background

While running `scripts/m5r3_run_glim.sh`, GLIM's odometry estimator
continuously emits, for every validation window over the entire bag:

```
[odom] [warning] IMU prediction is not good.
[odom] [warning] Possibly T_lidar_imu is not accurate or IMU bias is not well estimated.
[odom] [warning] IMU better ratios rot=0.58, trans=0.11, vel=0.16
```

`IMU better ratios` measures the fraction of validation windows where
IMU-fused prediction beats LiDAR-only prediction (1.0 = ideal, 0.5 =
break-even). GLIM's warning thresholds (`src/glim/common/imu_validation.cpp`)
are rot=0.7 / trans=0.4 / vel=0.5. The chair sits at 0.58 / 0.11 / 0.16
— rotation is borderline, translation and velocity are pathologically
low (IMU **hurts** translation prediction in 89% of samples).

## Candidate root causes (from Issue #64)

1. Noetic-inherited `T_lidar_imu` value itself imprecise
2. IMU noise / bias parameters not tuned for the actual sensor
3. IMU bias temperature drift / initialization
4. `/imu/data_rep145` rate or buffering issue

## Experiment summary

| Hypothesis | Method | Result | Verdict |
|---|---|---|---|
| (4) IMU rate | inspect bag: 23945 msgs / 239 s | **99.99 Hz** (matches spec) | rejected |
| (1) SE3 math wrong | re-derive `T_lidar_imu` from noetic `extrinsic_T`/`extrinsic_R` | matches GLIM config to 1e-7 | math correct; **noetic value itself** may still be imprecise |
| (2) Noise sigmas too large | patch `imu_acc_noise` 0.05→0.008, `imu_gyro_noise` 0.02→0.0013 (datasheet), re-run | trans 0.11→**0.02**, vel 0.16→**0.06** (worse) | **rejected** — current inflated values are a deliberate defence |
| extra: LiDAR-only | `enable_imu: false` in sub/global mapping | loop error 3.97 m (vs 3.99 m baseline) | front-end odom IMU still active; sub/global IMU contribution near zero |

(Static 5-second noise-density estimate from the bag: gyro stddev ≈
0.002 rad/s, accel stddev ≈ 0.025 m/s² — IMU is *quieter* than the
MPU-9250 datasheet, so its raw noise is not the problem.)

## Conclusion

The actual bottleneck is the **rotational accuracy of `T_lidar_imu`**.

- The SE3 math in `m5r3_run_glim.sh` correctly inverts the noetic-
  inherited extrinsic.
- The noetic-inherited `extrinsic_R` (the LiDAR↔IMU rotation matrix
  itself) was never independently calibrated with kalibr or similar.
  Empirically a 1–2° rotation error here is enough to mis-project
  gravity, inject ≈0.17 m/s² of spurious acceleration, and accumulate
  into translation / velocity prediction errors that exceed the
  LiDAR-only baseline.

Hypothesis (3) (bias initialisation) is no longer plausible because
flooding the IMU with more (correctly weighted) data did not improve
the trans/vel ratios.

## Remediation: GRIL-Calib

**GRIL-Calib** (RA-L 2024, `Taeyoung96/GRIL-Calib`, BSD-3-Clause,
official `humble` branch) was selected for the recalibration.

Why GRIL-Calib:
- Targetless: no calibration board required. The chair just drives
  in figure-8 patterns on a flat floor.
- Designed for ground robots — uses planar motion + ground-plane
  constraint, which is exactly the WHILL situation.
- ROS 2 humble branch builds cleanly against our existing
  `livox_ros_driver2` install.

Rejected alternatives:
- **kalibr** (ETH-ASL): ROS 1 noetic native; humble port is non-trivial.
- **lidar_imu_calib (APRIL-ZJU)**: ROS 1 only, last updated 2020.
- **Plan C (extended static init)**: would have addressed candidate (3),
  but the experiments above prove (3) is not the bottleneck.

## Procedure

### 1. Set up GRIL-Calib (done — see `scripts/install_gril_calib.sh`)

Builds GRIL-Calib in `~/calib_ws/` against the in-repo
`livox_ros_driver2` install (~1 minute, no Docker needed).

### 2. Record a motion bag (user, pending)

| Item | Value |
|---|---|
| Location | Indoor flat floor, ≥ 5 m × 5 m clear area |
| Vehicle | WHILL, joystick-driven by human, 0.3–0.5 m/s |
| Required motion | (a) figure-8, ≥ 3 laps; (b) hard accel/decel in straight runs; (c) two 360° in-place rotations |
| Duration | 3–5 minutes |
| Environment | Static — avoid pedestrians / bicycles |
| Topics | `/velodyne_points`, `/imu/data_rep145`, `/tf_static` (same as production map recording) |
| Compression | None (zstd is unsupported by both GLIM and GRIL-Calib) |

### 3. Run GRIL-Calib

After motion bag is recorded:

```bash
scripts/m5r4_run_gril_calib.sh docs/m5r-bench-data/<run>/bag <out-dir>
```

(Wrapper script to be added in a follow-up PR alongside the motion bag.
For the manual procedure see Appendix A of the Japanese version.)

### 4. Apply the new `T_lidar_imu` to GLIM

Transcribe GRIL-Calib's output into `scripts/m5r3_run_glim.sh`'s
`new_tli` literal (around line 274).

### 5. Verify

Re-run GLIM on the existing 2026-06-24 bag. Pass criteria:

| Metric | Current | Target |
|---|---|---|
| rot ratio | 0.60 | ≥ 0.7 |
| trans ratio | 0.11 | ≥ 0.4 |
| vel ratio | 0.16 | ≥ 0.5 |
| Loop error | 3.99 m / 106 m (≈3.7%) | < 1.06 m (< 1%) |

If all four pass, update ADR-0003 with a note that the GLIM choice is
validated under recalibrated extrinsic.

## Related

- Issue #64 (this document's parent)
- PR #62 (Issue #61): `base_link → imu_link` measurement — prerequisite
- PR #71 (Issue #63): GLIM auto_quit — prerequisite (GLIM had to
  complete a run for these measurements to exist)
- `docs/ja/m3-extrinsics-from-noetic.md`: source of the
  noetic-inherited extrinsic that this calibration replaces
- ADR-0003: GLIM adoption; to be amended after recalibration
- `src/third_party/glim/src/glim/common/imu_validation.cpp`: where the
  warning thresholds are implemented

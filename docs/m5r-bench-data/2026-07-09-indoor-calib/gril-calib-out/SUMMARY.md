# GRIL-Calib run summary

- Bag: `/home/systemlab/whill_lab0_ros2/docs/m5r-bench-data/2026-07-09-indoor-calib/bag`
- Bag duration: 209.149863567s
- Watchdog: 269s
- Status: **insufficient_motion**
- Patched config: `velodyne16_whill.yaml`
- Launch log: `launch.log`
- Bag-play log: `bag-play.log`
- whill_lab0_ros2 commit: `aed1e4d5ff87dd929713775f3a788d4c6049461b` (dirty)

## Initial extrinsic guess passed to GRIL-Calib

- trans_IL (IMU->LiDAR): (0.104136, 0.411548, 0.323704) m
  — noetic origin, audit §4.1 で TF chain と 1 µm 未満で一致確認済
- Rot_IL (IMU->LiDAR): **identity** (upstream default)
  — audit §4.4 シナリオ A に整合、GRIL-Calib は identity から収束させる

- Result file: not produced (insufficient motion excitation)
- Required next step: record a motion bag per
  `docs/ja/m5r-imu-diagnostic.md` §2 (figure-8 + accel/decel + in-place
  rotation, indoor flat floor, 3-5 min)

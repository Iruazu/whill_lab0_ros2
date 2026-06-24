# M5-R: `/velodyne_points` 1 Hz failure mode — root cause and RMW choice

Language: [日本語](../ja/m5r-rmw-cyclonedds.md) | [English](m5r-rmw-cyclonedds.md)

Diagnostic record for the 2026-06-24 recording session where
`/velodyne_points` dropped from its nominal 10 Hz to roughly 1 Hz.
This is the primary source of truth for the measurement evidence and
the permanent remediation (RMW = CycloneDDS). The "Runtime environment
prerequisites" section in CLAUDE.md and the §recording section of
`docs/ja/m5r-pipeline.md` defer to this document.

See [日本語版](../ja/m5r-rmw-cyclonedds.md) for the full report —
this stub exists per the i18n convention (ADR-0001). Key facts:

- **Symptom**: `/velodyne_points` at ~0.94 Hz, std dev > 1 s, with the
  upstream `/velodyne_packets` healthy at 9.857 Hz.
- **Eliminated**: LiDAR, network, `velodyne_driver_node`, subscription
  set-up, QoS, CPU governor, system load — all proven healthy or
  benign in isolation.
- **Root cause**: FastDDS (default RMW on humble) intermittently
  stalls when delivering large messages like `velodyne_msgs/VelodyneScan`
  (76-packet container). Small packets on the same transport are fine.
- **Fix**: switch RMW to CycloneDDS. After
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `/velodyne_points`
  stabilises at 9.1–9.5 Hz with std dev 0.03 s.
- **Side finding**: the CPU governor resets to `powersave` on reboot
  on this Alienware x15 R2; recording requires
  `sudo cpupower frequency-set -g performance` per session.
- **Existing bag**: `2026-06-24-loop-outdoor-ext/bag` was recorded
  before the FastDDS failure mode hit (9.86 Hz, 100 Hz IMU verified);
  no re-record needed.

# whill_lab0_ros2 docs

Project-level documentation for the noetic → humble migration. Browse from
this index or jump straight to a milestone document.

## Strategy

- [Migration plan](migration-plan.md) — package inventory, Group A/B/C
  classification, branch / PR convention.

## Carry-over from the noetic stack

- [LiDAR ↔ IMU extrinsics inherited from noetic](m3-extrinsics-from-noetic.md)
  — calibration values from `whill_lab0/FAST_LIO/config/velodyne.yaml`,
  used as the M4 starting point.

## Milestones

| | Document | Status |
|--|----------|--------|
| M1 | [Environment setup](m1-environment-setup.md) | done |
| M2 | [WHILL core driver on real hardware](m2-whill-core.md) | done |
| M3 | [Sensor stack](m3-sensors.md) | done (PR #4 merged 2026-05-07) |
| M4 | [Localization — FAST-LIO](m4-localization.md) | done (PR #6 merged 2026-05-08) |
| M5 | (Navigation — pedestrian flow / route / obstacle detection) | not yet drafted |
| M6 | (Bringup integration + on-vehicle validation) | not yet drafted |

## Session logs

Time-stamped narrative records of investigation work — kept so future
contributors don't repeat the same dead ends.

- [2026-05-06](session-2026-05-06.md) — M2 wrap-up: cold-boot quirk on
  Model CR2, fork patch, end-to-end verification.
- [2026-05-07](session-2026-05-07.md) — M3 wrap-up: chair-mounted
  three-sensor bringup, IMU lifecycle race fix, RealSense model
  correction (D455 → D435), Velodyne netplan, static + drive bags
  for M4 input.
- [2026-05-08](session-2026-05-08.md) — M4 baseline: FAST-LIO
  bringup, the identity-extrinsic detour and recovery, three-run
  reproducibility study showing capture quality dominates.

## Conventions

- One document per milestone, named `mN-<slug>.md`. Each ends with a
  `Status` table that PR review can check off.
- Session logs are dated `session-YYYY-MM-DD.md` and capture the *path*
  taken (including misdiagnoses), not just the conclusion.
- External authoritative references (vendor PDFs, upstream READMEs) are
  linked, not copied — but their *interpretation* lives here so we own the
  understanding.

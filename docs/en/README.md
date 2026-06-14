# whill_lab0_ros2 docs

Language: [日本語](../ja/README.md) | [English](README.md)

Project-level documentation for the noetic → humble migration. Browse from
this index or jump straight to a milestone document.

## Strategy

The authoritative forward-planning document is the platform-pivot policy
([`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)).
It defines the post-pivot phases M4-R through M9, the prohibited directions
(frozen M5-d/M5-e, identity-`tf_bridge` re-use, runtime FAST-LIO hardening),
and the per-phase acceptance criteria.

The earlier [migration plan](migration-plan.md) remains as the execution
record of the initial M1–M3 noetic → humble port; it is no longer the
source of truth for what to build next.

## Planning, research, and decisions

The subdirectories under `docs/` separate three different kinds of writing:

- [`plans/`](plans/) — multi-phase plans authored by `pm-orchestrator`. Each
  plan states acceptance criteria and prohibitions. New phases start here.
  - [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
- [`research/`](research/) — technical surveys authored by `research-analyst`,
  used as input to ADRs and plans.
  - [`research/2026-06-11-localization-survey.md`](research/2026-06-11-localization-survey.md)
- [`decisions/`](decisions/) — Architecture Decision Records (`NNNN-*.md`). Drafts may
  come from agents but the human authors the `accepted` line. Open ADR candidates
  are listed in [the platform-pivot policy §7](plans/2026-06-11-platform-pivot.md).
  Filed ADRs:
  - [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) — docs bilingualisation policy

## Carry-over from the noetic stack

- [LiDAR ↔ IMU extrinsics inherited from noetic](m3-extrinsics-from-noetic.md)
  — calibration values from `whill_lab0/FAST_LIO/config/velodyne.yaml`,
  used as the M4 starting point.
- [Legacy repo index](legacy-index.md) — entry-point map to `~/whill_lab0/`,
  used as the starting point of the `legacy-archaeologist` agent.

## M4-R prerequisites

- [ModelCr2State unit pinning](m4r-whill-units.md) — required before M4R-1 (case 1: add `/whill/odom` publisher to the ros2_whill fork). Determines motor_angle/motor_speed units on real hardware.

## M5-R prerequisites

- [CUDA Toolkit 12.4 and cuDNN 8 setup](m5r-cuda-setup.md) — required to build GLIM (M5-R first candidate)

## Milestones

Initial roadmap (M1–M5) is complete; the post-pivot roadmap (M4-R through
M9) is tracked in the platform-pivot policy §4 and on the README. Per-phase
documents below are the historical execution records.

| | Document | Status |
|--|----------|--------|
| M1 | [Environment setup](m1-environment-setup.md) | done |
| M2 | [WHILL core driver on real hardware](m2-whill-core.md) | done |
| M3 | [Sensor stack](m3-sensors.md) | done (PR #4 merged 2026-05-07, PR #5 wrap-up merged 2026-05-08) |
| M4 | [Localization — FAST-LIO](m4-localization.md) | done (PR #6 merged 2026-05-08) |
| M5 | [Navigation — Nav2](m5-navigation.md) | M5-a/b/c/d done (PR #7 merged 2026-05-20); M5-d continuation and M5-e frozen by the 2026-06-11 platform pivot |
| M4-R … M9 | — (per-phase docs not yet drafted) | see [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §4 |

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
- Language-specific docs are placed in parallel under `docs/ja/` and `docs/en/`
  (policy: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md)). Once
  Issue #15 closes, every document under `docs/` is bilingualised.

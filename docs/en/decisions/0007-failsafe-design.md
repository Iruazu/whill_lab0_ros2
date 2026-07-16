# ADR 0007: Failsafe node + twist_mux design (M6-R)

Language: [日本語](../../ja/decisions/0007-failsafe-design.md) | [English](0007-failsafe-design.md)

- Status: **proposed** (drafted at M6R-3 kickoff, to be accepted on M6R-5 completion)
- Date: 2026-07-14
- Deciders: Iruazu (approval pending)

> **Translation status**: this English version is a stub during the proposed
> phase. The authoritative text is [`../../ja/decisions/0007-failsafe-design.md`](../../ja/decisions/0007-failsafe-design.md).
> Full English translation will be produced when the ADR is promoted to
> `accepted` at M6R-5.

> **2026-07-14 scope-reduction note**: M6R-3 is being implemented as a
> "lite" subset for the 2026-08-01 demo (operator walks beside the chair
> with physical joystick override, so hysteresis / jump detection / G4
> hardware trials are deferred). See §Demo-scope reduction in the
> Japanese version for the full lite-vs-full delta table and the
> restoration path.

## Summary

Adds a `failsafe_node` to the `whill_safety` package (created at M6R-2) plus
a `twist_mux` gate on `/cmd_vel`. The failsafe subscribes to three signals —
manual `/reinitialization_requested`, localizer `/alignment_status` (NDT
divergence), and `/pcl_pose` continuity (silent-stall detection) — and
publishes zero twist on `/cmd_vel_safety` whenever any of them trips. The
mux gives safety priority 100 and navigation priority 10 so a zero-twist
publication wins.

Thresholds derive from the 2026-07-12 M6R-2 live acceptance and the
2026-07-14 verify走行 measurements (fitness range 0.02-0.3, /pcl_pose ~10 Hz):
`FITNESS_MAX=1.0`, `WINDOW_S=2.0`, `PCL_POSE_TIMEOUT_S=1.0`, `SAFE_HOLD_S=3.0`.

See the Japanese version for the full decision, rejected alternatives, and
promotion criteria.

## Layer D — Forward sector perception gate (2026-07-16 addendum, proposed)

**Trigger**: 2026-07-16 field found V2 (stop for person 3-4 m ahead)
failing even with `use_collision_detection: true` + person visible on
`local_costmap`. RPP's collision reach is
`max_allowed_time_to_collision_up_to_carrot × desired_linear_vel =
1.0 × 0.3 = 0.3 m`, evaluated only along the carrot (lookahead 0.8 m).
Once the planner replans around the person the "obstacle on path"
condition no longer holds. The "stop for obstacle → resume on clear"
demo requirement is not implemented in any Nav2 layer.

**Rejected**: extending RPP's own reach (mixes stop/avoid
responsibilities, still coupled to carrot geometry).

**Adopted**: add Layer D to `failsafe_node`. Subscribes to `/scan`
(reliable, ~10 Hz, base_link frame from p2ls), counts points inside a
forward sector (±30°, 0.5-2.0 m). ≥ 5 points → gate `/cmd_vel` via
`/cmd_vel_safety`; continuous clear for 0.5 s releases (same
re-latch pattern as Layer A). Rationale for every value + BT
interaction (progress_checker / recovery) is in the Japanese version.

**V2 / V3 redefinition**: V2 = "person enters 1.5-2 m forward sector
→ `/cmd_vel = 0` within 1 s + `D:forward_blocked` in log"; V3 =
"person exits sector → Layer D released within 1 s + Nav2 resumes";
V6.4 added = "verify sector geometry at ±30° boundaries, distances
1.5 / 2.0 / 2.5 m".

**Operational gate** (mandatory pre-drive check): documented in the
demo prep checklist.

**Promotion criteria**: V2 + V3 + V6.4 PASS on the field, plus V4 30-
min drive with zero false trips against static path-side buildings /
trees.

# ADR 0007: Failsafe node + twist_mux design (M6-R)

Language: [日本語](../../ja/decisions/0007-failsafe-design.md) | [English](0007-failsafe-design.md)

- Status: **proposed** (drafted at M6R-3 kickoff, to be accepted on M6R-5 completion)
- Date: 2026-07-14
- Deciders: Iruazu (approval pending)

> **Translation status**: this English version is a stub during the proposed
> phase. The authoritative text is [`../../ja/decisions/0007-failsafe-design.md`](../../ja/decisions/0007-failsafe-design.md).
> Full English translation will be produced when the ADR is promoted to
> `accepted` at M6R-5.

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

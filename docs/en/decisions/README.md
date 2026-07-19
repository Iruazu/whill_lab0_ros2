# Architecture Decision Records (ADR)

Language: [日本語](../../ja/decisions/README.md) | [English](README.md)

This directory records the architectural, governance, and operational decisions of this repository in ADR form.

## Naming convention

`NNNN-<short-kebab-case-slug>.md` (e.g. `0001-docs-i18n.md`, `0002-localizer-choice.md`). The number increments with each new ADR; no duplicates. The slug is lowercase ASCII with hyphens (a romanised Japanese slug is acceptable but match the existing files).

## Required sections

1. Header — title, Status (proposed / accepted / superseded by NNNN / deprecated), Date, Deciders.
2. Context — why this decision is needed. Technical, organisational, and historical facts.
3. Decision — what is adopted. Write it as observable rules or structure.
4. Alternatives — other options considered and why they were rejected. Record at least one.
5. Consequences — what is gained, what is lost, and the follow-up work caused by the decision.

## Status update flow

- An agent or human files the ADR as `proposed`.
- After the user reviews, the user edits this file to settle the `accepted` line. (An agent does not write `accepted` by itself.)
- If a later ADR overturns it, rewrite the status as `Status: superseded by NNNN`.

## Index

Check the highest number here before taking the next ADR number; add one row when filing a new ADR.

| No. | Title | Status |
|-----|-------|--------|
| 0001 | Bilingualisation policy for docs (Japanese / English) | accepted |
| 0002 | (unused) | — |
| 0003 | M5-R map-building SLAM final choice | accepted |
| 0004 | M5-R dynamic-object removal tool choice | accepted |
| 0005 | `docs/maps/<site>/` map artifact convention | proposed |
| 0006 | Runtime localizer choice (M6-R) | proposed |
| 0007 | Failsafe node + twist_mux design (M6-R) | proposed |
| 0008 | (unused; leftover of the m6r4 plan's candidate-number drift) | — |
| 0009 | pointcloud_to_laserscan height band + QoS bridge (M6-R) | accepted |
| 0010 | Keep Nav2 planner + controller and set `allow_unknown: false` (M6-R) | proposed |
| 0011 | Ground removal preprocessing — Patchwork++ core + own ROS 2 wrapper (M6-R) | accepted |
| 0012 | whill_dispatch Web-boundary interface style (M7) | proposed (ja only) |

## Related

- Active policy: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §7 lists ADR candidates.
- `CLAUDE.md`: the file-location convention section.

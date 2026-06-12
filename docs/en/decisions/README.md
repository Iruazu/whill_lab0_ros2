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

## Related

- Active policy: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §7 lists ADR candidates.
- `CLAUDE.md`: the file-location convention section.

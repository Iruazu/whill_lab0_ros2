# ADR 0005: `docs/maps/<site>/` map artifact convention

Language: [日本語](../../ja/decisions/0005-maps-spec.md) | [English](0005-maps-spec.md)

- Status: proposed
- Date: 2026-06-21
- Deciders: Iruazu (pending review)

## Context

The policy document
([`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md))
§6 acceptance criterion (3) requires that M5-R deliver
"`docs/maps/<site>/` containing pcd / pgm / yaml plus metadata covering the
acquisition date, route, and weather." The M5-R execution plan
([`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md))
§M5R-5 decomposed this into "fix it as a directory convention plus a README
template plus a `metadata.yaml` schema," which this Issue (#47) made
concrete.

Once frozen, this convention will be referenced for a long time:

- M5R-6 (#50, occupancy-grid conversion) targets it as the output path,
- M5R-7 (#51, pipeline integration) treats it as the documented E2E
  endpoint,
- M6-R (scan-to-map localizer + Nav2 obstacle layer re-enable) treats it as
  the input path,
- M9 / outdoor expansion (campus production routes, GNSS integration) will
  extend the metadata schema.

Because so many downstream phases will touch this convention, we record not
just the convention itself but the rationale and the rejected alternatives
as an ADR.

## Decision

1. **The canonical convention lives in
   [`../../maps/README.md`](../../maps/README.md)**. This ADR retains the
   rationale and rejected alternatives; the convention itself is edited in
   the README. To keep the ADR DRY, the details (directory layout,
   `metadata.yaml` schema, gitignore rules, `_template/` usage, treatment
   of `lab-legacy-m5b/`) live in the README.
2. **Directory structure**: each `docs/maps/<site>/` contains `static.pcd`,
   `occupancy.pgm`, `occupancy.yaml`, `metadata.yaml` (and optionally
   `README.md`). Intermediate artifacts (bags, raw SLAM PCDs, etc.) live in
   a separate `docs/m5r-bench-data/` (its convention is fixed in M5R-7).
3. **`metadata.yaml` required fields**: `acquired_at`, `route_summary`,
   `weather`, `slam_method`, `source_bag`, `commit`. Optional:
   `slam_params`, `erasor_params`. When extending the schema, update the
   table in the README first.
4. **gitignore rule**: `docs/maps/**/*.pcd` is excluded recursively (PCDs
   are tens to hundreds of MB). `.pgm` / `.yaml` / `.md` are tracked.
5. **`_template/` workflow**: start a new site with `cp -r
   docs/maps/_template docs/maps/<site-name>`. Fill in every placeholder
   (`<...>`) before commit.
6. **i18n exception**: `docs/maps/README.md` itself is exempt from
   ADR-0001's rule "narrative docs under `docs/{ja,en}/` are
   bilingualised" and stays Japanese-only. The reason is the same as for
   `docs/m4r-bench-data/README.md`: operational registries are read only
   by repo workers, and the cost of maintaining the operational details in
   two languages exceeds the benefit. The ADR itself remains bilingual per
   ADR-0001.

## Alternatives considered

- **Bilingualise every file** (`docs/{ja,en}/maps/`):
  the cost of maintaining the operational registry in two languages
  (duplicated updates on every change, duplicated `_template/`) exceeds the
  benefit (discoverability for English readers). The
  `docs/m4r-bench-data/` precedent already settles this. Rejected.
- **Define the convention via `docs/maps/<site>/.gitkeep`**:
  if the convention is expressed only via `.gitkeep`, downstream
  contributors can easily create sites without `metadata.yaml`, drifting
  from the convention. Forcing `_template/` as the starting point makes
  drift less likely. Rejected.
- **Track PCDs via git LFS**:
  LFS adds operational overhead at lab scale (storage quota management,
  `lfs fetch` in CI / CD, PR review diff display, ...). PCDs are
  regeneratable from the source bag plus SLAM parameters, so it makes
  more sense to track the regeneration recipe than the artifact itself.
  Rejected.
- **Delete `docs/m5-maps/` instead of renaming**:
  `velodyne_whill.yaml` and `nav_launch.py` reference the old paths
  directly; outright deletion would break active config. Renaming to
  `lab-legacy-m5b/` and marking the directory as "pre-freeze prototypes"
  is safer; once M5R-7 re-aims those references at the new convention,
  the directory becomes a deletion candidate. Rejected (rename adopted).
- **Lint `metadata.yaml` with a JSON Schema**:
  useful for catching schema violations, but currently `metadata.yaml` is
  authored mostly by hand (M5R-7 will auto-fill some fields like `commit`
  but not all), and the cost of maintaining a JSON Schema is not yet
  justified. Worth re-evaluating once site count grows in M9 and beyond.
  Rejected for now.

## Consequences

What we gain:

- M5R-6 / M5R-7 / M6-R are not blocked on "where do inputs / outputs go."
- Long term (campus routes → multi-site → outdoor GNSS integration), every
  site lines up on the same schema regardless of when it was added.
- The regeneration recipe (source bag + parameters) is always preserved in
  `metadata.yaml`, so a site can be rebuilt after SLAM / ERASOR
  parameter tuning.

What we lose (costs):

- Every new site requires filling in `metadata.yaml` placeholders. M5R-7
  will provide auto-fill scripts for `commit` and `acquired_at` to lighten
  the load.
- Schema changes (adding fields, etc.) need `_template/` and every
  existing site to be brought in line. Trivial during M5-R (few sites),
  managed via follow-up ADRs in M9 and beyond.

Follow-ups:

- M5R-6 (#50): point the occupancy-grid conversion script at this
  convention's output paths (`docs/maps/<site>/occupancy.{pgm,yaml}`).
- M5R-7 (#51): write the E2E pipeline doc (bag → SLAM → ERASOR → occupancy
  → `docs/maps/<site>/` placement) and the auto-fill scripts for `commit`
  etc.
- M6-R: the scan-to-map localizer takes `docs/maps/<site>/static.pcd` as
  input and publishes `map -> odom`. The Nav2 obstacle layer takes
  `docs/maps/<site>/occupancy.yaml` via `map_server`.
- M9 / outdoor expansion: when GNSS-related fields (`gnss_used`,
  `utm_zone`, ...) are added to `metadata.yaml`, supersede this ADR with a
  new one.

## Related

- Policy document: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §3.1 (offline-online split), §6 (3) (acceptance criterion), §7 (ADR
  candidate list, "ADR-0005 candidate: settle the `docs/maps/<site>/`
  convention")
- M5-R execution plan: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §M5R-5, §11 (lists this ADR as "ADR-0005 candidate")
- Canonical convention: [`../../maps/README.md`](../../maps/README.md)
- Rename record: [`../legacy-findings/2026-06-21-m5b-maps-renamed.md`](../legacy-findings/2026-06-21-m5b-maps-renamed.md)
- Related issues: #47 (this Issue, M5R-5), #50 (M5R-6), #51 (M5R-7)

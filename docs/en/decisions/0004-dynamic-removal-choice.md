# ADR 0004: M5-R dynamic-object removal tool choice

Language: [日本語](../../ja/decisions/0004-dynamic-removal-choice.md) | [English](0004-dynamic-removal-choice.md)

- Status: accepted
- Date: 2026-06-21
- Deciders: Iruazu

## Context

The platform-pivot plan
[`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
§3.3 (candidate table) prescribed dynamic-object removal as:

> | Role | First candidate | Alternative | Reason |
> |---|---|---|---|
> | Dynamic-object removal | ERASOR family | Removert | Fast; preserves static points. Offline, so no on-vehicle constraints. |

The M5-R execution plan
[`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
§Issue M5R-4 inherits this and assumes ERASOR carries an
``Apache-2.0`` license. This ADR records that the assumption broke and
replaces the choice.

### How the assumption broke

While starting Issue #49, DUFOMap surfaced as an additional candidate.
Verifying each candidate's license and maintenance state via ``gh
api`` against the upstream repos showed:

| Candidate | Upstream | License | Maintenance | Custom-data support |
|---|---|---|---|---|
| ERASOR v1 | [`LimHyungTae/ERASOR`](https://github.com/LimHyungTae/ERASOR) | **GPL-3.0** (Issue #49's "Apache-2.0" was incorrect) | ROS 1 melodic only, no active maintenance | Requires standing up a ROS 1 stack |
| ERASOR2 | [`url-kaist/ERASOR2`](https://github.com/url-kaist/ERASOR2) | **GPL-3.0** | Active | SemanticKITTI input format only; no adapter for custom data |
| Removert | [`irapkaist/removert`](https://github.com/irapkaist/removert) | **No LICENSE file** (= not adoptable; code without an explicit license can not safely depend on this repo) | ROS 1 noetic-centric | ROS 1 stack required |
| GLIM internal `dynamic_remover` | [`koide3/glim_ext`](https://github.com/koide3/glim_ext) | MIT | Active | — | The public module list does not contain a "dynamic removal" equivalent (verified 2026-06-21) |
| DUFOMap | [`KTH-RPL/dufomap`](https://github.com/KTH-RPL/dufomap) | **BSD-3-Clause** | Active (commits past 2024) | Python API via ``pip install dufomap``; ROS-independent; works on Ubuntu 22.04 + Python 3.10 |

License axis: §3.4 of the parent plan says "keep the operational
stack permissive (MIT/BSD/Apache); GPL-family code is restricted to
use as an offline map-building tool in a separated process". Dynamic
removal sits in the offline map-building phase, so GPL is technically
allowed, but this ADR opts for the more conservative permissive route
because (a) M6-R has not yet locked which pieces leak into the
operational stack, so staying permissive keeps later decisions
cheaper, and (b) the third-party manifest (``whill_lab.repos``) is
currently uniformly permissive — adding one GPL entry would require a
license-inventory exception at company-handoff time.

Technical axis: ERASOR v1 / Removert are ROS 1 only and do not work
in ROS 2 humble. ERASOR2 is the ROS 2 port but takes SemanticKITTI
format exclusively, which our GLIM keyframe dumps do not satisfy.
DUFOMap exposes a ROS-independent Python API, so a GLIM-to-PCD
converter is the only bridge needed (implemented in this Issue).

## Decision

**Adopted: DUFOMap** (`KTH-RPL/dufomap`, BSD-3-Clause, `pip install
dufomap`).

Bridge implementation (new in this Issue):

- `scripts/m5r_glim_to_pcd.py` — converts GLIM keyframe dirs
  (`NNNNNN/points_compact.bin` + `data.txt`'s `T_world_origin`) into
  per-keyframe PCDs annotated with PCL `VIEWPOINT` headers.
- `scripts/m5r_run_dufomap_core.py` — wraps DUFOMap's Python API.
- `scripts/m5r_run_dufomap.sh` — single-command orchestrator over the
  two above.
- `scripts/m5r_dufomap_diff.py` — before/after PCD overlay for visual
  inspection.

DUFOMap parameter defaults come from upstream
`KTH-RPL/dufomap/assets/config.toml` verbatim (`resolution=0.1`,
`inflate_hits_dist=0.2`, `inflate_unknown=2`). Tuning guidance lives
in [`../m5r-pipeline.md`](../m5r-pipeline.md).

This ADR overrides the "dynamic-object removal" row of the parent
plan's §3.3 candidate table. The parent plan body is not rewritten —
the override is recorded here and downstream code refers to this ADR.
(The parent plan is referenced by multiple ADRs and freezing changes
to it would be costly.)

## Alternatives considered

### ERASOR v1 (`LimHyungTae/ERASOR`)

- License: GPL-3.0 (Issue #49's "Apache-2.0" was incorrect)
- ROS 1 melodic only, no active maintenance
- Rejected: this repo is ROS 2 humble; standing up a separate ROS 1
  stack is not worth the effort. GPL also requires a §3.4 exception.

### ERASOR2 (`url-kaist/ERASOR2`)

- License: GPL-3.0
- Input format: SemanticKITTI only (paired label + velodyne_bin)
- Rejected: writing an adapter from the GLIM keyframe dir to
  SemanticKITTI format (which requires synthesised semantic labels —
  an unsolved problem of its own) is heavier than the DUFOMap bridge
  (which just writes a PCD header). Also GPL-3.0 loses against the
  permissive policy.

### Removert (`irapkaist/removert`)

- License: **no LICENSE file** (the upstream repo has no
  LICENSE / COPYING and no explicit statement in the README)
- Rejected: code without an explicit license is incompatible with
  this repo's BSD-3-Clause stance and incompatible with future
  company-handoff licensing decisions. Excluded before any technical
  comparison.

### GLIM internal `dynamic_remover`

- Upstream: the published module list of `koide3/glim_ext` does not
  contain a "dynamic removal" equivalent (verified via
  `gh api repos/koide3/glim_ext/contents` on 2026-06-21)
- Rejected: the feature does not exist. Recording this explicitly so
  the "maybe GLIM handles it" phantom does not haunt M6-R.

### Self-rolled implementation

- Rejected: dynamic-object removal is a research field of its own
  with sufficient benchmark + paper accumulation. Self-rolling would
  burn time unrelated to this Issue's scope (M5-R pipeline assembly).

## Consequences

What we gain:

- Adding a single BSD-3-Clause dependency keeps the parent plan's
  §3.4 permissive policy intact.
- Install is `pip install dufomap` only; verified on the lab host
  (Ubuntu 22.04 + Python 3.10). The converter is implemented in this
  Issue. The actual DUFOMap run is handed to the user as on-host
  verification.
- ROS-independence means no dynamic-removal runtime code can leak
  into the on-vehicle stack (DUFOMap is used only in the offline
  map-building stage).

What we lose (costs):

- A new conversion layer from GLIM keyframe dirs to per-scan PCDs is
  required, because DUFOMap takes per-scan PCD + VIEWPOINT headers.
  (Implemented as `scripts/m5r_glim_to_pcd.py` in this Issue.)
- The parent plan §3.3 candidate table now disagrees with what we
  actually use. We resolve via override-by-ADR rather than rewriting
  the parent, but agents reading only the parent may be momentarily
  confused. Updating CLAUDE.md's "known issues" section is out of
  scope for this Issue.

Follow-ups:

- M5R-4 (this Issue #49): DUFOMap real run + static PCD + visual
  confirmation (the run itself is user-side verification).
- M5R-6 (#50): consume `static.pcd` as input to occupancy-grid
  conversion.
- M5R-7 (#51): the E2E pipeline doc (bag → GLIM → DUFOMap →
  occupancy) finalises both the cross-reference to this ADR and the
  `dufomap_params` field in `docs/maps/<site>/metadata.yaml`
  (renaming / reusing ADR-0005's optional `erasor_params` field).
- Issue #49 body: this PR only renames the script files
  (`m5r_run_erasor.sh` → `m5r_run_dufomap.sh`, `m5r_erasor_diff.py`
  → `m5r_dufomap_diff.py`). Editing the Issue body itself is left as
  a separate task.

## Related

- Parent plan (overridden here): [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §3.3 (the "dynamic-object removal" row of the candidate table),
  §3.4 (license policy)
- M5-R execution plan: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §M5R-4 (Issue #49), §11 (lists this ADR as "ADR-0004 candidate")
- Predecessor ADRs: [`0003-mapping-slam-choice.md`](0003-mapping-slam-choice.md)
  (adopted SLAM = GLIM, defines this ADR's input format),
  [`0005-maps-spec.md`](0005-maps-spec.md) (`docs/maps/<site>/`
  convention, defines this ADR's `static.pcd` output target)
- Related issues: #49 (origin of this ADR), #50 (M5R-6 occupancy),
  #51 (M5R-7 integration)
- Pipeline document: [`../m5r-pipeline.md`](../m5r-pipeline.md)
- DUFOMap upstream: [KTH-RPL/dufomap](https://github.com/KTH-RPL/dufomap)
  (BSD-3-Clause, ROS-independent Python API)

# Rename record: legacy M5-b maps directory (2026-06-21)

Language: [日本語](../../ja/legacy-findings/2026-06-21-m5b-maps-renamed.md) | [English](2026-06-21-m5b-maps-renamed.md)

## What / why / when

On 2026-06-21, Issue #47 (M5R-5: establish the `docs/maps/<site>/` artifact
convention) renamed the old `docs/m5-maps/` directory to
`docs/maps/lab-legacy-m5b/` and introduced the new convention in
`docs/maps/README.md`.

### Background

The M5-R plan
(`docs/en/plans/2026-06-21-m5r-execution.md`) introduces the
`docs/maps/<site>/` convention as the storage location for the final outputs
of the mapping pipeline (bag → SLAM → ERASOR → occupancy grid). This is the
operational realisation of acceptance criterion §6 (3) in the policy
document. The old `docs/m5-maps/` directory held the prototype outputs from
M5-b (2026-05, predating the M5-d / e era frozen by the policy document),
but:

- its directory name (`docs/m5-maps/`) collides namespace-wise with the new
  `docs/maps/<site>/` convention,
- the M5-b prototypes do not satisfy the new metadata requirements
  (`metadata.yaml`, SLAM identification, ERASOR parameters, commit SHA,
  etc.),
- and once we start placing the new `lab-loop` site under the convention,
  having both `docs/m5-maps/` and `docs/maps/lab-loop/` adjacent to each
  other gets confusing fast.

So the M5-b prototypes are filed away under a name that marks them as
"pre-freeze prototypes" rather than current artifacts. We chose rename over
delete because `velodyne_whill.yaml` and `nav_launch.py` currently reference
the old paths directly; outright deletion would break active config.
`lab-legacy-m5b/` itself becomes a deletion candidate once M5R-7 (#51)
re-aims those references at the new convention.

### Physical operations

```bash
git mv docs/m5-maps/lab.pgm  docs/maps/lab-legacy-m5b/lab.pgm
git mv docs/m5-maps/lab.yaml docs/maps/lab-legacy-m5b/lab.yaml
mv     docs/m5-maps/lab.pcd                    docs/maps/lab-legacy-m5b/
mv     docs/m5-maps/global_2026-06-04_10min.pcd docs/maps/lab-legacy-m5b/
rmdir  docs/m5-maps
```

`lab.pcd` was tracked once at the M5-b commit `6d8b299` but was subsequently
ignored when `docs/m5-maps/*.pcd` was added to `.gitignore`, removing it
from the git index. This rename therefore appears in git history only as
"delete `docs/m5-maps/lab.pcd` from index + add new (still ignored)
`docs/maps/lab-legacy-m5b/lab.pcd`". The new path continues to be ignored
via the `docs/maps/**/*.pcd` rule (the old `docs/m5-maps/*.pcd` rule was
consolidated into this single recursive pattern in the same Issue).

`global_2026-06-04_10min.pcd` was never tracked, so git sees no change at
all — only the working tree moves.

### Impact and follow-ups

Three places in active code / config referenced the old path and were
updated to match:

- `src/whill_localization/config/velodyne_whill.yaml:24` (`map_file_path`)
- `src/whill_navigation/launch/nav_launch.py:55` (`default_map_yaml`)
- `src/whill_navigation/config/nav2_params.yaml:212` (comment near
  `map_server.yaml_filename`)

The following references to the old path were left intact deliberately, all
of which are historical narrative:

- `docs/{ja,en}/session-2026-05-08.md` (M5-b session log)
- `docs/{ja,en}/m5-navigation.md` (M5 milestone record)
- `docs/{ja,en}/plans/2026-06-21-m5r-execution.md` §3.2 / §M5R-5 (the plan
  itself, which forecasted this rename. Keeping the forecast text unchanged
  preserves the "forecast → actual execution" relationship)
- `scripts/pcd_to_occupancy_grid.py:22-23` (docstring example from the M5-b
  era script. Issue #50 / M5R-6 plans to replace the script outright, so we
  do not touch the current docstring example in this Issue)

### Future of `lab-legacy-m5b/` itself

- Short term: continue to resolve the legacy path references from
  `velodyne_whill.yaml` and `nav_launch.py`.
- After M5R-7 (#51) re-aims those references at the new convention
  (`docs/maps/<site>/...`), no active code will need `lab-legacy-m5b/`.
  At that point it becomes a deletion candidate.
- When deletion happens, it should be a stand-alone Issue ("physically
  delete `docs/maps/lab-legacy-m5b/`") so this rename record survives as
  history.

## Related

- Policy document: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §6 (3) acceptance criterion
- M5-R execution plan: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §3.2 (legacy M5-b remnants), §M5R-5
- New convention: [`../../maps/README.md`](../../maps/README.md)
- Related issues: #47 (this Issue, M5R-5), #50 (M5R-6), #51 (M5R-7)

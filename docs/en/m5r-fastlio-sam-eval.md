# M5-R prerequisite: FAST-LIO SAM evaluation candidate (clone-on-demand)

Language: [日本語](../ja/m5r-fastlio-sam-eval.md) | [English](m5r-fastlio-sam-eval.md)

## Goal

Bring the second-candidate map-building SLAM for M5-R,
[FAST-LIO SAM (RightTr fork)](https://github.com/RightTr/FAST-LIO-SAM), to a
state where the upcoming M5R-3 (Issue #48, "GLIM vs FAST-LIO SAM on real
bags") can build and smoke-test it. Concretely:

- Pin down the upstream license situation and decide how this repository
  takes the code in.
- Document the local clone procedure and the Ubuntu 22.04 GTSAM (PPA)
  install steps.
- Provide an idempotent helper script at
  `scripts/clone_fastlio_sam_for_eval.sh` so the evaluator can reproduce
  the setup.

This document ends at "M5R-3 can start building and smoke-testing".
**Real-bag evaluation, parameter tuning, and the ADR-0003 (final SLAM
choice) write-up belong to M5R-3.**

For how this Issue maps onto the M5-R execution plan, see
[`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md)
§3.1 ("handling of `src/third_party/FAST_LIO_SAM/`").

## License status (most important)

| Aspect | Fact |
|---|---|
| Upstream repo | `https://github.com/RightTr/FAST-LIO-SAM` |
| LICENSE file | **does not exist** (`find -iname LICENSE*` returns nothing; verified via WebFetch as well) |
| README license declaration | none, no license badge |
| `package.xml` `<license>` | declares `BSD`, but no LICENSE text accompanies it in the repository, so there is no enforceable grant |
| Origin | derived from HKU-MaRS [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (**GPL-2.0**). FAST-LIO SAM adds GTSAM loop closure and smoothing on top of FAST-LIO. The GPL copyleft clauses can propagate to derivatives. |

### Interpretation under copyright law

In the absence of an explicit license grant, the work cannot be used,
copied, modified, or redistributed without the copyright holder's
permission. Practically this is **"all rights reserved"**, which is
*stricter* than permissive BSD/MIT and *more uncertain* than GPL-2.0 (the
latter at least permits redistribution under defined conditions). The
`<license>BSD</license>` line in `package.xml` is an unilateral upstream
claim; without a LICENSE document, a third party has weak grounds to
read BSD as applying to the repository as a whole.

### Treatment in this repository

The platform-pivot plan
[`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
§3.4 (license policy) states:

> GPL-family code (FAST-LIO and friends) is restricted to use as an
> *offline map-building tool* in a separated process.

FAST-LIO SAM is in this "FAST-LIO family". The upstream LICENSE absence
narrows the allowed surface further. The decisions are:

| Action | Allowed? | Rationale |
|---|---|---|
| Local clone for map-building-only evaluation | yes (this document is exactly that) | platform-pivot §3.4 "offline map-building tool in a separated process" |
| Storing only the **static PCD / occupancy grid** outputs under `docs/maps/<site>/` | yes | These are outputs against evaluation bags, not redistribution of upstream code |
| Adding a `whill_lab.repos` entry | **no** | Would silently turn every `vcs import` invocation into an implicit redistribution of license-uncertain code |
| Linking from operational packages (`whill_navigation`, `whill_localization`, ...) | **no** | Breaks the permissive-only stance of the operational stack and keeps FAST-LIO GPL propagation as live risk |
| Forking or sending fixes upstream | **out of scope** for this Issue | Strictly in-repo work here. Future need requires an ADR. |

### Why clone-on-demand instead of `whill_lab.repos`

Via `whill_lab.repos`:

- Every clone of this repository would pull the upstream automatically
  through `vcs import`.
- Including a license-uncertain upstream there makes every repo clone an
  implicit redistribution point.
- If M5-R later decides FAST-LIO SAM is not adopted, we still have to
  argue about purging the entry plus existing cached clones in each
  contributor's workspace.

Via clone-on-demand (the chosen path):

- The evaluator runs `scripts/clone_fastlio_sam_for_eval.sh` themselves.
  Accepting the license risk becomes an explicit, individual decision.
- The script requires the environment variable
  `FASTLIO_SAM_LICENSE_ACK=yes` as a misuse guard — accidental clones
  are blocked.
- When M5R-3 closes ADR-0003, we re-decide: (a) move to `whill_lab.repos`
  (adopted *and* upstream license clarified), (b) keep clone-on-demand
  permanently (adopted but license still unresolved), or (c) drop the
  candidate (not adopted).

## Setup procedure (for the evaluator)

All steps are run manually by the evaluator. A fresh clone of this
repository does **not** run them — that is the point.

### 0. Acknowledge the license caveat

```bash
export FASTLIO_SAM_LICENSE_ACK=yes
```

Without this variable, `scripts/clone_fastlio_sam_for_eval.sh` prints
the license caveat (the gist of the §"License status" above) to stderr
and exits 1. It is a misuse guard, and the act of setting it functions
as an explicit "I accept the license risk" signal.

### 1. Clone and install GTSAM (via PPA)

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash         # sets ROS_DISTRO=humble
./scripts/clone_fastlio_sam_for_eval.sh
```

The script does the following in order:

1. Verify Ubuntu 22.04, `ROS_DISTRO=humble`, and
   `FASTLIO_SAM_LICENSE_ACK=yes` (exit 1 otherwise).
2. Add the `borglab/gtsam-release-4.1` PPA and install
   `libgtsam-dev libgtsam-unstable-dev` (skipped if equivalent versions
   are already installed).
3. Clone the upstream into `src/third_party/FAST_LIO_SAM/` (or, when the
   clone already exists, fetch and fast-forward to upstream master).
4. Verify that `package.xml` declares `<name>fast_lio_sam</name>`.

The script does **not** run `colcon build`. Reasons: (a) the main
objective of this Issue is the license/setup hygiene, not the build —
build-success is the M5R-3 evaluator's call; (b) the upstream README
still lists "Full ROS2 adaptation" and "ROS2 adaptation Test" as TODO,
so master may not build cleanly today.

### 2. Build (when M5R-3 evaluator gets there)

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio_sam --symlink-install
```

Upstream also documents a `build.sh humble` wrapper. We call colcon
directly because colcon is this repository's single build interface.
If a symptom only reproduces under `build.sh`, that delta belongs in
ADR-0003 (M5R-3).

### 3. Warning about GTSAM conflicts

M5R-1 (Issue #45) already installs a source build of **GTSAM 4.3a0 to
`/usr/local/lib/libgtsam.so.4.3a0`** for GLIM. This Issue introduces
**GTSAM 4.1.1 via the PPA at `/usr/lib/x86_64-linux-gnu/libgtsam.so.4.1.1`**.
The two are ABI-incompatible. Which one CMake picks depends on its
search order:

- `find_package(GTSAM)` ordinarily prefers `/usr/local` over `/usr`, so
  the GLIM 4.3a0 build wins by default.
- If FAST-LIO SAM hard-requires GTSAM 4.1, CMake either issues a
  version-mismatch warning or fails outright.
- If needed, force the PPA version with
  `cmake -DGTSAM_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTSAM`.

When M5R-3 starts, attempt the build with nothing specified. If it
breaks, document the workaround (explicit `GTSAM_DIR`, temporarily
shadowing `/usr/local/lib/libgtsam*`, etc.) in ADR-0003 Context. The
larger task of making both versions coexist cleanly (e.g. installing the
GLIM GTSAM under a non-default prefix) is deferred to a follow-up Issue
*after* the SLAM choice is settled.

## Known uncertainties

| Item | Detail | Handled in M5R-3 by |
|---|---|---|
| Upstream ROS 2 readiness | README still lists "Full ROS2 adaptation" and "ROS2 adaptation Test" as TODO; current master may not build | If the build fails, capture a minimal repro (CMake paths, leftover roscpp bits, API drift) in ADR-0003 Context. Patches go into in-repo wrappers — direct edits to `src/third_party/` are forbidden (CLAUDE.md). |
| Upstream LICENSE added | Upstream may add a LICENSE file or explicitly declare GPL-2.0 / MIT later | When that happens, update this document and re-evaluate moving to `whill_lab.repos`. If permissive, evaluate operational-stack inclusion. |
| GTSAM conflict | see §"Warning about GTSAM conflicts" | M5R-3 will attempt the build, observe the conflict, and write the workaround to ADR-0003 |
| Sample bag | Upstream README points at an example bag | Out of scope here. M5R-3 uses the real loop-driving bag captured during M4-R bringup. |

## Handover to M5R-3 (#48)

Starting from a working clone-and-build per this document, M5R-3:

1. Runs GLIM and FAST-LIO SAM on the **same** M4-R-bringup loop-driving
   bag under matched conditions.
2. Measures the loop-closure error at the start/end (mean of three
   points on the same wall, target ≤ 0.5 m), wall time, VRAM, and
   CPU/GPU load.
3. Repeats with the dynamic-object bag (pedestrian crossing) to check
   fitness as input to downstream ERASOR.
4. Writes ADR-0003 (`docs/decisions/0003-mapping-slam-choice.md`) and
   fills the Context with the license status, build success, loop
   closure accuracy, time, and VRAM.
5. The Decision section nails down the first-choice SLAM (keep GLIM or
   switch to FAST-LIO SAM).

If ADR-0003 does **not** adopt FAST-LIO SAM, this document and
`scripts/clone_fastlio_sam_for_eval.sh` stay as history; a separate
Issue later decides deprecation.

## Related

- Strategy: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §3.3 (candidate table, FAST-LIO SAM as GLIM's alternative) and §3.4
  (license policy: GPL family restricted to offline map-building tools).
- M5-R execution plan:
  [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md)
  §3.1 (handling of `src/third_party/FAST_LIO_SAM/`) and §6 (Issue
  M5R-2 acceptance criteria).
- Sibling document: [`m5r-glim-setup.md`](m5r-glim-setup.md) — source
  build of the first-candidate GLIM.
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md)
  — new documents are authored in parallel under `docs/ja/` and
  `docs/en/`.
- Script: [`scripts/clone_fastlio_sam_for_eval.sh`](../../scripts/clone_fastlio_sam_for_eval.sh)
  — the idempotent clone helper this document is paired with.
- Related issues: #46 (this document and script) and the upcoming #48
  M5R-3 (empirical real-bag comparison and ADR-0003).

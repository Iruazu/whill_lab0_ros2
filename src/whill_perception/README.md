# whill_perception

Perception preprocessing nodes for the WHILL chair. Currently ships one
node: `patchworkpp_node`, a ground-removal filter that stands between
the VLP-16 driver and the Nav2 obstacle_layer's 2D-slice input.

## Why this package exists

Nav2 `obstacle_layer` fed a `pointcloud_to_laserscan` 2D slice at the
`base_link` horizontal plane breaks on outdoor gradient and local
unevenness: 2026-07-14 Phase B (工農研横) observed manholes, potholes,
and 5° slope crossings pushing the ground itself past the slice's
`min_height`, painting wide false-lethal bands. Raising `min_height`
cleaned the flat sections but did not fix the slope crossings —
`min_height` is a base-link-flat threshold and cannot follow the
terrain.

Ground removal *before* the 2D slice fixes this at the source. The
downstream slice then sees only obstacles standing above the terrain,
and the min/max height band can be relaxed back toward capturing low
steps / pedestrian legs (landed at 0.05 m in ADR-0009 §Verification
2026-07-15 A/B).

## Pipeline (after `whill_perception` is wired in)

```
/velodyne_points (best-effort, sensor QoS, 10 Hz)
     │
     └─> patchworkpp_node (this package)
            │
            └─> /velodyne_points_no_ground (best-effort, sensor QoS, ~10 Hz)
                   │
                   └─> pointcloud_to_laserscan_node (m6r/4-nav2-obstacle-layer)
                          │
                          └─> /scan (reliable)
                                 │
                                 └─> obstacle_layer of local/global costmaps
```

M6R4-c (PR #84) landed the remap flip: `p2ls_node.cloud_in` in
`whill_navigation/launch/nav_launch.py` now points at
`/velodyne_points_no_ground`. The upstream `/velodyne_points` topic
stays available for diagnostics and RViz overlay.

## Algorithm

Patchwork++ (KAIST Urban Robotics Lab, IROS'22). Concentric-zone model
with region-wise plane fitting + Adaptive Ground Likelihood Estimation.
The C++ core is BSD-2-Clause; the upstream ROS 2 wrapper has an
inconsistent license (`ros/LICENSE` MIT vs `ros/package.xml` GPL-3.0),
so this package writes its own permissive wrapper against the BSD core.
ADR-0011 (see `docs/decisions/`) records the licensing carve-out.

## Setup

The C++ core lives under `src/third_party/patchwork_plusplus/cpp/` and
is fetched via the workspace `whill_lab.repos`. After a fresh
`vcs import` a `COLCON_IGNORE` in the upstream `ros/` subtree is
required to keep colcon from building the GPL-declared wrapper:

```bash
cd ~/whill_lab0_ros2-m6r4b   # or your worktree root
vcs import src < whill_lab.repos
touch src/third_party/patchwork_plusplus/ros/COLCON_IGNORE
colcon build --symlink-install --packages-select whill_perception
```

`src/third_party/patchwork_plusplus/cpp/` also has a `COLCON_IGNORE`
shipped by upstream, which is correct — the core is not a ROS package,
`whill_perception`'s CMakeLists.txt pulls it in via `add_subdirectory`.

## Launch

Standalone:

```bash
ros2 launch whill_perception ground_removal_launch.py
```

Subscribes on `/velodyne_points`, publishes on
`/velodyne_points_no_ground`. All Patchwork++ parameters live in
`config/patchworkpp.yaml`; only `sensor_height` (0.79 m from the M4-R
extrinsic ledger), `min_range`, and `max_range` are overridden — the
rest stay at the Patchwork++ struct defaults.

## Bag replay verification

```bash
# Terminal A — playback (raw sensors from a previous field run)
ros2 bag play docs/m6r-bench-data/2026-07-14-verify-campus/bag/ --clock

# Terminal B — this node
ros2 launch whill_perception ground_removal_launch.py

# Terminal C — sanity
ros2 topic hz /velodyne_points          # ~10 Hz (input)
ros2 topic hz /velodyne_points_no_ground # ~10 Hz (output, matches input)
```

**PASS if:**

- `/velodyne_points_no_ground` publishes at within 1 Hz of the input
  rate for 30 s of playback
- RViz side-by-side (raw `/velodyne_points` vs
  `/velodyne_points_no_ground`, same `velodyne` frame): the ground plane
  (asphalt, ramps, manholes) disappears, vertical structures
  (buildings, poles, people) remain
- `top -p $(pgrep patchworkpp_node)` shows a single-core load ≤ 100%
  during playback (headroom target: sustained < 80% on the Alienware
  x15 R2)

## Environment (verified 2026-07-14)

Findings pinned down during bag-replay verification. New hosts running
this node must match these before Phase C.

### QoS observed on this deployment

| Topic | Publisher | Reliability | Notes |
|-------|-----------|:-----------:|-------|
| `/velodyne_points` | VLP-16 driver (live) *and* `ros2 bag play` | **RELIABLE** | Non-default for the driver, but this is what we measured on both live and bag inputs. `patchworkpp_node` subscribes with `SensorDataQoS()` (best-effort), which accepts a reliable publisher without complaint |
| `/velodyne_points_no_ground` | `patchworkpp_node` (this) | **BEST_EFFORT** | Standard sensor-data pattern; matches downstream `pointcloud_to_laserscan_node`'s subscription default |

**RViz side**: to display `/velodyne_points_no_ground`, set the
PointCloud2 display's **Topic → Reliability Policy = Best Effort**.
Default is Reliable, which will silently fail to subscribe.

### UDP receive buffer (sysctl, permanent)

Large PointCloud2 frames overflow the DDS default UDP receive buffer
during bag replay — the live driver rate does not, but bag-player
bursts do. Fixed permanently on this host:

```bash
# /etc/sysctl.d/60-ros2-dds-buffer.conf
net.core.rmem_max     = 26214400   # 25 MB
net.core.rmem_default = 26214400   # 25 MB
```

Apply after edit: `sudo sysctl --system`.

The same risk applies to Phase C field runs where a bag is being
recorded on the same host — combined record + subscribe hits the same
recv path. A host missing this sysctl setting must not run this node
plus a `ros2 bag record` in parallel.

### Silent-failure retrospective (2 incidents, closed)

Two failures were caught during 2026-07-14 verification and now have
guards + environment fixes so a fresh checkout does not re-hit them:

1. **RNR intensity column missing** — Patchwork++ core with RNR on
   rejects frames whose input matrix is Nx3 (no intensity). Since
   commit `ceb3bb3` the wrapper converts to Nx4 and adds two
   `WARN_THROTTLE(5s)` guards (input has no `intensity` field / core
   returned 0 ground + 0 non-ground).
2. **UDP fragment loss on bag replay** — fixed by the 25 MB rmem
   sysctl above.

Both are documented for the future — see [`docs/decisions/0011-ground-removal-choice.md`](../../docs/decisions/0011-ground-removal-choice.md) "Silent-failure retrospective".

## Parameters

See `config/patchworkpp.yaml`. Field data must precede parameter
sweeping — Patchwork++ struct defaults are calibrated for KITTI 64ch,
and the VLP-16 16ch story is expected to be similar but has not been
published. `sensor_height` is authoritative from the calibration
ledger; do not adjust it as a tuning knob for false positives.

## Related

- [`docs/ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../../docs/ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md) §"Follow-up: ground removal"
- ADR-0011 (proposed, in this PR): Ground removal algorithm choice + license carve-out
- Upstream: <https://github.com/url-kaist/patchwork-plusplus> (v1.4.1 pinned in `whill_lab.repos`)
- Original paper: <https://arxiv.org/abs/2207.11919>

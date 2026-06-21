#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Thin wrapper around the DUFOMap Python API for M5R-4 (Issue #49).

DUFOMap [KTH-RPL, BSD-3-Clause] ships a ``pip install dufomap`` package
whose Python API takes per-scan point clouds + sensor poses, builds an
occupancy map, propagates dynamic / free / unknown labels through the
map, and exports the surviving static cloud.

We isolate the DUFOMap call here (rather than inlining it into the
shell orchestrator) for three reasons:

* ``import dufomap`` is slow (loads native libs + threads), and pulling
  it into the shell wrapper would make ``--help`` slow too.
* The API surface (``run``, ``oncePropagateCluster``, ``outputMap``) is
  subtly version-sensitive — keeping the Python boundary thin and
  isolated means a future API drift only needs edits here, not in shell.
* Failure modes (missing pcd dir, malformed VIEWPOINT, etc.) are
  easier to surface from Python than from a shell pipeline.

Input contract: ``<data-dir>/pcd/*.pcd`` where each PCD has a PCL
VIEWPOINT header carrying the keyframe's pose in the map frame.  The
companion ``scripts/m5r_glim_to_pcd.py`` converter produces exactly
this layout from a GLIM keyframe dump.

Output: a single static PCD at ``--output``.  DUFOMap's ``outputMap``
writes to a path of its own choosing; we work around that by
post-moving the produced file to the user-specified ``--output``.

Usage:

    scripts/m5r_run_dufomap_core.py \\
        --data-dir /tmp/m5r49_staging \\
        --output /tmp/m5r49_static.pcd

Parameter defaults come from DUFOMap's upstream ``assets/config.toml``
and are documented in ``docs/ja/m5r-pipeline.md``.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

import numpy as np


# We import dufomap lazily inside main() so that --help works on hosts
# where DUFOMap is not (yet) installed. The shell orchestrator
# (m5r_run_dufomap.sh) checks importability separately so the user gets
# a clear "pip install dufomap" hint up front.
def _import_dufomap():
    try:
        from dufomap import dufomap  # type: ignore
        from dufomap.utils import pcdpy3  # type: ignore
    except ImportError as exc:
        print(
            "ERROR: failed to import dufomap. Install with "
            "`pip install dufomap`. Underlying error: "
            f"{exc}",
            file=sys.stderr,
        )
        raise
    return dufomap, pcdpy3


def collect_pcds(data_dir: Path) -> list[Path]:
    """Return the PCD files DUFOMap should ingest, in stable order.

    DUFOMap is order-sensitive: it builds an occupancy map incrementally
    and uses earlier scans' free-space rays to invalidate later scans'
    points.  Sorting by filename gives us the same temporal order that
    the GLIM converter emits (six-digit zero-padded keyframe index).
    """
    pcd_subdir = data_dir / "pcd"
    if not pcd_subdir.is_dir():
        raise FileNotFoundError(
            f"{pcd_subdir} not found. Did you run m5r_glim_to_pcd.py "
            f"first with --out-dir {data_dir}?"
        )
    pcds = sorted(pcd_subdir.glob("*.pcd"))
    if not pcds:
        raise FileNotFoundError(f"No .pcd files in {pcd_subdir}")
    return pcds


def _viewpoint_to_world_transform(viewpoint) -> np.ndarray:
    """Build a 4x4 world<-local transform from a PCL VIEWPOINT 7-tuple.

    VIEWPOINT order per PCL spec: (tx, ty, tz, qw, qx, qy, qz).
    Returned matrix turns keyframe-local points into world-frame points
    via ``p_world = T @ [p_local; 1]``.
    """
    from scipy.spatial.transform import Rotation
    tx, ty, tz, qw, qx, qy, qz = (float(v) for v in viewpoint)
    # scipy's from_quat expects (x, y, z, w).
    rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rot
    T[:3, 3] = (tx, ty, tz)
    return T


def run_dufomap(data_dir: Path, output: Path,
                resolution: float, d_s: float, d_p: int,
                num_threads: int) -> int:
    """Drive DUFOMap end-to-end on a directory of per-scan PCDs.

    Mirrors upstream's ``KTH-RPL/dufomap/main.py`` accumulator pattern:

    1. Construct a ``dufomap`` instance (resolution / d_s / d_p).
    2. For each scan, transform keyframe-local points into the world
       frame using the PCD's VIEWPOINT pose, append to an accumulator,
       and feed DUFOMap with ``cloud_transform=False`` (we already did
       the transform).
    3. ``oncePropagateCluster`` finalises dynamic / free / unknown
       labels.  Clustering is left off — matches upstream default and
       gives a big speedup with marginal quality loss on outdoor loops.
    4. ``outputMap(cloud_acc, voxel_map=True, file_name=<basename>)``
       writes ``<basename>.pcd`` directly to the path we want; no chdir
       or post-move dance is required (file_name kwarg added upstream
       in dufomap 1.x).
    """
    dufomap, pcdpy3 = _import_dufomap()
    pcds = collect_pcds(data_dir)

    mydufo = dufomap(resolution, d_s, d_p, num_threads=num_threads)

    # DUFOMap's outputMap() filters this accumulated cloud against the
    # map's seen-free voxels and writes the surviving (static) subset.
    # Must be in the same frame as the voxel map — i.e. world frame —
    # since the filter is spatial. Pre-allocating with float32 keeps the
    # downstream ascontiguousarray.astype(np.float32) inside outputMap a
    # no-op.
    cloud_acc = np.zeros((0, 3), dtype=np.float32)

    for i, pcd_path in enumerate(pcds):
        pcd = pcdpy3.PointCloud.from_path(str(pcd_path))
        # pcdpy3 may return float64 depending on the source PCD's TYPE
        # field; DUFOMap's run() casts to float32 internally but we cast
        # here too so the homogeneous-transform matmul stays in f64 and
        # only the final cloud is f32.
        local_pts = pcd.np_data[:, :3].astype(np.float64)
        viewpoint = list(pcd.viewpoint)
        T_world_local = _viewpoint_to_world_transform(viewpoint)
        # Homogeneous-coords transform: faster as (R @ pts.T).T + t than
        # building an Nx4 augmented array.
        world_pts = (T_world_local[:3, :3] @ local_pts.T).T + T_world_local[:3, 3]
        world_pts_f32 = world_pts.astype(np.float32)

        # cloud_transform=False because we already moved points to world.
        # Pose still needs to be world-frame (DUFOMap uses it as the
        # sensor origin for ray casting); viewpoint translation already
        # equals T_world_local[:3, 3], so it's the right value.
        mydufo.run(world_pts_f32, viewpoint, cloud_transform=False)
        cloud_acc = np.concatenate((cloud_acc, world_pts_f32), axis=0)
        print(f"[{i + 1}/{len(pcds)}] ingested {pcd_path.name} "
              f"({world_pts_f32.shape[0]} points)")

    print(f"Propagating cluster labels over {cloud_acc.shape[0]} accumulated points...")
    mydufo.oncePropagateCluster(if_propagate=True, if_cluster=False)

    # outputMap appends ".pcd" to file_name itself, so strip any suffix
    # the user passed via --output.
    if output.suffix == ".pcd":
        basename = str(output.with_suffix(""))
    else:
        basename = str(output)
    try:
        mydufo.outputMap(cloud_acc, voxel_map=True, file_name=basename)
    except Exception as exc:
        # Anything out of the native binding here means an API shape
        # change. Surface that clearly rather than letting a raw error
        # escape.
        print(
            f"ERROR: DUFOMap raised during outputMap: {type(exc).__name__}: "
            f"{exc}. Check the installed `dufomap` version against the "
            "expected API: outputMap(points: np.ndarray, voxel_map: bool, "
            "file_name: str).",
            file=sys.stderr,
        )
        return 1

    # Observed in dufomap 1.1.1: voxel_map=True appends "_voxel.pcd",
    # voxel_map=False appends ".pcd". Try both so the script keeps
    # working if the suffix convention shifts.
    candidates = [
        Path(basename + "_voxel.pcd"),
        Path(basename + ".pcd"),
    ]
    produced = next((c for c in candidates if c.is_file()), None)
    if produced is None:
        print(
            "ERROR: DUFOMap reported success but no output PCD was found "
            f"at any of: {[str(c) for c in candidates]}. Check the "
            "installed dufomap version.",
            file=sys.stderr,
        )
        return 1
    # Honour --output exactly. dufomap's file_name kwarg gets us close
    # but doesn't let us pick the suffix; rename the produced file so
    # the user sees what they asked for.
    if str(produced) != str(output):
        shutil.move(str(produced), str(output))
        produced = output

    print(f"\nStatic map written to {produced}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run DUFOMap on a directory of per-scan PCDs (e.g. the "
            "staging dir produced by scripts/m5r_glim_to_pcd.py) and "
            "emit a single static PCD."
        ),
    )
    parser.add_argument(
        "--data-dir", required=True, type=Path,
        help="Staging directory containing pcd/*.pcd",
    )
    parser.add_argument(
        "--output", required=True, type=Path,
        help="Path for the static PCD result",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Overwrite --output if it already exists",
    )
    # The three primary DUFOMap parameters; defaults come from
    # KTH-RPL/dufomap/assets/config.toml and are documented in
    # docs/ja/m5r-pipeline.md.
    parser.add_argument(
        "--resolution", type=float, default=0.1,
        help="Voxel size (m). Upstream default 0.1; lower for indoors, "
             "higher for wide outdoor scenes.",
    )
    parser.add_argument(
        "--d-s", type=float, default=0.2,
        help="inflate_hits_dist (m). Upstream default 0.2; raise for "
             "noisier sensors.",
    )
    parser.add_argument(
        "--d-p", type=int, default=2,
        help="inflate_unknown (voxel count). Upstream default 2 per "
             "the DUFOMap paper.",
    )
    parser.add_argument(
        "--num-threads", type=int, default=12,
        help="DUFOMap worker thread count (upstream main.py uses 12)",
    )
    args = parser.parse_args()

    if args.output.exists() and not args.force:
        print(
            f"ERROR: {args.output} already exists. Re-run with --force "
            "to overwrite.", file=sys.stderr,
        )
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)

    try:
        return run_dufomap(
            data_dir=args.data_dir,
            output=args.output,
            resolution=args.resolution,
            d_s=args.d_s,
            d_p=args.d_p,
            num_threads=args.num_threads,
        )
    except (FileNotFoundError, ImportError) as exc:
        # Both already printed their own error.
        if isinstance(exc, FileNotFoundError):
            print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

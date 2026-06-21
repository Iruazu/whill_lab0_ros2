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


def run_dufomap(data_dir: Path, output: Path,
                resolution: float, d_s: float, d_p: int,
                num_threads: int) -> int:
    """Drive DUFOMap end-to-end on a directory of per-scan PCDs.

    Steps mirror upstream's ``KTH-RPL/dufomap/main.py``:

    1. Construct a ``dufomap`` instance with the three primary
       parameters (resolution / inflate_hits_dist / inflate_unknown).
    2. Feed each scan with its viewpoint, leaving ``cloud_transform``
       at ``False`` so DUFOMap applies the supplied pose itself (any
       pre-transform here would double-apply).
    3. Call ``oncePropagateCluster`` to finalise dynamic / free /
       unknown labels.  Clustering is left off (matches upstream
       default) — the speed-up from skipping it is significant and the
       quality difference is marginal for our outdoor loop bag.
    4. Call ``outputMap`` which writes ``dufomap_output.pcd`` into the
       process CWD.  We then move that file to ``--output``.
    """
    dufomap, pcdpy3 = _import_dufomap()
    pcds = collect_pcds(data_dir)

    mydufo = dufomap(resolution, d_s, d_p, num_threads=num_threads)

    for i, pcd_path in enumerate(pcds):
        pcd = pcdpy3.PointCloud.from_path(str(pcd_path))
        # pcdpy3 may return float64 depending on the source PCD's TYPE
        # field. DUFOMap's run() requires float32 (upstream main.py uses
        # data['pc'].astype(np.float32)); an f64 array silently truncates
        # or raises inside the native binding.
        pts = pcd.np_data[:, :3].astype(np.float32)
        # PCL VIEWPOINT order is (tx, ty, tz, qw, qx, qy, qz). DUFOMap's
        # run() accepts the same layout — see upstream main.py.
        pose = list(pcd.viewpoint)
        mydufo.run(pts, pose, cloud_transform=False)
        print(f"[{i + 1}/{len(pcds)}] ingested {pcd_path.name} "
              f"({pts.shape[0]} points)")

    print("Propagating cluster labels...")
    mydufo.oncePropagateCluster(if_propagate=True, if_cluster=False)

    # outputMap takes a "cloud accumulator" path / handle.  Upstream's
    # main.py passes a writable handle whose target is fixed at
    # ``dufomap_output.pcd`` in CWD.  We pre-create the staging dir,
    # chdir into it for the call only, then move the result out.  This
    # keeps the user-visible output path under their control while
    # tolerating any version of DUFOMap that wires the output filename
    # internally.
    staging = output.parent / ".dufomap_tmp"
    staging.mkdir(parents=True, exist_ok=True)
    prev_cwd = Path.cwd()
    try:
        os.chdir(staging)
        # The accumulator argument is the basename DUFOMap uses for the
        # output file (some versions ignore it and always write
        # "dufomap_output.pcd" anyway). Pass the basename we want and
        # fall back to the canonical name if the version-specific
        # behaviour differs.
        mydufo.outputMap("dufomap_output", voxel_map=True)
    except Exception as exc:
        # An exception out of outputMap is almost always a DUFOMap
        # API drift (renamed kwarg, changed return type, etc.). Surface
        # that clearly rather than letting a raw RuntimeError escape, and
        # sweep the staging dir so the next --force-less re-run starts
        # clean.
        print(
            f"ERROR: DUFOMap raised during outputMap: {type(exc).__name__}: "
            f"{exc}. Check the installed `dufomap` version against "
            "scripts/m5r_run_dufomap_core.py's expected API "
            "(`mydufo.outputMap(basename, voxel_map=True)`).",
            file=sys.stderr,
        )
        shutil.rmtree(staging, ignore_errors=True)
        return 1
    finally:
        os.chdir(prev_cwd)

    produced = None
    # Cover both observed naming conventions across DUFOMap releases.
    for candidate in ("dufomap_output.pcd", "dufomap_output_voxel_map.pcd"):
        c = staging / candidate
        if c.is_file():
            produced = c
            break
    if produced is None:
        # Fall back to "anything that landed here".
        leftovers = list(staging.glob("*.pcd"))
        if not leftovers:
            print(
                f"ERROR: DUFOMap produced no PCD in {staging}. The "
                "Python API may have a different output convention than "
                "expected; inspect the directory manually.",
                file=sys.stderr,
            )
            return 1
        produced = leftovers[0]
        print(f"WARNING: using unexpected output filename "
              f"{produced.name}", file=sys.stderr)

    shutil.move(str(produced), str(output))
    # Best-effort: remove the staging dir if it's empty afterwards.
    try:
        staging.rmdir()
    except OSError:
        pass

    print(f"\nStatic map written to {output}")
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

#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Convert a DUFOMap static PCD into a Nav2 occupancy grid (.pgm + .yaml).

This is the M5R-6 (Issue #50) tail end of the M5-R map-building
pipeline: GLIM (ADR-0003) -> DUFOMap (ADR-0004, dynamic-removal) ->
*this script* -> Nav2 map_server.

It is intentionally a sibling of (not a refactor of) the older
``scripts/pcd_to_occupancy_grid.py``.  The older script was written for
the M5-b FAST-LIO map: it hard-codes a +/-20 m crop around the world
origin and ray-casts from (0, 0) on the assumption that the sensor
started at the world origin and never strayed far from it.  Those
assumptions no longer hold under the M5-R pipeline -- the DUFOMap
output is already cleaned of pedestrian streaks and may cover an
arbitrary outdoor loop whose centroid sits nowhere near the GLIM map
origin.  Rather than retrofit the old script and risk breaking
``docs/maps/lab-legacy-m5b/`` (which ``nav_launch.py`` still consumes),
we leave it alone and start clean here.

Differences from the legacy script that matter:

* The XY bounding box is computed from the input, not hard-coded.  A
  configurable ``--padding`` adds breathing room so obstacles at the
  edge of the traverse do not get clipped.
* Ray-cast has two modes selected by ``--anchor-mode``:

  * ``trajectory`` (default): read ``traj_lidar.txt`` (TUM format, from
    the GLIM run), downsample to ``--traj-stride`` m intervals, then
    for each anchor cast Bresenham rays to all occupied cells within
    ``--max-range`` m.  Result: only cells "seen from at least one
    anchor" become FREE; the middle of an untraversed courtyard stays
    UNKNOWN even if it is topologically inside the map.  This is what
    Nav2 needs to avoid planning through regions the chair never
    actually observed.
  * ``single``: keep the legacy single-anchor behaviour (occupied-cell
    centroid or ``--anchor-{x,y}``).  Radial streak artefacts are
    unavoidable with a single anchor; keep for backward compatibility
    and for offline conversions where the trajectory is unavailable.

  For trajectory mode we do NOT walk rays to *every* occupied cell in
  range — that grows quadratically with map size.  Instead we bin
  occupied cells by angle (0.5° = 720 bins) and keep the closest hit
  per bin; this preserves the "any-occ-stops-a-ray" semantics of
  Bresenham raycast while capping per-anchor work at ~720 rays.
* The YAML output matches ``docs/maps/_template/occupancy.yaml``
  exactly (including ``free_thresh: 0.196`` -- the Nav2 map_server
  default -- not the legacy 0.25 the M5-b script writes).
* No outlier-cluster filter and no clear-radius hack.  Both existed
  in the legacy script to paper over FAST-LIO drift + chair
  self-returns; DUFOMap removes the former, and the latter is moot
  since we no longer assume the chair starts at world origin.

Z-slice defaults (0.1 m to 1.5 m) preserve the chair-relevant band
the legacy script also used: above floor noise / sloped ground, and
below most ceiling fixtures and door lintels.

PGM pixel convention follows ROS ``map_server``:
    0   = occupied (black)
    254 = free
    205 = unknown
The PGM is written as binary P5 with the 0..255 maxval so map_server
can ingest it without translation.

Usage:

    scripts/m5r_pcd_to_occupancy.py <input.pcd> <output-dir> [options]

Example (matches the M5R-4 bench run):

    scripts/m5r_pcd_to_occupancy.py /tmp/m5r49_dufomap/static.pcd \\
        docs/maps/lab-loop --force

Output (in <output-dir>):
    occupancy.pgm   binary P5, 1 byte / cell
    occupancy.yaml  Nav2 map_server metadata (resolution / origin / thresholds)
"""

import argparse
import sys
from pathlib import Path

import numpy as np


# ROS map_server pixel convention. Keep these as module constants so
# downstream readers (and any future per-cell debug dump) can refer to
# them symbolically instead of re-deriving the magic numbers.
PIX_OCCUPIED = 0
PIX_FREE = 254
PIX_UNKNOWN = 205


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Return an (N, 3) float32 array of XYZ from a PCD (binary or ASCII).

    DUFOMap emits binary PCDs with FIELDS = ``x y z`` (no intensity, no
    RGB). GLIM's per-keyframe staging is ASCII with the same XYZ-only
    layout. We support both formats and ignore any extra fields so the
    converter remains usable on hand-edited PCDs without needing a
    re-encode step. PCL/Open3D are deliberately not pulled in -- the
    parser is small and the only mandatory dependency is numpy, which
    matches the rest of the M5-R script suite.
    """
    with path.open("rb") as f:
        header_lines = []
        while True:
            raw_line = f.readline()
            if not raw_line:
                raise ValueError(f"{path}: unexpected EOF inside PCD header")
            line = raw_line.decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                data_fmt = line.split()[1]
                break
        # Capture binary payload position so we can re-read as bytes
        # later. Doing it now avoids juggling text/binary modes.
        binary_offset = f.tell()
        binary_payload = f.read()

    # Parse header values. Each PCD header field appears on exactly one
    # line; we tolerate them appearing in any order (the spec allows it).
    def header_field(name: str) -> list[str]:
        for line in header_lines:
            if line.startswith(name + " "):
                return line.split()[1:]
        raise ValueError(f"{path}: PCD header missing '{name}'")

    fields = header_field("FIELDS")
    sizes = [int(x) for x in header_field("SIZE")]
    counts = [int(x) for x in header_field("COUNT")]
    types = header_field("TYPE")
    n_points = int(header_field("POINTS")[0])

    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(
            f"{path}: PCD FIELDS {fields!r} missing one of x/y/z"
        )

    # PCD allows SIZE=4 + TYPE=I/U/F. We only handle TYPE F for x/y/z
    # because reading int32 bits as float32 silently corrupts data.
    # DUFOMap, GLIM and our own m5r_glim_to_pcd.py all emit TYPE F F F,
    # so this guard only fires on hand-edited or third-party PCDs.
    for name in ("x", "y", "z"):
        i = fields.index(name)
        if types[i] != "F":
            raise ValueError(
                f"{path}: PCD field {name!r} has TYPE {types[i]!r}; "
                f"only TYPE F (float) is supported"
            )

    if data_fmt == "ascii":
        # Re-read the file as text for the ASCII branch -- doing it in a
        # second pass keeps the binary path zero-copy and the ASCII path
        # straightforward, at the cost of one extra read on ASCII files.
        text = path.read_text().splitlines()
        body = text[len(header_lines):]
        if len(body) < n_points:
            # Without this guard the missing rows would leak through as
            # zero-initialised XYZ from np.empty (which numpy does NOT
            # zero), producing phantom (0, 0, 0) hits near the world
            # origin. Bail early with a clear message instead.
            raise ValueError(
                f"{path}: ASCII PCD body has {len(body)} lines, "
                f"header POINTS={n_points}"
            )
        xyz = np.empty((n_points, 3), dtype=np.float32)
        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
        for i, line in enumerate(body[:n_points]):
            tokens = line.split()
            xyz[i, 0] = float(tokens[xi])
            xyz[i, 1] = float(tokens[yi])
            xyz[i, 2] = float(tokens[zi])
        return xyz

    if data_fmt != "binary":
        # binary_compressed would need lzf decoding; not worth the
        # dependency until something in the pipeline actually emits it.
        raise ValueError(
            f"{path}: DATA {data_fmt!r} unsupported (only ascii/binary)"
        )

    step = sum(s * c for s, c in zip(sizes, counts))
    expected_bytes = step * n_points
    if len(binary_payload) < expected_bytes:
        raise ValueError(
            f"{path}: binary payload {len(binary_payload)} bytes < "
            f"expected {expected_bytes} (POINTS={n_points}, step={step})"
        )

    arr = np.frombuffer(
        binary_payload[:expected_bytes], dtype=np.uint8
    ).reshape(n_points, step)
    xyz = np.empty((n_points, 3), dtype=np.float32)
    offset = 0
    for name, s, c in zip(fields, sizes, counts):
        # Only consume x/y/z; skip everything else by advancing the
        # offset (intensity, rgb, normal_*, etc.).
        if name in ("x", "y", "z") and s == 4 and c == 1:
            idx = "xyz".index(name)
            xyz[:, idx] = np.frombuffer(
                arr[:, offset:offset + 4].tobytes(), dtype=np.float32
            )
        offset += s * c
    # binary_offset is unused after the read but we keep it captured
    # above for readability — if a future debug printout wants to show
    # "header ended at byte N", it is already in scope.
    del binary_offset
    return xyz


def load_trajectory_tum(path: Path) -> np.ndarray:
    """Load TUM-format trajectory (timestamp x y z qx qy qz qw). Return (N, 2) xy.

    Only x, y are used for the grid; z is available in the file but the
    occupancy grid is 2D and does not need it. Comment lines starting
    with '#' are skipped.
    """
    xy = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        toks = line.split()
        if len(toks) < 4:
            continue
        try:
            xy.append((float(toks[1]), float(toks[2])))
        except ValueError:
            continue
    if not xy:
        raise ValueError(f"{path}: no poses parsed (TUM format expected)")
    return np.asarray(xy, dtype=np.float32)


def downsample_trajectory(xy: np.ndarray, min_stride_m: float) -> np.ndarray:
    """Keep points that are at least min_stride_m from the last kept one.

    Preserves the first and last poses. Deterministic (no random skips)
    so re-runs produce byte-identical output given the same input.
    """
    if xy.shape[0] == 0 or min_stride_m <= 0:
        return xy
    kept = [0]
    last = xy[0]
    for i in range(1, xy.shape[0]):
        p = xy[i]
        if np.hypot(p[0] - last[0], p[1] - last[1]) >= min_stride_m:
            kept.append(i)
            last = p
    # Guarantee the last pose is included (loop closure anchor)
    if kept[-1] != xy.shape[0] - 1:
        kept.append(xy.shape[0] - 1)
    return xy[np.asarray(kept, dtype=np.int64)]


def bresenham_rays_to_free_trajectory(
    grid: np.ndarray,
    anchor_pixels: np.ndarray,
    occupied_pixels: np.ndarray,
    max_range_px: float,
    n_angular_bins: int = 720,
) -> int:
    """Multi-anchor walk-to-hit with per-anchor angular dedup.

    For each anchor:
      1. Filter occupied cells to those within max_range_px.
      2. Compute the angle from anchor to each such cell; bin (720 bins).
      3. For each angular bin, keep only the CLOSEST hit — that is the
         cell that would stop a ray in that direction (any farther cell
         is occluded by it, or in a slightly different direction).
      4. Walk DDA from anchor to each retained hit, marking intervening
         UNKNOWN cells as FREE and stopping at any OCCUPIED cell (in
         case another obstacle is between the anchor and the retained
         hit — the sort by (bin, distance) does not guarantee that).
      5. Mark the anchor cell itself as FREE (chair was there).

    Returns the total number of cells flipped UNKNOWN → FREE.
    """
    h, w = grid.shape
    occ_col = occupied_pixels[:, 0].astype(np.float64)
    occ_row = occupied_pixels[:, 1].astype(np.float64)
    max_r_sq = float(max_range_px) ** 2
    bin_step = 2.0 * np.pi / n_angular_bins

    n_marked = 0
    for i in range(anchor_pixels.shape[0]):
        a_col = int(anchor_pixels[i, 0])
        a_row = int(anchor_pixels[i, 1])

        # Mark anchor cell first (chair was here)
        if 0 <= a_col < w and 0 <= a_row < h and grid[a_row, a_col] == PIX_UNKNOWN:
            grid[a_row, a_col] = PIX_FREE
            n_marked += 1

        # Filter occupied cells to within max_range
        dc = occ_col - a_col
        dr = occ_row - a_row
        d_sq = dc * dc + dr * dr
        near_mask = d_sq <= max_r_sq
        if not near_mask.any():
            continue
        near_dc = dc[near_mask]
        near_dr = dr[near_mask]
        near_col = occ_col[near_mask].astype(np.int64)
        near_row = occ_row[near_mask].astype(np.int64)
        near_d = np.sqrt(d_sq[near_mask])

        # Angular bin
        angle = np.arctan2(near_dr, near_dc)  # (-pi, pi]
        bin_idx = ((angle + np.pi) / bin_step).astype(np.int64)
        bin_idx = np.clip(bin_idx, 0, n_angular_bins - 1)

        # Sort by (bin_idx, near_d) and keep the first hit per bin
        order = np.lexsort((near_d, bin_idx))
        bin_sorted = bin_idx[order]
        col_sorted = near_col[order]
        row_sorted = near_row[order]
        _, first_idx = np.unique(bin_sorted, return_index=True)
        keep_col = col_sorted[first_idx]
        keep_row = row_sorted[first_idx]

        # Walk each retained ray
        for j in range(keep_col.shape[0]):
            hc = int(keep_col[j])
            hr = int(keep_row[j])
            steps = max(abs(hc - a_col), abs(hr - a_row))
            if steps < 2:
                # anchor coincides with or is adjacent to the hit; no
                # intervening cells to mark
                continue
            # DDA sampling: steps+1 points along the ray, drop endpoints
            # (anchor and hit). Round preserves the classic Bresenham
            # pixel selection well enough for a 5cm grid.
            xs = np.linspace(a_col, hc, steps + 1)[1:-1].round().astype(np.int64)
            ys = np.linspace(a_row, hr, steps + 1)[1:-1].round().astype(np.int64)
            valid = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
            xs = xs[valid]
            ys = ys[valid]
            if xs.size == 0:
                continue
            vals = grid[ys, xs]
            # Stop at the first cell that is OCCUPIED (an obstacle we did
            # not target directly, but a ray-close-enough hit anyway).
            occ_here = vals == PIX_OCCUPIED
            if occ_here.any():
                first_occ = int(np.argmax(occ_here))
                xs = xs[:first_occ]
                ys = ys[:first_occ]
                if xs.size == 0:
                    continue
            # Mark UNKNOWN → FREE (don't overwrite already-FREE)
            unknown_mask = grid[ys, xs] == PIX_UNKNOWN
            if unknown_mask.any():
                sel_x = xs[unknown_mask]
                sel_y = ys[unknown_mask]
                grid[sel_y, sel_x] = PIX_FREE
                n_marked += int(sel_x.size)

        if (i + 1) % 200 == 0 or i + 1 == anchor_pixels.shape[0]:
            print(
                f"  anchor {i + 1}/{anchor_pixels.shape[0]}  "
                f"({n_marked} cells free so far)",
                file=sys.stderr,
            )

    return n_marked


def bresenham_rays_to_free(
    grid: np.ndarray,
    anchor_px: tuple[int, int],
    hit_pixels: np.ndarray,
) -> None:
    """Mark cells along anchor->hit rays as free, in-place.

    For each unique occupied pixel we walk Bresenham from the anchor
    toward (but excluding) that pixel, flipping intervening UNKNOWN
    cells to FREE. We deliberately do not touch cells that are already
    OCCUPIED: occupancy wins over freeness if a later hit's ray passes
    through an earlier hit. That matches Nav2's costmap occupancy
    semantics (occupied is the safer assumption).

    The single-anchor approximation is documented at module level: a
    per-scan implementation would carry the keyframe poses through this
    stage, which is out of scope for Issue #50.
    """
    anchor_col, anchor_row = anchor_px
    h, w = grid.shape
    for hit_col, hit_row in hit_pixels:
        dx = abs(hit_col - anchor_col)
        dy = -abs(hit_row - anchor_row)
        sx = 1 if anchor_col < hit_col else -1
        sy = 1 if anchor_row < hit_row else -1
        err = dx + dy
        col, row = anchor_col, anchor_row
        while True:
            if col == hit_col and row == hit_row:
                # Stop just before stamping the occupied endpoint as
                # free (the occupied stamp happens outside this loop).
                break
            if 0 <= col < w and 0 <= row < h:
                if grid[row, col] == PIX_UNKNOWN:
                    grid[row, col] = PIX_FREE
                elif grid[row, col] == PIX_OCCUPIED:
                    # Occlusion: a real obstacle blocks this ray.
                    # Anything beyond it is not "definitely free" so
                    # stop walking.
                    break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                col += sx
            if e2 <= dx:
                err += dx
                row += sy


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a DUFOMap static PCD into a Nav2-compatible 2D "
            "occupancy grid (occupancy.pgm + occupancy.yaml). Part of "
            "the M5-R map pipeline (Issue #50)."
        ),
    )
    parser.add_argument(
        "input_pcd", type=Path,
        help="Input static PCD (binary or ASCII, XYZ fields required)",
    )
    parser.add_argument(
        "output_dir", type=Path,
        help="Output directory; occupancy.pgm and occupancy.yaml are "
             "written into it. Typically docs/maps/<site>/.",
    )
    parser.add_argument(
        "--resolution", type=float, default=0.05,
        help="Cell size in metres (default 0.05, matches "
             "_template/occupancy.yaml)",
    )
    # Z slice defaults preserve the chair-relevant obstacle band the
    # legacy M5-b script also used. Above this floor noise and sloped
    # ground get baked in; below the cap, door lintels and signage end
    # up as phantom obstacles even though the chair passes underneath.
    parser.add_argument(
        "--z-min", type=float, default=0.1,
        help="Lower Z slice [m] (default 0.1, excludes floor noise)",
    )
    parser.add_argument(
        "--z-max", type=float, default=1.5,
        help="Upper Z slice [m] (default 1.5, excludes lintels / signs)",
    )
    parser.add_argument(
        "--anchor-mode", choices=("trajectory", "single"), default="trajectory",
        help="Ray-cast anchor selection. 'trajectory' (default) reads "
             "traj_lidar.txt and casts from each downsampled pose within "
             "--max-range. 'single' keeps the legacy behaviour (occupied "
             "centroid or --anchor-x/y). Use single for offline conversions "
             "where the trajectory is unavailable.",
    )
    parser.add_argument(
        "--traj", type=Path, default=None,
        help="Path to traj_lidar.txt (TUM format). Default: look in "
             "<output_dir>/traj_lidar.txt, then <input_pcd_dir>/"
             "traj_lidar.txt. Only used when --anchor-mode=trajectory.",
    )
    parser.add_argument(
        "--traj-stride", type=float, default=1.0,
        help="Downsample stride [m] for trajectory anchors (default 1.0 m; "
             "the raw GLIM traj is ~0.06 m per pose so this drops the "
             "anchor count to ~loop_length_m).",
    )
    parser.add_argument(
        "--max-range", type=float, default=20.0,
        help="Per-anchor raycast radius [m] (default 20 m, matches VLP-16 "
             "effective range for indoor/near-outdoor scenes). Only used "
             "when --anchor-mode=trajectory.",
    )
    parser.add_argument(
        "--anchor-x", type=float, default=None,
        help="Ray-cast anchor X [m] for --anchor-mode=single "
             "(default: occupied-cell XY centroid)",
    )
    parser.add_argument(
        "--anchor-y", type=float, default=None,
        help="Ray-cast anchor Y [m] for --anchor-mode=single "
             "(default: occupied-cell XY centroid)",
    )
    parser.add_argument(
        "--no-raycast", action="store_true",
        help="Skip the Bresenham free-marking pass; cells are either "
             "occupied or unknown. Useful for debugging or when "
             "track_unknown_space=false in Nav2.",
    )
    # 0.196 matches Nav2 map_server's documented default and the
    # _template/occupancy.yaml committed under #47. The legacy
    # pcd_to_occupancy_grid.py wrote 0.25, which is a *Nav1*-era value;
    # we deliberately do not propagate that here.
    parser.add_argument(
        "--occupied-thresh", type=float, default=0.65,
        help="YAML occupied_thresh (default 0.65, Nav2 default)",
    )
    parser.add_argument(
        "--free-thresh", type=float, default=0.196,
        help="YAML free_thresh (default 0.196, Nav2 default)",
    )
    parser.add_argument(
        "--padding", type=float, default=2.0,
        help="Padding [m] added outside the XY bounding box so edge "
             "obstacles are not clipped (default 2.0)",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Overwrite existing occupancy.pgm / occupancy.yaml. "
             "Default is to abort, matching the rest of the M5-R "
             "script suite.",
    )
    args = parser.parse_args()

    if not args.input_pcd.is_file():
        print(f"ERROR: input PCD {args.input_pcd} not found",
              file=sys.stderr)
        return 1

    out_dir: Path = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / "occupancy.pgm"
    yaml_path = out_dir / "occupancy.yaml"

    if not args.force:
        existing = [p for p in (pgm_path, yaml_path) if p.exists()]
        if existing:
            names = ", ".join(p.name for p in existing)
            print(
                f"ERROR: {names} already exists under {out_dir}. "
                f"Re-run with --force to overwrite.",
                file=sys.stderr,
            )
            return 1

    print(f"Reading {args.input_pcd} ...", file=sys.stderr)
    try:
        xyz = read_pcd_xyz(args.input_pcd)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"Loaded {len(xyz)} points", file=sys.stderr)

    z_mask = (xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)
    sliced = xyz[z_mask]
    print(
        f"Z-sliced to [{args.z_min:.2f}, {args.z_max:.2f}]: "
        f"{len(sliced)} points retained",
        file=sys.stderr,
    )
    if len(sliced) == 0:
        print(
            "ERROR: no points survived Z-slice. Check --z-min / --z-max "
            "against the PCD bounding box.",
            file=sys.stderr,
        )
        return 1

    xmin = float(sliced[:, 0].min())
    xmax = float(sliced[:, 0].max())
    ymin = float(sliced[:, 1].min())
    ymax = float(sliced[:, 1].max())

    # In trajectory mode we also need the trajectory to fit inside the
    # grid — otherwise anchors in open areas (no z=[0.1, 1.5] obstacle
    # observations near them) drop off the grid and 15%+ of the traversal
    # cannot be represented. Load traj_lidar.txt here so its bbox joins
    # the point cloud's before we compute grid dimensions. If loading
    # fails we still print the resolved path for the user to fix.
    traj_xy = None
    traj_path_resolved: Path | None = None
    if args.anchor_mode == "trajectory" and not args.no_raycast:
        if args.traj is not None:
            traj_path_resolved = args.traj
        else:
            candidates = [
                out_dir / "traj_lidar.txt",
                args.input_pcd.parent / "traj_lidar.txt",
            ]
            traj_path_resolved = next(
                (p for p in candidates if p.is_file()), None
            )
            if traj_path_resolved is None:
                print(
                    "ERROR: trajectory mode selected but traj_lidar.txt not "
                    "found in\n"
                    f"  {candidates[0]}\n"
                    f"  {candidates[1]}\n"
                    "Pass --traj PATH, or copy the GLIM output traj_lidar.txt "
                    "next to static.pcd, or use --anchor-mode single.",
                    file=sys.stderr,
                )
                return 1
        try:
            traj_xy = load_trajectory_tum(traj_path_resolved)
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        xmin = min(xmin, float(traj_xy[:, 0].min()))
        xmax = max(xmax, float(traj_xy[:, 0].max()))
        ymin = min(ymin, float(traj_xy[:, 1].min()))
        ymax = max(ymax, float(traj_xy[:, 1].max()))

    pcd_bbox = (
        float(sliced[:, 0].min()), float(sliced[:, 0].max()),
        float(sliced[:, 1].min()), float(sliced[:, 1].max()),
    )
    xmin -= args.padding
    xmax += args.padding
    ymin -= args.padding
    ymax += args.padding
    res = args.resolution
    width = int(np.ceil((xmax - xmin) / res))
    height = int(np.ceil((ymax - ymin) / res))
    print(
        f"Grid: {width} x {height} cells @ {res} m  "
        f"(pcd bbox x:[{pcd_bbox[0]:.2f}, {pcd_bbox[1]:.2f}], "
        f"y:[{pcd_bbox[2]:.2f}, {pcd_bbox[3]:.2f}]" +
        (f" ∪ traj bbox x:[{float(traj_xy[:, 0].min()):.2f}, "
         f"{float(traj_xy[:, 0].max()):.2f}], "
         f"y:[{float(traj_xy[:, 1].min()):.2f}, {float(traj_xy[:, 1].max()):.2f}]"
         if traj_xy is not None else "") +
        f", padding {args.padding} m)",
        file=sys.stderr,
    )

    # Guard against pathological inputs producing multi-GB grids.
    # 100 M cells = ~100 MB at 1 byte/cell. Pick that as a soft cap and
    # bail with a helpful message rather than OOM-killing the host.
    if width * height > 100_000_000:
        print(
            f"ERROR: grid would be {width * height} cells "
            f"(>{100_000_000} cap). Use a coarser --resolution or "
            f"narrower --z-min/--z-max (or pre-crop the PCD).",
            file=sys.stderr,
        )
        return 1

    grid = np.full((height, width), PIX_UNKNOWN, dtype=np.uint8)

    # World -> pixel mapping. PGM row 0 is the top of the image but our
    # YAML origin is the bottom-left corner (Nav2 map_server convention),
    # so we flip the row axis.
    cols = ((sliced[:, 0] - xmin) / res).astype(np.int64)
    rows = height - 1 - ((sliced[:, 1] - ymin) / res).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows = cols[valid], rows[valid]
    # Unique occupied cells (used both for anchor centroid and for the
    # ray-cast pass).
    occupied = np.unique(np.stack([cols, rows], axis=1), axis=0)

    # Convert a world (x, y) to grid pixel (col, row).
    def world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
        col = int(round((wx - xmin) / res))
        row = height - 1 - int(round((wy - ymin) / res))
        return col, row

    if args.no_raycast:
        print("Skipping ray-cast (--no-raycast).", file=sys.stderr)
    elif args.anchor_mode == "trajectory":
        # traj_xy was already loaded above (joined into the bbox). Reuse.
        assert traj_xy is not None, "trajectory mode reached without traj_xy"
        print(f"Using trajectory {traj_path_resolved}", file=sys.stderr)
        n_raw = traj_xy.shape[0]
        traj_ds = downsample_trajectory(traj_xy, args.traj_stride)
        n_ds = traj_ds.shape[0]
        # World → pixel for each anchor
        anchor_cols = np.round((traj_ds[:, 0] - xmin) / res).astype(np.int64)
        anchor_rows = height - 1 - np.round((traj_ds[:, 1] - ymin) / res).astype(np.int64)
        anchor_px = np.stack([anchor_cols, anchor_rows], axis=1)
        # Drop anchors outside the grid (rare but possible if padding is
        # tighter than the trajectory extent).
        in_grid = (
            (anchor_cols >= 0) & (anchor_cols < width)
            & (anchor_rows >= 0) & (anchor_rows < height)
        )
        n_out = int((~in_grid).sum())
        anchor_px = anchor_px[in_grid]
        max_range_px = args.max_range / res

        print(
            f"Trajectory: {n_raw} raw poses → {n_ds} anchors "
            f"(stride {args.traj_stride} m); {anchor_px.shape[0]} inside grid "
            f"({n_out} dropped)",
            file=sys.stderr,
        )
        print(
            f"Ray-casting: max_range={args.max_range} m "
            f"({max_range_px:.1f} px), 720 angular bins per anchor",
            file=sys.stderr,
        )
        n_marked = bresenham_rays_to_free_trajectory(
            grid, anchor_px, occupied, max_range_px, n_angular_bins=720,
        )
        print(f"Trajectory raycast complete: {n_marked} cells marked free.",
              file=sys.stderr)
    else:  # single mode (legacy)
        if args.anchor_x is not None and args.anchor_y is not None:
            anchor_world = (args.anchor_x, args.anchor_y)
            anchor_source = "user"
        elif args.anchor_x is not None or args.anchor_y is not None:
            print(
                "ERROR: --anchor-x and --anchor-y must be passed together",
                file=sys.stderr,
            )
            return 1
        else:
            # Centroid of occupied cells in world coords. This is a coarse
            # but pose-free proxy for "somewhere the sensor was likely able
            # to see from". A U-shaped traverse can put the centroid outside
            # the navigable region -- the user should pass an explicit
            # --anchor-{x,y} in that case.
            cx_px = float(occupied[:, 0].mean())
            cy_px = float(occupied[:, 1].mean())
            ax = xmin + (cx_px + 0.5) * res
            ay = ymin + (height - 1 - cy_px + 0.5) * res
            anchor_world = (ax, ay)
            anchor_source = "auto, occupied centroid"

        anchor_col, anchor_row = world_to_pixel(*anchor_world)
        print(
            f"Anchor ({anchor_source}): "
            f"({anchor_world[0]:.2f}, {anchor_world[1]:.2f})",
            file=sys.stderr,
        )
        anchor_in_grid = 0 <= anchor_col < width and 0 <= anchor_row < height
        if not anchor_in_grid:
            print(
                f"WARNING: anchor pixel ({anchor_col}, {anchor_row}) is "
                f"outside the grid (0..{width-1}, 0..{height-1}); "
                "skipping ray-cast — all cells will be OCCUPIED or UNKNOWN.",
                file=sys.stderr,
            )
        else:
            print(
                f"Ray-casting (single) from anchor to {len(occupied)} "
                f"occupied cells...",
                file=sys.stderr,
            )
            bresenham_rays_to_free(grid, (anchor_col, anchor_row), occupied)

    # Stamp occupied last so free-marking from later rays cannot
    # overwrite a real obstacle (defence in depth -- the ray walker
    # already respects this, but the explicit stamp removes any doubt).
    grid[rows, cols] = PIX_OCCUPIED

    n_occ = int((grid == PIX_OCCUPIED).sum())
    n_free = int((grid == PIX_FREE).sum())
    n_unk = int((grid == PIX_UNKNOWN).sum())
    print(
        f"Occupied: {n_occ} cells   Free: {n_free} cells   "
        f"Unknown: {n_unk} cells",
        file=sys.stderr,
    )

    # PGM P5 binary. The comment line carries the producer name so a
    # future debug session can grep the file and find the script that
    # made it. We deliberately omit timestamps / input paths from the
    # header: keeping the output byte-identical for identical inputs
    # preserves the idempotency contract documented in the help text.
    with pgm_path.open("wb") as f:
        f.write(b"P5\n")
        f.write(b"# m5r_pcd_to_occupancy.py output\n")
        f.write(f"{width} {height}\n".encode("ascii"))
        f.write(b"255\n")
        f.write(grid.tobytes())
    print(
        f"Wrote {pgm_path} (1 byte/pixel, {pgm_path.stat().st_size} bytes)"
    )

    # YAML output. Field order mirrors docs/maps/_template/occupancy.yaml
    # so a side-by-side diff highlights only the values that differ
    # (origin + the image filename if a future caller wants to override).
    yaml_text = (
        f"# Generated by scripts/m5r_pcd_to_occupancy.py from "
        f"{args.input_pcd.name}\n"
        f"image: occupancy.pgm\n"
        f"resolution: {res}\n"
        f"origin: [{xmin:.3f}, {ymin:.3f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: {args.occupied_thresh}\n"
        f"free_thresh: {args.free_thresh}\n"
    )
    yaml_path.write_text(yaml_text)
    print(f"Wrote {yaml_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
